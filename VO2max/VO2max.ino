//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Spirometrie Project
// https://www.instructables.com/Accurate-VO2-Max-for-Zwift-and-Strava/
// BLE by Andreas Spiess https://github.com/SensorsIot/Bluetooth-BLE-on-Arduino-IDE
// Modifications by Ulrich Rissel and Ivor Hewitt
//
// TTGO T-Display: SDA-Pin21, SCL-Pin22
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

const String Version = "V2.5.1 2025/01/12";

/*Board: ESP32 Dev Module
Upload Speed: 921600
CPU Frequency: 240Mhz (WiFi/BT)
Flash Frequency: 80Mhz
Flash Mode: QIO
Flash Size: 4MB (32Mb)
Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
Core Debug Level: None
PSRAM: Disabled*/

// Libraries:
//   Sensirion_I2C_STC3x : 1.0.1
//   SparkFun_SHTC3_Humidity_and_Temperature_Sensor_Library : 1.1.4
//   Omron_D6F-PH_Arduino_Library : 1.1.0
//   TFT_eSPI : 2.5.43
//   Adafruit_BME280_Library : 2.2.4
//   Adafruit_BMP085_Library : 1.2.4
//   Adafruit_BMP280_Library : 2.6.8
//   Sensirion_Gadget_BLE_Arduino_Lib : 1.4.1
//   NimBLE-Arduino : 2.1.2
//   DFRobot_OxygenSensor : 1.0.1

/* Note: In  Arduino/libraries/TFT_eSPI/User_Setup_Select.h
 * make sure to uncomment the t-display driver line (Setup25) */

/// ############################################################################################
/// MACROS TO BE DEFINED IN CONFIG.H
/// ############################################################################################
#include "config.h"

#include "esp_adc_cal.h" // ADC calibration data
#include <EEPROM.h>      // include library to read and write settings from flash
#define ADC_EN  14       // ADC_EN is the ADC detection enable port
#define ADC_PIN 34
int vref = 1100;

#ifdef OXYSENSOR
#include "DFRobot_OxygenSensor.h" //Library for Oxygen sensor
#elif !defined(STC_31) && !defined(SCD_30)
#error "A CO2 sensor must be enabled if no OXY sensor"
#endif

#ifdef SCD_30      // original SCD_30 sensor (inadequate - max 4% co2)
#include "SCD30.h" //declares "SCD_30 scd30"
#elif defined(STC_31)
#include "SensirionI2cStc3x.h" //Use Sensirion library
SensirionI2cStc3x stc3x_sensor;
#include "SparkFun_SHTC3.h"    //Use sparkfun shtc3 library
SHTC3             mySHTC3;
#endif

#include "hal/gpio_ll.h"
#include <Omron_D6FPH.h> //Library for pressure sensor
#include <SPI.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip, see note above (line 20)
#include <Wire.h>

// NOTE: If GadgetBLE doesn't compile because of setMinPreferred/MaxPreferred removed,
// replace GadgetBLE code with:
// _data->pNimBLEAdvertising->setPreferredParams(0x06, 0x12);
#ifdef GADGET
#include "Sensirion_Gadget_BLE.h" //library to publish to Sensirion 'gadget' App
#endif

// declarations for bluetooth serial --------------
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

// declarations for BLE ---------------------
#include <BLE2902.h> // used for notifications 0x2902: Client Characteristic Configuration
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

byte bpm;
byte heart[8] = {0b00001110, 60, 0, 0, 0, 0, 0, 0}; // defines the BT heartrate characteristic

// Byte[0]: flags: 0b00001110:
// not used/n.u./n.u./RR value available/Energy val.av./
// Sensor contact status/Sens.cont.supported/HR Format: (0: uint_8)
// Byte[1]: HR (uint_8)
// Byte[2]: Energy in J MSB
// Byte[3]: Energy in J LSB
// Byte[4]: RR
// Byte[5]: RR
// Byte[6]: ?
// Byte[7]: ?

byte hrmPos[1] = {2};

bool _BLEClientConnected = false;

#define batteryLevelServiceId BLEUUID((uint16_t)0x180F)
BLECharacteristic batteryLevelCharacteristics(BLEUUID((uint16_t)0x2A19),
                                              BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

// heart rate service
#define heartRateService BLEUUID((uint16_t)0x180D)
BLECharacteristic heartRateMeasurementCharacteristics(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A38), BLECharacteristic::PROPERTY_READ);
BLEDescriptor     heartRateDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor     sensorPositionDescriptor(BLEUUID((uint16_t)0x2901)); // 0x2901: Characteristic User Description

struct { // variables for GoldenCheetah
    short freq;
    byte  temp;
    byte  hum;
    short rmv;
    short feo2;
    short vo2;
} cheetah;

// GoldenCheetah service
// Publish to golden cheetah as a 'vo2master'
#define cheetahService BLEUUID("00001523-1212-EFDE-1523-785FEABCD123")
BLECharacteristic cheetahCharacteristics(BLEUUID("00001524-1212-EFDE-1523-785FEABCD123"), //
                                         BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor     cheetahDescriptor(BLEUUID((uint16_t)0x2901));

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) { _BLEClientConnected = true; };

    void onDisconnect(BLEServer *pServer) {
        BLEDevice::startAdvertising();
        _BLEClientConnected = false;
    }
};

#if defined(BMP085)
#include <Adafruit_BMP085.h> //Library for barometric sensor
Adafruit_BMP085 bmp;
#elif defined(BME280)
#include <Adafruit_BME280.h> //Library for barometric sensor
Adafruit_BME280 bmp;
#elif defined(BMP280)
#include <Adafruit_BMP280.h> //Library for barometric sensor
Adafruit_BMP280 bmp;
#endif

bool co2Enabled = false;
bool bmpEnabled = false;

// Starts Screen for TTGO device
TFT_eSPI tft = TFT_eSPI(); // Invoke library, pins defined in User_Setup.h

// Labels the pressure sensor: mySensor
Omron_D6FPH mySensor;

// Label of oxygen sensor
#ifdef OXYSENSOR
DFRobot_OxygenSensor Oxygen;
#define COLLECT_NUMBER    10        // collect number, the collection range is 1-100.
#define Oxygen_IICAddress ADDRESS_3 // I2C  label for o2 address
#endif

// Defines button state for adding wt
const int buttonPin1 = 0;
const int buttonPin2 = 35;
int       wtTotal = 0;
int       buttonPushCounter1 = 0; // counter for the duration of button1 pressed
int       buttonState1 = 1;       // current state of the button
int       buttonPushCounter2 = 0; // counter for the duration of button2 pressed
int       buttonState2 = 1;       // current state of the button
int       screenChanged = 0;
int       screenNr = 1;
int       HeaderStreamed = 0;
int       HeaderStreamedBT = 0;

// ############################################
//  Select correct diameter depending on printed
//  case dimensions:
// ############################################

// Defines the size of the Venturi openings for the  calculations of AirFlow
const float area_1 = 0.000531; // = 26mm diameter
#if (DIAMETER == 20)
const float area_2 = 0.000314; // = 20mm diameter
#elif (DIAMETER == 19)
const float area_2 = 0.000284; // = 19mm diameter
#else // default
const float area_2 = 0.000201; // = 16mm diameter
#endif

// Air density values
const float dryConstant = 287.058;
const float wetConstant = 461.495;

// Sensible defaults to use with no barometers
const float rhoSTPD = 1.292; // STPD conditions: density at 0°C, MSL, 1013.25 hPa, dry air
float       rhoATPS = 1.225; // ATP conditions: density based on ambient conditions, dry air
float       rhoBTPS = 1.123; // BTPS conditions: density at ambient  pressure, 35°C, 95% humidity

// Alternative ATPS for room temperature - 20c, 50% = 1.199

// Default ambient values for standard ATPS
float TempC = 15.0;    // Air temperature in Celsius
float PresPa = 101325; // uncorrected (absolute) barometric pressure
float Humid = 0;       // dry air

float massFlow = 0.0;
float volFlow = 0.0;
float volumeTotal = 0.0;    // variable for holding total volume of breath
float pressure = 0.0;       // differential pressure of the venturi nozzle
float pressThreshold = 0.2; // threshold for starting calculation of VE
float volumeVE = 0.0;
float volumeVEmean = 0.0;
float volumeExp = 0.0;

// Basic defaults in settings, saved to eeprom
struct {
    int   version = 1;            // Make sure saved data is right version
    float correctionSensor = 1.0; // calculated from 3L calibration syringe
    float weightkg = 75.0;        // Standard-body-weight
    bool  heart_on = false;       // Output vo2 as a HRM
    bool  sens_on = true;         // Output as sensiron data
    bool  serialbt = false;
    bool  cheet_on = false; // Output as vo2master for GoldenCheetah
    bool  co2_on = false;   // CO2 sensor active
} settings;

unsigned long TimerVolCalc = 0;
unsigned long Timer5s = 0;
unsigned long Timer30s = 0;
unsigned long TimerVO2calc = 0;
unsigned long TimerVO2diff = 0; // used for integral of calories
unsigned long TimerStart = 0;
unsigned long TotalTime = 0;
String        TotalTimeMin = String("00:00");
int           readVE = 0;
unsigned long TimerVE = 0;
unsigned long DurationVE = 0;

float lastO2 = 0;
float baselineO2 = 0;
float co2 = 0;
float calTotal = 0;
float vo2Cal = 0;
float vo2CalH = 0;        // calories per hour
float vo2CalDay = 0.0;    // calories per day
float vo2CalDayMax = 0.0; // highest value of calories per day
float vo2Max = 0;         // value of vo2Max/min/kg, calculated every 30 seconds
float vo2Total = 0.0;     // value of total vo2Max/min
float vo2MaxMax = 0;      // Best value of vo2 max for whole time machine is on

float respq = 0.0; // respiratory quotient in mol VCO2 / mol VO2

#ifdef SCD_30
float co2ppm = 0.0; // CO2 sensor in ppm
#endif

float co2perc = 0.0;     // = CO2ppm /10000
float baselineCO2 = 0.0; // initial value of CO2 in ppm
float vco2Total = 0.0;
float vco2Max = 0.0;
float co2temp = 0.0; // temperature CO2 sensor
float co2hum = 0.0;  // humidity CO2 sensor (not used in calculations)

float freqVE = 0.0;     // ventilation frequency
float freqVEmean = 0.0; // mean ventilation frequency

float expiratVol = 0.0; // last expiratory volume in L
float volumeTotalOld = 0.0;
float volumeTotal2 = 0.0;

float Battery_Voltage = 0.0;

// settings for Sensirion App
#ifdef GADGET
NimBLELibraryWrapper lib;
DataProvider         provider(lib, DataType::T_RH_CO2_ALT);
#endif

// STC_31 calc
#ifdef STC_31
static float frcReferenceValue = 0.0;
uint16_t     signalRawGasConcentration(float gasConcentration) { return (uint16_t)gasConcentration * 327.68 + 16384.0; }
#endif

//----------------------------------------------------------------------------------------------------------
//                  SETUP
//----------------------------------------------------------------------------------------------------------

void loadSettings() {
    // Check version first.
    int version = EEPROM.read(0);
    if (version == settings.version) {
        for (int i = 0; i < sizeof(settings); ++i)
            ((byte *)&settings)[i] = EEPROM.read(i);
    }
}

void saveSettings() {
    bool changed = false;
    for (int i = 0; i < sizeof(settings); ++i) {
        byte b = EEPROM.read(i);
        if (b != ((byte *)&settings)[i]) {
            EEPROM.write(i, ((byte *)&settings)[i]);
            changed = true;
        }
    }
    if (changed) EEPROM.commit();
}

//----------------------------
void setup() {
    EEPROM.begin(sizeof(settings));

    pinMode(buttonPin1, INPUT_PULLUP);
    pinMode(buttonPin2, INPUT_PULLUP);

    // defines ADC characteristics for battery voltage
    /*
      ADC_EN is the ADC detection enable port
      If the USB port is used for power supply, it is turned on by default.
      If it is powered by battery, it needs to be set to high level
    */
    // setup for analog digital converter used for battery voltage ---------
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    // Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        // Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        // Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n",
        // adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        // Serial.println("Default Vref: 1100mV");
    }

    // init display ----------
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    readVoltage();
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("VO2max", 0, 25, 4);
    tft.drawString(Version, 0, 50, 4);
    tft.drawString("Initialising...", 0, 75, 4);
    delay(1000);
    tft.fillScreen(TFT_BLACK);

    // init serial communication  ----------
    Wire.begin();
    Serial.begin(9600); // drop to 9600 to see if improves reliability
    if (!Serial) {
        tft.drawString("Serial ERROR!", 0, 0, 4);
    } else {
        tft.drawString("Serial ok", 0, 0, 4);
    }

#if defined(BMP085) || defined(BME280) || defined(BMP280)
// init barometric sensor ----------
#if defined(BMP280)
    bmpEnabled = bmp.begin(0x76);
#else
    bmpEnabled = bmp.begin(0x76, &Wire);
#endif
    if (!bmpEnabled) {
#ifdef VERBOSE
        Serial.println("Unable to initialise bmp sensor");
#endif
        tft.drawString("Temp/Pres. Error!", 0, 25, 4);
    } else {
        tft.drawString("Temp/Pres. ok", 0, 25, 4);
    }
#endif

    // TODO cleanup/simplify output of messages to screen

#ifdef STC_31
    stc3x_sensor.begin(Wire, 0x29);
    delay(20);
    uint32_t id;
    uint64_t serial;
    stc3x_sensor.prepareProductIdentifier();
    stc3x_sensor.getProductId(id, serial);

    int gas;

    // TODO change logging/errors to display on tft
    if (id == 0x8010304) // stc31-c
    {
        Serial.println("STC_31-C");
        gas = 0x13; // standard - filter recommended
        // gas = 0x03; // low noise - filter not needed
    } else // 0x8010301 //stc31
    {
        Serial.println("STC_31");
        gas = 0x03;
    }
    if (0 != stc3x_sensor.setBinaryGas(gas)) Serial.println("Unable to access stc3x");

    if (mySHTC3.begin() != SHTC3_Status_Nominal) {
        Serial.println(F("SHTC3 not detected. Please check wiring. Freezing..."));
        while (1)
            ;
    }
    if (mySHTC3.update() != SHTC3_Status_Nominal) // Request a measurement
    {
        Serial.println(F("Could not read the RH and T from the SHTC3! Freezing..."));
        while (1)
            ;
    }
    float temperature;
    float RH;

    stc3x_sensor.disableAutomaticSelfCalibration();

    float prevCo2 = 0.0;
    float stc3xCo2Concentration = 0.0;
    float stc3xTemperature = 0.0;

    // stabilise CO2 sensor
    // TODO move co2 stabilize/calibrate into idle/waiting state loop
    tft.setCursor(0, 50, 4);
    tft.println("Stablise");
    // datasheet suggests 20s?
    for (int i = 0; i < 20; i++) {
        temperature = mySHTC3.toDegC(); // "toDegC" returns the temperature
        RH = mySHTC3.toPercent();       // "toPercent" returns the percent humidity
        PresPa = bmp.readPressure();    // pressure in pa

        stc3x_sensor.setRelativeHumidity(RH);
        stc3x_sensor.setTemperature(temperature);
        stc3x_sensor.setPressure(PresPa / 100);

        stc3x_sensor.measureGasConcentration(stc3xCo2Concentration, stc3xTemperature);
        tft.setCursor(0, 75, 4);
        tft.print(stc3xCo2Concentration);
        tft.print("% ");
        tft.print(prevCo2 - stc3xCo2Concentration);
        tft.println("% ");

        prevCo2 = stc3xCo2Concentration;
        delay(1000);
    }
    tft.setCursor(0, 50, 4);
    tft.println("Calibrate");

    do { // shouldnt need loop
        uint16_t frcReferenceValueRaw = signalRawGasConcentration(frcReferenceValue);
        stc3x_sensor.forcedRecalibration(frcReferenceValueRaw);

        delay(1000);
        stc3x_sensor.measureGasConcentration(stc3xCo2Concentration, stc3xTemperature);
        tft.setCursor(0, 75, 4);
        tft.print(stc3xCo2Concentration);
        tft.println("%  ");

    } while (stc3xCo2Concentration > 0.04); // shouldnt be necessary after recal
    co2Enabled = true;
    tft.setCursor(0, 50, 4);
    tft.println("                ");
    tft.setCursor(0, 75, 4);
    tft.println("                ");
    tft.drawString("CO2 ok  ", 120, 50, 4);
#endif

#ifdef SCD_30
    // init CO2 sensor Sensirion SCD_30 -------------
    scd30.initialize();
    scd30.setAutoSelfCalibration(0);
    while (!scd30.isAvailable()) {
        tft.drawString("CO2init...", 120, 50, 4);
    }
    co2Enabled = true;
    tft.drawString("CO2 ok   ", 120, 50, 4);
#endif

    // init O2 sensor DF-Robot -----------
#ifdef OXYSENSOR
    if (!Oxygen.begin(Oxygen_IICAddress)) {
        tft.drawString("O2 ERROR!", 0, 50, 4);
    } else {
        tft.drawString("O2 ok   ", 0, 50, 4);
    }
#endif

    // init flow/pressure sensor Omron D6F-PF0025AD1 (or D6F-PF0025AD2) ----------
    while (!mySensor.begin(MODEL_0025AD1)) {
        // Serial.println("Flow sensor error!");
        tft.drawString("Flow-Sensor ERROR!", 0, 75, 4);
    }
    // Serial.println("Flow-Sensor I2c connect success!");
    tft.drawString("Flow-Sensor ok", 0, 75, 4);

    delay(2000);

    // Get/check settings
    doMenu();

    if (settings.cheet_on || settings.heart_on) {
        InitBLE(); // Now initialise BLE output
    }

    showParameters();

    // activate Sensirion App ----------
#ifdef GADGET
    if (settings.sens_on) {
        provider.begin();
#ifdef VERBOSE
        Serial.print("Sensirion GadgetBle Lib initialized with deviceId = ");
        Serial.println(provider.getDeviceIdString());
#endif
    }
#endif

    // init serial bluetooth -----------
    if (settings.serialbt) {
        if (!SerialBT.begin("VO2max")) { // Start Bluetooth with device name
            tft.drawString("BT NOT ready!", 0, 100, 4);
        } else {
            tft.drawString("BT ready", 0, 100, 4);
        }
    }

#ifdef OXYSENSOR
    CheckInitialO2();
#else
    baselineO2 = 20.9;
    settings.co2_on = true; // force on
#endif

    if (settings.co2_on && co2Enabled) {
        //
        CheckInitialCO2();
    }

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    tft.drawCentreString("Ready...", 120, 55, 4);

    TimerVolCalc = millis(); // timer for the volume (VE) integral function
    Timer5s = millis();
    Timer30s = millis();
    TimerVO2calc = millis(); // timer between VO2max calculations
    TimerStart = millis();   // holds the millis at start
    TotalTime = 0;
    // ++++++++++++++++++++++++++++++++++++++++++++
}

//----------------------------------------------------------------------------------------------------------
//                  MAIN PROGRAM
//----------------------------------------------------------------------------------------------------------

void loop() {
    TotalTime = millis() - TimerStart; // calculates actual total time
    VolumeCalc();                      // Starts integral function

    // VO2max calculation, tft display and excel csv every 5s --------------
    if ((millis() - TimerVO2calc) > 5000 &&
        pressure < pressThreshold) { // calls vo2maxCalc() for calculation Vo2Max every 5 seconds.
        TimerVO2diff = millis() - TimerVO2calc;
        TimerVO2calc = millis(); // resets the timer

        vo2maxCalc(); // vo2 max function call

        showScreen();
        volumeTotal2 = 0; // resets volume2 to 0 (used for initial 10s sensor test)
        readVoltage();
#ifdef VERBOSE
        DataStream(false); // send csv data via wired com port
#endif
        if (settings.serialbt) {
            DataStream(true); // send csv data via Bluetooth com port
        }
        // how often to broadcast? is 5s too slow?
#ifdef GADGET
        if (settings.sens_on) GadgetWrite(); // Send to sensirion
#endif
        if (settings.cheet_on) VO2Notify(); // Send to GoldenCheetah as VO2 Master

        // send BLE data ----------------
        bpm = int(vo2Max + 0.5);
        heart[1] = (byte)bpm;

        int energyUsed = calTotal * 4.184; // conversion kcal into kJ
        heart[3] = energyUsed / 256;
        heart[2] = energyUsed - (heart[3] * 256);

        delay(100);

        if (settings.heart_on && _BLEClientConnected) {
            heartRateMeasurementCharacteristics.setValue(heart, 8); // set the new value for heartrate
            heartRateMeasurementCharacteristics.notify();           // send a notification that value has changed

            sensorPositionCharacteristic.setValue(hrmPos, 1);
        }
    }

    ReadButtons();
    if (buttonPushCounter1 > 20 && buttonPushCounter2 > 20) ESP.restart();
    if (buttonPushCounter1 == 2) {
        screenNr--;
        screenChanged = 1;
    }
    if (buttonPushCounter2 == 2) {
        screenNr++;
        screenChanged = 1;
    }
    if (screenNr < 1) screenNr = 6;
    if (screenNr > 6) screenNr = 1;
    if (screenChanged == 1) {
        showScreen();
        screenChanged = 0;
    }

    if (millis() - Timer30s > 30000) {
        Timer30s = millis(); // reset timer
        BatteryBT();
    }

    TimerVolCalc = millis(); // part of the integral function to keep calculation volume over time
    // Resets amount of time between calcs
#ifdef GADGET
    if (settings.sens_on) provider.handleDownload();
#endif
}

//----------------------------------------------------------------------------------------------------------
//                  FUNCTIONS
//----------------------------------------------------------------------------------------------------------

#ifdef OXYSENSOR
void CheckInitialO2() {
    // check initial O2 value -----------
    baselineO2 = Oxygen.getOxygenData(COLLECT_NUMBER); // read and check initial VO2%
    bool low = false;

    if (baselineO2 < 20.00) {
        // Clear colour once we're above 20
        while (digitalRead(buttonPin1)) {
            if (!low && baselineO2 < 20) {
                tft.fillScreen(TFT_RED);
                tft.setTextColor(TFT_WHITE, TFT_RED);
                tft.setCursor(5, 5, 4);
                tft.println("INITIAL O2% LOW!");
                tft.setCursor(5, 30, 4);
                tft.println("Wait to continue!");
                low = true;

            } else if (low && baselineO2 >= 20) {
                tft.fillScreen(TFT_GREEN);
                tft.setTextColor(TFT_BLACK, TFT_GREEN);
                tft.setCursor(5, 5, 4);
                tft.println("O2% OK!");
                tft.setCursor(5, 30, 4);
                tft.println("Continue!");
                low = false;
            }

            baselineO2 = Oxygen.getOxygenData(COLLECT_NUMBER);
            tft.setCursor(5, 67, 4);
            tft.print("O2: ");
            tft.print(baselineO2);
            tft.println(" % ");
            tft.setCursor(5, 105, 4);
            tft.println("Continue              >>>");
            delay(500);
        }
        if (baselineO2 < 20.00) baselineO2 = 20.90;
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(5, 5, 4);
        tft.println("Initial O2% set to:");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(5, 55, 4);
        tft.print(baselineO2);
        tft.println(" % ");
        delay(5000);
    }
}
#endif

//--------------------------------------------------

void CheckInitialCO2() { // check initial CO2 value
    readCO2();
    baselineCO2 = co2perc;
    bool high = false;

    if (baselineCO2 > 0.1) {
        while (digitalRead(buttonPin1)) {
            if (!high && baselineCO2 > 0.1) {
                tft.fillScreen(TFT_RED);
                tft.setTextColor(TFT_WHITE, TFT_RED);
                tft.setCursor(5, 5, 4);
                tft.println("INITIAL CO2% HIGH!");
                tft.setCursor(5, 30, 4);
                tft.println("Wait to continue!");
                high = true;
            } else if (high && baselineCO2 <= 0.1) {
                tft.fillScreen(TFT_GREEN);
                tft.setTextColor(TFT_BLACK, TFT_GREEN);
                tft.setCursor(5, 5, 4);
                tft.println("CO2% OK!");
                tft.setCursor(5, 30, 4);
                tft.println("Continue!");
                high = false;
            }

            readCO2();
            baselineCO2 = co2perc;
            tft.setCursor(5, 67, 4);
            tft.print("CO2: ");
            tft.print(baselineCO2, 2);
            tft.println(" % ");
            tft.setCursor(5, 105, 4);
            tft.println("Continue              >>>");
            delay(500);
        }
        if (baselineCO2 > 0.1) baselineCO2 = 0.1;
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(5, 5, 4);
        tft.println("Initial CO2 set to:");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(5, 55, 4);
        tft.print(baselineCO2, 2);
        tft.println("%");
        delay(5000);
    }
}

//--------------------------------------------------

void ConvertTime(float ms) {
    long   inms = long(ms);
    int    h, m, s;
    String strh, strm, strs;
    s = (inms / 1000) % 60;
    m = (inms / 60000) % 60;
    h = (inms / 3600000) % 24;
    strs = String(s);
    if (s < 10) strs = String("0") + strs;
    strm = String(m);
    if (m < 10) strm = String("0") + strm;
    strh = String(h);
    if (h < 10) strh = String("0") + strh;
    TotalTimeMin = String(strh) + String(":") + String(strm) + String(":") + String(strs);
}

//--------------------------------------------------

void VolumeCalc() {

    // Read pressure from Omron D6F PH0025AD1 (or D6F PH0025AD2)
    float pressureraw = mySensor.getPressure();
    pressure = pressure / 2 + pressureraw / 2;

    if (isnan(pressure)) { // isnan = is not a number,  unvalid sensor data
        tft.fillScreen(TFT_RED);
        tft.setTextColor(TFT_WHITE, TFT_RED);
        tft.drawCentreString("VENTURI ERROR!", 120, 55, 4);
    }
    if (pressure > 266) { // upper limit of flow sensor warning
        // tft.fillScreen(TFT_RED);
        tft.setTextColor(TFT_WHITE, TFT_RED);
        tft.drawCentreString("SENSOR LIMIT!", 120, 55, 4);
    }
    if (pressure < 0) pressure = 0;

    if (pressure < pressThreshold && readVE == 1) { // read volumeVE
        readVE = 0;
        DurationVE = millis() - TimerVE;
        TimerVE = millis(); // start timerVE
        volumeExp = volumeTotal;
        volumeTotal = 0; // resets volume for next breath
        volumeVE = volumeExp / DurationVE * 60;
        volumeExp = volumeExp / 1000;
        volumeVEmean = (volumeVEmean * 3 / 4) + (volumeVE / 4); // running mean of one minute volume (VE)
        if (volumeVEmean < 1) volumeVEmean = 0;
        freqVE = 60000 / DurationVE;
        if (volumeVE < 0.1) freqVE = 0;
        freqVEmean = (freqVEmean * 3 / 4) + (freqVE / 4);
        if (freqVEmean < 1) freqVEmean = 0;

#ifdef VERBOSE
        Serial.print("volumeExp: ");
        Serial.print(volumeExp);
        Serial.print("   VE: ");
        Serial.print(volumeVE);
        Serial.print("   VEmean: ");
        Serial.print(volumeVEmean);
        Serial.print("   freqVE: ");
        Serial.print(freqVE, 1);
        Serial.print("   freqVEmean: ");
        Serial.println(freqVEmean, 1);
#endif
    }
    if (millis() - TimerVE > 5000) readVE = 1; // readVE at least every 5s

    if (pressure >= pressThreshold) { // ongoing integral of volumeTotal
        if (volumeTotal > 50) readVE = 1;
        massFlow =
            1000 * sqrt((abs(pressure) * 2 * rhoATPS) / ((1 / (pow(area_2, 2))) - (1 / (pow(area_1, 2))))); // Bernoulli equation
        volFlow = massFlow / rhoATPS;                  // volumetric flow of air
        volFlow = volFlow * settings.correctionSensor; // correction of sensor calculations
        volumeTotal = volFlow * (millis() - TimerVolCalc) + volumeTotal;
        volumeTotal2 = volFlow * (millis() - TimerVolCalc) + volumeTotal2;
    } else if ((volumeTotal2 - volumeTotalOld) > 200) { // calculate actual expiratory volume
        expiratVol = (volumeTotal2 - volumeTotalOld) / 1000;
        volumeTotalOld = volumeTotal2;
    }
}

#ifdef GADGET
//--------------------------------------------------
void GadgetWrite() {
    // Send to sensirion app
    provider.writeValueToCurrentSample(vo2Total, SignalType::CO2_PARTS_PER_MILLION);
    provider.writeValueToCurrentSample(vo2Max, SignalType::TEMPERATURE_DEGREES_CELSIUS);
    provider.writeValueToCurrentSample(lastO2, SignalType::RELATIVE_HUMIDITY_PERCENTAGE);
    provider.commitSample();
}
#endif

//--------------------------------------------------
// Output as basic VO2 Master data for GoldenCheetah
void VO2Notify() {
    if (co2Enabled && settings.co2_on) // CO2 temp data
    {
        cheetah.temp = co2temp;
        cheetah.hum = co2hum; // humid
    } else if (bmpEnabled)    // baro temp data
    {
        cheetah.temp = TempC;
        cheetah.hum = Humid; // humid
    } else                   // btps defaults
    {
        cheetah.temp = 35;
        cheetah.hum = 95;
    }
    cheetah.rmv = volumeVEmean;
    cheetah.vo2 = vo2Max;
    cheetah.feo2 = lastO2 * 100;

    if (_BLEClientConnected) {
        cheetahCharacteristics.setValue((uint8_t *)&cheetah, 10);
        cheetahCharacteristics.notify(true);
    }
}

//--------------------------------------------------
void DataStream(bool bt) { // bt - send over bluetooth serial
    Stream *strm = bt ? (Stream *)&SerialBT : (Stream *)&Serial;

    // HeaderStreamed = 1;// TEST: Deactivation of header
    if (HeaderStreamed == 0) {
        const char *labels[] = {"Time", //
                                "VO2",
                                "VO2MAX",
                                "VCO2",
                                "RQ",
                                "Bvol",
                                "VEmin",
                                "Brate",
                                "outO2%",
                                "CO2%\n"};
        const int   numlab = sizeof(labels) / sizeof(const char *);

        for (int i = 0; i < numlab; i++) {
            if (i) strm->print(",");
            strm->print(labels[i]);
        }
        HeaderStreamed = 1;
    }

    float values[] = {float(TotalTime / 1000), //
                      vo2Max,
                      vo2MaxMax,
                      vco2Max,
                      respq,
                      volumeExp,
                      volumeVEmean,
                      freqVEmean,
                      lastO2,
                      co2perc};
    int   numval = sizeof(values) / sizeof(float);

    int prec = 0;
    for (int i = 0; i < numval; i++) {
        if (i > 0) prec = 2;
        if (i == numval - 1) prec = 3;

        if (i) strm->print(",");
        strm->print(values[i], prec);
    }
    strm->println("");
}

//--------------------------------------------------
// TODO add voltage to percent table. hardcode some basics for now.
void BatteryBT() {
    int level;
    if (Battery_Voltage > 4.0)
        level = 100;
    else if (Battery_Voltage > 3.9)
        level = 75;
    else if (Battery_Voltage > 3.7)
        level = 50;
    else if (Battery_Voltage > 3.5)
        level = 25;
    else if (Battery_Voltage > 3.3)
        level = 10;

    byte value[1] = {level};
    batteryLevelCharacteristics.setValue(value, 1);
    batteryLevelCharacteristics.notify();
}

//--------------------------------------------------
// Deduce co2 from Oxygen sensor, alternatively use CO2 data directly
float CalcCO2() {
    // Are we using the co2 sensor?
    if (settings.co2_on && co2Enabled) {
        readCO2();
#ifdef DEBUG
        Serial.print("Read co2: ");
        Serial.println(co2perc);
#endif
    } else { // default co2values
        co2temp = 35;
    }
#ifdef OXYSENSOR
    float oxygenData = Oxygen.getOxygenData(COLLECT_NUMBER);
    lastO2 = oxygenData;
    if (lastO2 > baselineO2) baselineO2 = lastO2; // correction for drift of O2 sensor
#ifdef DEBUG
    // Debug. compare co2
    Serial.print("Calc co2: ");
    Serial.println(baselineO2 - lastO2);
#endif
    return baselineO2 - lastO2;
#else
    float O2 = baselineO2 - co2perc;
    lastO2 = O2;
    return co2perc;
#endif
}

//--------------------------------------------------
void readCO2() {
    float result[3] = {0};
    bool  read = false;

#ifdef STC_31
    if (mySHTC3.update() == SHTC3_Status_Nominal) {
        co2temp = mySHTC3.toDegC();
        stc3x_sensor.setTemperature(co2temp);
        co2hum = mySHTC3.toPercent();
        stc3x_sensor.setRelativeHumidity(co2hum);
        uint16_t pressure = PresPa / 100;
        stc3x_sensor.setPressure(pressure); // Pressure in mbar
#ifdef DEBUG
        Serial.print("Read temp ");
        Serial.print(co2temp);
        Serial.print(" hum ");
        Serial.println(co2hum);
#endif
    }

    if (stc3x_sensor.measureGasConcentration(co2perc, co2temp) == 0) {
#ifdef DEBUG
        Serial.print("Co2 ");
        Serial.println(co2perc); // Log read value
#endif
        if (co2perc < 0) {
            co2perc = 0.0;
            // Reset baseline if below zero?
            // mySTC_31.forcedRecalibration(0.0);
        }
        read = true;
    }
#endif

#ifdef SCD_30
    if (scd30.isAvailable()) {
        scd30.getCarbonDioxideConcentration(result);

        co2ppm = result[0];
        if (co2ppm >= 40000) { // upper limit of SCD_30 CO2 sensor
            // tft.fillScreen(TFT_RED);
            tft.setTextColor(TFT_WHITE, TFT_RED);
            tft.drawCentreString("CO2 LIMIT!", 120, 55, 4);
        }

        co2perc = co2ppm / 10000;
        co2temp = result[1];
        co2hum = result[2];
        read = true;
    }
#endif

    if (read) {
        // perhaps change to occasional reset/rebase? or average? due to sensor innaccuracy.
        if (baselineCO2 > co2perc) baselineCO2 = co2perc;

        float co2percdiff = co2perc - baselineCO2; // calculates difference to initial CO2
        if (co2percdiff < 0) co2percdiff = 0;

        // VCO2 calculation is based on changes in CO2 concentration (difference to baseline)
        vco2Total = volumeVEmean * rhoBTPS / rhoSTPD * co2percdiff * 10; // = vco2 in ml/min (* co2% * 10 for L in ml)
        vco2Max = vco2Total / settings.weightkg;                         // correction for wt
        respq = (vco2Total * 44) / (vo2Total * 32);                      // respiratory quotient based on molarity
        // CO2: 44g/mol, O2: 32 g/mol
    }
#ifdef VERBOSE
    Serial.print("VCO2t: ");
    Serial.print(vco2Total);
    Serial.print("VO2t: ");
    Serial.print(vo2Total);
    Serial.print("RQ: ");
    Serial.println(respq);
#endif
    if (isnan(respq)) respq = 0; // correction for errors/div by 0
    if (respq > 1.5) respq = 0;

#ifdef VERBOSE
    Serial.print("CO2 values: ");
    Serial.print(co2perc);
    Serial.print(" %");
    Serial.print("Temp = ");
    Serial.print(co2temp);
    Serial.print(" ℃ ");
    Serial.print("Hum = ");
    Serial.print(co2hum);
    Serial.println(" %");
#endif
}

//--------------------------------------------------
// Calculate air density at temp and humidity
// https://en.wikipedia.org/wiki/Density_of_air
float calcRho(float tempC, float humid, float pressure) {

    // Use simple Tetens equation
    float p1 = 6.1078 * (pow(10, (7.5 * tempC / (tempC + 237.3))));

    float pv = humid * p1;
    float pd = pressure - pv;
    float tempK = tempC + 273.15;

    float rho = (pd / (dryConstant * tempK)) + (pv / (wetConstant * tempK));
    return rho;
}

// Update air density factors for ambient and expired
void AirDensity() {
    if (bmpEnabled) {
        TempC = bmp.readTemperature(); // Temp in C from baro sensor
        PresPa = bmp.readPressure();   // pressure in pa
#ifdef BME280
        Humid = bmp.readHumidity(); // %
#endif
        // TODO NOTE
        //  Always use initial temperature until we can check/move/setup the barometer to not
        //  be affected by the increase in device temperature during use.
        static float baseTempC = TempC; // static for single set
        // rhoATPS = calcRho(TempC, Humid, PresPa); // get Ambient factor
        rhoATPS = calcRho(baseTempC, Humid, PresPa); // get Ambient factor
    }

    if (settings.co2_on && co2Enabled) {
        // TODO NOTE
        //  For now hardcode body temp. instead of using values from co2 sensor
        //  Humidity seems to read quickly, albeit always at about this level
        //  temperature is a long time to rise to temperature of expired air
        //  however, temp *is* used to set values for co2 sensor to improve accuracy
        rhoBTPS = calcRho(35, 95, PresPa); // get body factor

        // co2temp is temperature from CO2 sensor
        // rhoBTPS = calcRho(co2temp, co2hum, PresPa); // get body factor
    }

#ifdef VERBOSE
    Serial.print("Ambient: ");
    Serial.print(TempC);
    Serial.print("℃ ");
    Serial.print(Humid);
    Serial.print("% ");
    Serial.print("Expired: ");
    Serial.print(co2temp);
    Serial.print("℃ ");
    Serial.print(co2hum);
    Serial.print("% ");
    Serial.print("Pressure: ");
    Serial.print(PresPa / 100);
    Serial.println("pa");
    Serial.print("ATPS: ");
    Serial.print(rhoATPS);
    Serial.print("BTPS: ");
    Serial.println(rhoBTPS);
#endif

    // Use hardcoded constants instead
    // rhoATPS = PresPa / (TempC + 273.15) / 287.058; // calculation of ambient density
    // rhoBTPS = PresPa / (35 + 273.15) / 292.9; // density at BTPS: 35°C, 95% humidity
}

//--------------------------------------------------

void vo2maxCalc() { // V02max calculation every 5s
    co2 = CalcCO2();
    AirDensity(); // calculates air density factors

    vo2Total = volumeVEmean * rhoBTPS / rhoSTPD * co2 * 10; // = vo2 in ml/min (* co2% * 10 for L in ml)
    vo2Max = vo2Total / settings.weightkg;                  // correction for wt
    if (vo2Max > vo2MaxMax) vo2MaxMax = vo2Max;

    vo2Cal = vo2Total / 1000 * 4.86;                     // vo2Max liters/min * 4.86 Kcal/liter = kcal/min
    calTotal = calTotal + vo2Cal * TimerVO2diff / 60000; // integral function of calories
    vo2CalH = vo2Cal * 60.0;                             // actual calories/min. * 60 min. = cal./hour
    vo2CalDay = vo2Cal * 1440.0;                         // actual calories/min. * 1440 min. = cal./day
    if (vo2CalDay > vo2CalDayMax) vo2CalDayMax = vo2CalDay;
}

//--------------------------------------------------

void showScreen() { // select active screen
    ConvertTime(TotalTime);
    tft.setRotation(1);
    switch (screenNr) {
        case 1:
            tftScreen1();
            break;
        case 2:
            tftScreen2();
            break;
        case 3:
            tftScreen3();
            break;
        case 4:
            tftScreen4();
            break;
        case 5:
            // tft.setRotation(2);
            tftScreen5();
            break;
        case 6:
            tftParameters();
            break;
        default:
            // if nothing else matches, do the default
            // default is optional
            break;
    }
}

//--------------------------------------------------
void showParameters() {
    while (digitalRead(buttonPin2)) { // wait until button2 is pressed
        // Let stabilise
        if (settings.co2_on && co2Enabled) {
            readCO2();
            baselineCO2 = co2perc;
        }
        AirDensity();
        tftParameters(); // show initial sensor parameters

        tft.setCursor(220, 5, 4);
        tft.print(">");
        delay(500);
        tft.setCursor(220, 5, 4);
        tft.print("    ");
        delay(500);
    }
    while (digitalRead(buttonPin2) == 0)
        ;
}

//--------------------------------------------------
// Reset O2 calibration value
#ifdef OXYSENSOR
void fnCalO2() {
    Oxygen.calibrate(20.9, 0.0);
    showParameters();
}
#endif

//--------------------------------------------------
// Calibrate flow sensor
void fnCalAir() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(0, 5, 4);
    tft.println("Use 3L calib.pump");
    tft.setCursor(0, 30, 4);
    tft.println("for sensor check.");
    tft.setCursor(0, 105, 4);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Press to start      >>>");

    while (digitalRead(buttonPin1))
        ; // Start measurement ---------

    tft.fillScreen(TFT_BLACK);

    TimerStart = millis();
    float orig = settings.correctionSensor;
    settings.correctionSensor = 1.16; // precalibration factor
    // timing of the integral of volume calculation differs
    // between this calibration loop and the main loop

    volumeTotal2 = 0;

    do {
        TotalTime = millis() - TimerStart; // calculates actual total time
        VolumeCalc();                      // Starts integral function

        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(0, 5, 4);
        tft.println("Total Volume (ml):");
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(0, 55, 7);
        tft.println(volumeTotal2, 0);
        tft.setCursor(0, 105, 4);
        tft.print(expiratVol, 3);
        tft.setCursor(100, 105, 4);
        tft.print(TotalTime / 1000, 1);
        // tft.setCursor(170, 105, 4);
        // tft.println(pressure, 1);

        TimerVolCalc = millis(); // part of the integral function to keep calculation volume over time
                                 // Resets amount of time between calcs

    } while (TotalTime < 10000);

    settings.correctionSensor = 3000 / volumeTotal2;

    // leave alone if not sensible.
    if (settings.correctionSensor < 0.8 || settings.correctionSensor > 1.2) settings.correctionSensor = orig;

    showParameters();
}
//--------------------------------------------------

struct MenuItem {
    int   id;
    char *label;
    bool  toggle;
    void (*fn)();
    bool *val;
};

int      icount = 0;
MenuItem menuitems[] = {
#ifdef OXYSENSOR
    {icount++, "Recalibrate O2", false, &fnCalO2, 0},
#endif
    {icount++, "Calibrate Flow", false, &fnCalAir, 0},
    {icount++, "Set Weight", false, &GetWeightkg, 0},
    {icount++, "Heart", true, 0, &settings.heart_on},
#ifdef GADGET
    {icount++, "Sensirion", true, 0, &settings.sens_on},
#endif
    {icount++, "Cheetah", true, 0, &settings.cheet_on},
    {icount++, "SerialBT", true, 0, &settings.serialbt},
#if defined(STC_31) || defined(SCD_30)
    {icount++, "CO2 sensor", true, 0, &settings.co2_on},
#endif
    {icount++, "Done.", false, 0, 0}};

//--------------------------------------------------
void doMenu() {
    int total = 5; // max on screen
    if (total > icount) total = icount;
    int cur = icount - 1; // Default to Done.
    int first = 0;        // 2
    first = (cur - (total - 1));

    loadSettings();

    while (1) {

        // Make sure buttons unpressed
        do {
            delay(100);
        } while ((digitalRead(buttonPin1) == 0) || (digitalRead(buttonPin2) == 0));

        tft.fillScreen(TFT_BLUE);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);

        tft.setCursor(220, 5, 4);
        tft.print(">");
        tft.setCursor(220, 105, 4);
        tft.print("+");

        // Display
        for (int i = 0; i < total; i++) {
            int y = 5 + i * 25;
            int x = 5;

            tft.setCursor(x, y, 4);

            int  item = i + first;
            bool sel;
            if (cur == item) {
                tft.setTextColor(TFT_BLUE, TFT_WHITE);
                sel = true;
            } else {
                tft.setTextColor(TFT_WHITE, TFT_BLUE);
                sel = false;
            }

            tft.print(" ");
            tft.print(menuitems[item].label);
            if (menuitems[item].toggle) {
                tft.print(*menuitems[item].val ? " [Yes]" : " [No]");
            } else {
                tft.print("...");
            }
        }

        // Detect click
        do {
            ReadButtons();
            delay(100);
        } while (buttonPushCounter1 == 0 && buttonPushCounter2 == 0);

        do {
            delay(100);
        } while ((digitalRead(buttonPin1) == 0) || (digitalRead(buttonPin2) == 0));

        if (buttonPushCounter2) {
            if (menuitems[cur].toggle) {
                *menuitems[cur].val = !*menuitems[cur].val;
            } else {
                if (menuitems[cur].fn) {
                    (menuitems[cur].fn)(); // call function
                } else {
                    // Done
                    saveSettings();
                    return;
                }
            }
        }

        if (buttonPushCounter1) {
            cur = cur + 1;
            if (cur >= icount) cur = 0; // wrap
            first = (cur - (total - 1));
            if (first < 0) first = 0;
        }
    }
}

//--------------------------------------------------------
void tftScreen1() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5, 4);
    tft.print("Time  ");
    tft.setCursor(120, 5, 4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println(TotalTimeMin);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    tft.setCursor(5, 30, 4);
    tft.print("VO2 ");
    tft.setCursor(120, 30, 4);
    tft.println(vo2Max);

    tft.setCursor(5, 55, 4);
    tft.print("VO2MAX ");
    tft.setCursor(120, 55, 4);
    tft.println(vo2MaxMax);

    if (settings.co2_on) {
        tft.setCursor(5, 80, 4);
        tft.print("VCO2 ");
        tft.setCursor(120, 80, 4);
        tft.println(vco2Max);

        tft.setCursor(5, 105, 4);
        tft.print("RQ ");
        tft.setCursor(120, 105, 4);
        tft.println(respq);
    }
}

//--------------------------------------------------------
void tftScreen2() {

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5, 4);
    tft.print("Time  ");
    tft.setCursor(120, 5, 4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println(TotalTimeMin);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    tft.setCursor(5, 30, 4);
    tft.print("outO2% ");
    tft.setCursor(120, 30, 4);
    tft.println(lastO2);

    if (settings.co2_on) {
        tft.setCursor(5, 55, 4);
        tft.print("CO2% ");
        tft.setCursor(120, 55, 4);
        tft.println(co2perc, 2);
    }

    tft.setCursor(5, 80, 4);
    tft.print("kcal ");
    tft.setCursor(120, 80, 4);
    tft.println(calTotal, 0);

    tft.setCursor(5, 105, 4);
    tft.print("kcal/h ");
    tft.setCursor(120, 105, 4);
    tft.println(vo2CalH, 0);
}

//--------------------------------------------------------
void tftScreen3() {

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5, 4);
    tft.print("Time  ");
    tft.setCursor(120, 5, 4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println(TotalTimeMin);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    tft.setCursor(5, 30, 4);
    tft.print("Bvol ");
    tft.setCursor(120, 30, 4);
    tft.println(volumeExp);

    tft.setCursor(5, 55, 4);
    tft.print("VEmin ");
    tft.setCursor(120, 55, 4);
    tft.println(volumeVEmean, 1);

    tft.setCursor(5, 80, 4);
    tft.print("Brate ");
    tft.setCursor(120, 80, 4);
    tft.println(freqVEmean, 1);

    tft.setCursor(5, 105, 4);
    tft.print("O2%diff ");
    tft.setCursor(120, 105, 4);
    float co2diff = lastO2 - baselineO2;
    tft.println(co2diff);
}
//--------------------------------------------------------
void tftScreen4() {

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5, 4);
    tft.print("Time ");
    tft.setCursor(120, 5, 4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println(TotalTimeMin);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    tft.setCursor(5, 30, 4);
    tft.print("O2% ");
    tft.setCursor(120, 30, 4);
    tft.println(lastO2);

    tft.setCursor(5, 55, 4);
    tft.print("CO2% ");
    tft.setCursor(120, 55, 4);
    tft.println(co2perc, 2);

    tft.setCursor(5, 80, 4);
    tft.print("Pressure ");
    tft.setCursor(120, 80, 4);
    tft.println((PresPa / 100));

    tft.setCursor(5, 105, 4);
    tft.print("Humidity ");
    tft.setCursor(120, 105, 4);
    tft.println(Humid, 2);
}

//--------------------------------------------------------
void tftScreen5() {

    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5, 4);
    tft.print("Time  ");
    tft.setCursor(120, 5, 4);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.println(TotalTimeMin);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 30, 4);
    tft.print("VO2 ");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(90, 30, 7);
    tft.println(vo2Max);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 80, 4);
    tft.print("RQ ");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(90, 80, 7);
    tft.println(respq);
}

//--------------------------------------------------------
void tftParameters() {

    tft.fillScreen(TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);

    tft.setCursor(5, 5, 4);
    tft.print("Co2");
    tft.setCursor(120, 5, 4);
    tft.println(baselineCO2, 1);

    tft.setCursor(5, 30, 4);
    tft.print("hPA");
    tft.setCursor(120, 30, 4);
    tft.println((PresPa / 100));

    tft.setCursor(5, 55, 4);
    tft.print("kg/m3");
    tft.setCursor(120, 55, 4);
    tft.println(rhoATPS, 4);

    tft.setCursor(5, 80, 4);
    tft.print("kg");
    tft.setCursor(45, 80, 4);
    tft.println(settings.weightkg, 1);

    tft.setCursor(120, 80, 4);
    tft.print("cor");
    tft.setCursor(180, 80, 4);
    tft.println(settings.correctionSensor, 2);

    tft.setCursor(5, 105, 4);
    tft.print("inO2%");
    tft.setCursor(120, 105, 4);
    tft.println(baselineO2);
}

//--------------------------------------------------------
void ReadButtons() {
    buttonState1 = digitalRead(buttonPin1);
    buttonState2 = digitalRead(buttonPin2);
    if (buttonState1 == LOW) {
        buttonPushCounter1++;
    } else {
        buttonPushCounter1 = 0;
    }
    if (buttonState2 == LOW) {
        buttonPushCounter2++;
    } else {
        buttonPushCounter2 = 0;
    }
}
//---------------------------------------------------------

void GetWeightkg() {

    Timer5s = millis();
    int weightChanged = 0;
    tft.fillScreen(TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawString("Enter weight in kg", 20, 10, 4);
    tft.drawString(String(settings.weightkg), 48, 48, 7);

    while ((millis() - Timer5s) < 5000) {
        ReadButtons();

        if (buttonPushCounter1 > 0) {
            settings.weightkg = settings.weightkg - 0.5;
            if (buttonPushCounter1 > 8) settings.weightkg = settings.weightkg - 1.5;
            weightChanged = 1;
        }

        if (buttonPushCounter2 > 0) {
            settings.weightkg = settings.weightkg + 0.5;
            if (buttonPushCounter2 > 8) settings.weightkg = settings.weightkg + 1.5;
            weightChanged = 1;
        }

        if (settings.weightkg < 20) settings.weightkg = 20;
        if (settings.weightkg > 200) settings.weightkg = 200;
        if (weightChanged > 0) {
            tft.fillScreen(TFT_BLUE);
            tft.drawString("New weight in kg is:", 10, 10, 4);
            tft.drawString(String(settings.weightkg), 48, 48, 7);
            weightChanged = 0;
            Timer5s = millis();
        }
        delay(200);
    }
}

//---------------------------------------------------------

void readVoltage() {
    uint16_t v = analogRead(ADC_PIN);
    Battery_Voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    if (Battery_Voltage >= 4.3) tft.setTextColor(TFT_BLACK, TFT_WHITE); // USB powered, charging
    if (Battery_Voltage < 4.3) tft.setTextColor(TFT_BLACK, TFT_GREEN);  // battery full
    if (Battery_Voltage < 3.9) tft.setTextColor(TFT_BLACK, TFT_YELLOW); // battery half
    if (Battery_Voltage < 3.7) tft.setTextColor(TFT_WHITE, TFT_RED);    // battery critical
    tft.setCursor(0, 0, 4);
    tft.print(String(Battery_Voltage) + "V");
}

//---------------------------------------------------------

void InitBLE() {
    if (settings.cheet_on)
        BLEDevice::init("VO2-MAX"); // creates the device name
    else if (settings.heart_on)
        BLEDevice::init("VO2-HR"); // differnt name if just hr to help check/debugging

    // (1) Create the BLE Server
    BLEServer *pServer = BLEDevice::createServer(); // creates the BLE server
    pServer->setCallbacks(new MyServerCallbacks()); // creates the server callback function

    BLEService *batteryLevelService = pServer->createService(batteryLevelServiceId);
    batteryLevelCharacteristics.addDescriptor(new BLE2902());
    batteryLevelService->addCharacteristic(&batteryLevelCharacteristics);
    batteryLevelService->start();

    // (2) Create the BLE Service "heartRateService"
    if (settings.heart_on) {
        BLEService *pHeart = pServer->createService(heartRateService); // creates heatrate service with 0x180D

        // (3) Create the characteristics, descriptor, notification
        // characteristics 0x2837
        heartRateDescriptor.setValue("Rate from 0 to 200"); // describtion of the characteristic
        heartRateMeasurementCharacteristics.addDescriptor(&heartRateDescriptor);
        heartRateMeasurementCharacteristics.addDescriptor(new BLE2902()); // necessary for notifications
        pHeart->addCharacteristic(&heartRateMeasurementCharacteristics);  // creates heartrate
        //  client switches server notifications on/off via BLE2902 protocol

        // (4) Create additional characteristics
        sensorPositionDescriptor.setValue("Position 0 - 6");
        sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);
        pHeart->addCharacteristic(&sensorPositionCharacteristic);
        pHeart->start();
    }

    // (5) Create the BLE Service
    if (settings.cheet_on) {
        BLEService *pCheetah = pServer->createService(cheetahService);
        cheetahDescriptor.setValue("VO2 Data");
        cheetahCharacteristics.addDescriptor(&cheetahDescriptor);
        cheetahCharacteristics.addDescriptor(new BLE2902()); // will it work without?
        pCheetah->addCharacteristic(&cheetahCharacteristics);
        pCheetah->start();
    }
    BLEAdvertising *pAdvertising = pServer->getAdvertising();

    if (settings.cheet_on) {
        pAdvertising->addServiceUUID(cheetahService);
    }
    if (settings.heart_on) {
        pAdvertising->addServiceUUID(heartRateService);
    }

    pAdvertising->setScanResponse(true); // true? reduce power use?
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);
    // pAdvertising->setPreferredParams(0x06, 0x12);

    // (6) start the server and the advertising
    BLEDevice::startAdvertising();
}

//---------------------------------------------------------
