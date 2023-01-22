//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Spirometrie Project
// https://www.instructables.com/Accurate-VO2-Max-for-Zwift-and-Strava/
// Modifications by Ulrich Rissel & Ivor Hewitt
// git
const String Version = "V1.1 2022/03/29";
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "esp_adc_cal.h" // ADC calibration data
#define ADC_EN 14  //ADC_EN is the ADC detection enable port
#define ADC_PIN 34
int vref = 1100;

#include "Sensirion_GadgetBle_Lib.h"  //This is library to connect to Sensirion App
#include "DFRobot_OxygenSensor.h"    //Library for Oxygen sensor
#include <Omron_D6FPH.h>      //Library for pressure sensor
#include <Wire.h>
#include <TFT_eSPI.h> // Graphics and font library for ST7735 driver chip
#include <SPI.h>

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#include <Adafruit_BMP085.h> //Library for barometric sensor
Adafruit_BMP085 bmp;

//Starts Screen for TTGO device
TFT_eSPI tft = TFT_eSPI();  // Invoke library, pins defined in User_Setup.h

//Labels the pressure sensor: mySensor
Omron_D6FPH mySensor;

//Label of oxygen sensor
DFRobot_OxygenSensor Oxygen;
#define COLLECT_NUMBER    10             // collect number, the collection range is 1-100.
#define Oxygen_IICAddress ADDRESS_3  //I2C  label for o2 address

GadgetBle gadgetBle = GadgetBle(GadgetBle::DataType::T_RH_CO2); //uncomment 

const int16_t SCD_ADDRESS = 0x62;

//uint8_t data[12], counter;

struct {
  short freq;
  byte temp;
  byte hum;
  short rmv;
  short feo2;
  short vo2;
 } cheetah;

//00001524-1212-EFDE-1523-785FEABCD1
bool _BLEClientConnected = false;
#define cheetahService BLEUUID("00001523-1212-EFDE-1523-785FEABCD123") //"00001521-1212-EFDE-1523-785FEABCD123")

BLECharacteristic
    inCharacteristics(BLEUUID("00001525-1212-EFDE-1523-785FEABCD123"),
                      BLECharacteristic::PROPERTY_WRITE  |
                      0);
BLECharacteristic
    outCharacteristics(BLEUUID("00001526-1212-EFDE-1523-785FEABCD123"),
                      BLECharacteristic::PROPERTY_READ   |
                      0);

BLECharacteristic
    cheetahCharacteristics(BLEUUID("00001524-1212-EFDE-1523-785FEABCD123"), //00001524-1212-EFDE-1523-785FEABCD123"),
                      BLECharacteristic::PROPERTY_NOTIFY |
                      0);

                      
BLEDescriptor cheetahDescriptor(BLEUUID((uint16_t)0x2901));//??

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) { _BLEClientConnected = true; };
  void onDisconnect(BLEServer *pServer) { _BLEClientConnected = false; }
};

  
//Defines button state for adding wt
const int  buttonPin1 = 0;
const int  buttonPin2 = 35;
int wtTotal = 0;
int buttonPushCounter1 = 0;   // counter for the duration of button1 pressed
int buttonState1 = 1;         // current state of the button
int buttonPushCounter2 = 0;   // counter for the duration of button2 pressed
int buttonState2 = 1;         // current state of the button
int screenChanged = 0;
int screenNr = 1;
int HeaderStreamed = 0;
int HeaderStreamedBT = 0;
int DEMO = 0; // 1 = DEMO-mode

//Defines the size of the Venturi openings for the  calculations of AirFlow
float area_1 = 0.000531; //these are the areas taken carefully from the 3D printed venturi 2M before constriction
float area_2 = 0.000201;// this is area within the venturi
float rho = 1.225; //Density of air in kg/m3 under ISA conditions (15*C, 1013.25 hPa, sea level)
float massFlow = 0;
float volFlow = 0;
float volumeTotal = 0; //variable for holding total volume of breath
float pressure = 0.0; //differential pressure of the venturi nozzle
float volumeVE = 0.0;
float volumeVEmean = 0.0;
float volumeExp = 0.0;

//############### uncoment correct sensor for correction for sensor volumeTotal ###################

float correctionSensor = 1.0; //See how it works with default
//float correctionSensor = 0.962; //seemed to be close to this
//float correctionSensor = 0.92;   // mean for both sonsors
//float correctionSensor = 0.924; // BLUE SENSOR
//float correctionSensor = 0.915;   // WHITE SENSOR

//#################################################################################################

float weightkg = 74.0; // Standard-body-weight

float TimerVolCalc = 0.0;
float Timer5s = 0.0;
float Timer1min = 0.0;
float TimerVO2calc = 0.0;
float TimerVO2diff = 0.0; // used for integral of calories
float TimerStart = 0.0;
float TotalTime = 0.0;
String TotalTimeMin = String("00:00");
int readVE = 0;
float TimerVE = 0.0;
float DurationVE = 0.0;

float lastO2 = 0;
float initialO2 = 0;
float co2 = 0;
float calTotal = 0;
float vo2Cal = 0;
float vo2CalDay = 0.0; // calories per day
float vo2CalDayMax = 0.0; //highest value of calories per day
float vo2Max = 0; //value of vo2Max/min/kg, calculated every 30 seconds
float vo2Total = 0.0; //value of total vo2Max/min
float vo2MaxMax = 0; //Best value of vo2 max for whole time machine is on

float freqVE = 0.0; //ventilation frequency
float freqVEmean = 0.0;

float expiratVol = 0.0; // last expiratory volume in L
float volumeTotalOld = 0.0;
float volumeTotal2 = 0.0;
float TempC = 15.0; //Air temperature in Celsius barometric sensor BMP180
float PresPa = 101325; //uncorrected (absolute) barometric pressure

float Battery_Voltage = 0.0;

//----------------------------------------------------------------------------------------------------------
//                  SETUP
//----------------------------------------------------------------------------------------------------------
void setup()
{
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);

  InitBLE(); //

  gadgetBle.begin();  //uncomment to activate Sensirion App

  //defines ADC characteristics for battery voltage
  /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
  */
  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);    //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
    //Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
    //Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  } else {
    //Serial.println("Default Vref: 1100mV");
  }

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  readVoltage();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("iVOr2max", 0, 25, 4);
  tft.drawString(Version, 0, 50, 4);
  tft.drawString("Initialising...", 0, 75, 4);

  if (!digitalRead(buttonPin2)) { // DEMO Mode if button2 is pressed during power on
    DEMO = 1;
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("DEMO-MODE!", 0, 100, 4);
  }
  delay (3000);

  tft.fillScreen(TFT_BLACK);

  Wire.begin();
  Serial.begin(9600);
  if (!Serial) {
    tft.drawString("Serial ERROR!", 0, 0, 4);
  }
  else {
    tft.drawString("Serial ok", 0, 0, 4);
  }
  
/*  if (!SerialBT.begin("iVOr2 Max")) { // Start Bluetooth with device name
    tft.drawString("BT NOT ready!", 0, 25, 4);
  }
  else {
    tft.drawString("BT ready", 0, 25, 4);
  }*/
  
  if (!bmp.begin()) {
    //Serial.println("BMP180 sensor error!");
    tft.drawString("Temp/Pres. Error!", 0, 50, 4);
  }
  else {
    //Serial.println("Temp./pressure I2c connect success!");
    tft.drawString("Temp/Pres. ok", 0, 50, 4);
  }

  Wire.beginTransmission(0x25); //Start communication over i2c on channel 25 (page 6 in cut sheet)
  Wire.write(0x36); Wire.write(0x03); //Start sensor in mode "Averate till read, Mass flow", use 3615 for DP (see page 7 in cutsheet)
  Wire.endTransmission();
  Wire.beginTransmission(SCD_ADDRESS);
  Wire.write(0x21);
  Wire.write(0xb1);
  Wire.endTransmission();
  if (!Oxygen.begin(Oxygen_IICAddress)) {
    tft.drawString("O2-Sensor ERROR!", 0, 75, 4);
  }
  else {
    tft.drawString("O2-Sensor ok", 0, 75, 4);
  }
  while (!mySensor.begin(MODEL_0025AD1)) {
    //Serial.println("Flow sensor error!");
    tft.drawString("Flow-Sensor ERROR!", 0, 100, 4);
  }
  //Serial.println("Flow-Sensor I2c connect success!");
  tft.drawString("Flow-Sensor ok", 0, 100, 4);
  delay (2000);

  gadgetBle.writeCO2(1);
  gadgetBle.writeTemperature(1);
  gadgetBle.writeHumidity(1);
  gadgetBle.commit(); 

  initialO2 = Oxygen.ReadOxygenData(COLLECT_NUMBER); //read and check initial VO2%
  if (initialO2 < 20.00) {
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setCursor(5, 5, 4);
    tft.println("INITIAL VO2% LOW!");
    tft.setCursor(5, 30, 4);
    tft.println("Wait to continue!");
    while (digitalRead(buttonPin1)) {
      initialO2 = Oxygen.ReadOxygenData(COLLECT_NUMBER);
      tft.setCursor(5, 67, 4);
      tft.println("VO2%: ");
      tft.setCursor(120, 67, 4);
      tft.println(initialO2);
      tft.setCursor(5, 105, 4);
      tft.println("Continue              >>>");
      delay (500);
    }
    if (initialO2 < 20.00) initialO2 = 20.90;
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5, 4);
    tft.println("Initial VO2% set to:");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(100, 55, 4);
    tft.println(initialO2);
    delay (5000);
  }

  GetWeightkg();// Enter weight
  while (digitalRead(buttonPin2)) { //wait until button2 is pressed
    
    AirDensity();
    initialO2 = Oxygen.ReadOxygenData(COLLECT_NUMBER); //allow us to see it's sane and stable
    
    tftParameters();// show initial sensor parameters
    tft.setCursor(220, 5, 4);
    tft.print(">");
    delay (500);
    tft.setCursor(220, 5, 4);
    tft.print("    ");
    delay (500);
  }

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(0, 5, 4);
  tft.println("Use 3L calib.pump");
  tft.setCursor(0, 30, 4);
  tft.println("for sensor check.");
  tft.setCursor(0, 105, 4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println("Press to start      >>>");

  while (digitalRead(buttonPin1)); // Start measurement ---------

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  TimerVolCalc = millis(); //timer for the volume (VE) integral function
  Timer5s = millis();
  Timer1min = millis();
  TimerVO2calc = millis(); //timer between VO2max calculations
  TimerStart = millis(); // holds the millis at start
  TotalTime = 0;
  //BatteryBT(); // TEST for battery discharge log ++++++++++++++++++++++++++++++++++++++++++++

}

//----------------------------------------------------------------------------------------------------------
//                  MAIN PROGRAM
//----------------------------------------------------------------------------------------------------------

void loop()
{
   
  TotalTime = millis() - TimerStart; // calculates actual total time
  VolumeCalc(); //Starts integral function

  // VO2max calculation, tft display and excel csv every 5s --------------
  if ((millis() - TimerVO2calc) > 5000 && pressure < 1) { // calls vo2maxCalc() for calculation Vo2Max every 5 seconds.
    TimerVO2diff = millis() - TimerVO2calc;
    TimerVO2calc = millis();  //resets the timer
    vo2maxCalc();  //vo2 max function call
    if (TotalTime >= 10000) {
      showScreen();
      volumeTotal2 = 0;  //resets volume2 to 0 (used for initial 10s sensor test)
      readVoltage();
    }
    ExcelStream(); //send csv data via wired com port
    ExcelStreamBT(); //send csv data via Bluetooth com port ++++++++++++++++++++++++++++++++++
  
    GadgetStream();     
    BLEStream(); 
   }

  //
  if (TotalTime >= 10000) {// after 10 sec. activate the buttons for switching the screens
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
    if (screenNr < 1) screenNr = 5;
    if (screenNr > 5) screenNr = 1;
    if (screenChanged == 1) {
      showScreen();
      screenChanged = 0;
    }
  }

  if (millis() - Timer1min > 30000) {
    Timer1min = millis(); // reset timer
    //BatteryBT(); //TEST für battery discharge log ++++++++++++++++++++++++++++++++++++++++++
  }

  TimerVolCalc = millis(); //part of the integral function to keep calculation volume over time
  // Resets amount of time between calcs
gadgetBle.handleEvents();
}

//----------------------------------------------------------------------------------------------------------
//                  FUNCTIONS
//----------------------------------------------------------------------------------------------------------

void ConvertTime (float ms) {
  long inms = long(ms);
  int h, m, s;
  String strh, strm, strs;
  s = (inms / 1000) % 60;
  m  = (inms / 60000) % 60;
  h  = (inms / 3600000) % 24;
  strs = String(s);
  if (s < 10) strs = String("0") + strs;
  strm = String(m);
  if (m < 10) strm = String("0") + strm;
  strh = String(h);
  if (h < 10) strh = String("0") + strh;
  TotalTimeMin = String(strh) + String(":") + String(strm) + String(":") + String(strs);
}

//--------------------------------------------------

void showScreen() { // select active screen
  ConvertTime (TotalTime);
  tft.setRotation(1);
  switch (screenNr) {
    case 1:
      tftScreen1();
      break;
    case 2:
      tftScreen2();
      break;
    case 3:
      tft.setRotation(2);
      tftScreen3();
      break;
    case 4:
      tft.setRotation(0);
      tftScreen3();
      break;
    case 5:
      tftParameters();
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

//--------------------------------------------------

void VolumeCalc() {

  //Read pressure from Omron D6F PH0025AD1 (or D6F PH0025AD2)
  pressure = mySensor.getPressure();
  //delay(3);

  if (DEMO == 1) {
    pressure = 16; // TEST+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ((millis() - TimerVO2calc) > 2500) pressure = 0; // TEST++++++++++++++++++++++++++++
  }

  if (TotalTime < 10000) { // initial check of volume sensor
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
    //tft.setCursor(170, 105, 4);
    //tft.println(pressure, 1);
//    correctionSensor=1.0; //show raw vol on calibrate?
  }

  if (isnan(pressure)) { //isnan = is not a number,  unvalid sensor data
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawCentreString("VENTURI ERROR!", 120, 55, 4);
  }
  if (pressure > 266) { // upper limit of flow sensor warning
    //tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.drawCentreString("SENSOR LIMIT!", 120, 55, 4);
  }
  if (pressure < 0) pressure = 0;

  if (pressure < 1 && readVE == 1) { // read volumeVE
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

    /*
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
    */
  }
  if (millis() - TimerVE > 5000) readVE = 1; // readVE at least every 5s

  if (pressure >= 1) { // ongoing integral of volumeTotal
    if (volumeTotal > 50) readVE = 1;
    massFlow = 1000 * sqrt((abs(pressure) * 2 * rho) / ((1 / (pow(area_2, 2))) - (1 / (pow(area_1, 2))))); //Bernoulli equation
    volFlow = massFlow / rho; //volumetric flow of air
    volFlow = volFlow * correctionSensor; // correction of sensor calculations
    volumeTotal = volFlow * (millis() - TimerVolCalc) + volumeTotal;
    volumeTotal2 = volFlow * (millis() - TimerVolCalc) + volumeTotal2;
  }
  else if ((volumeTotal2 - volumeTotalOld) > 200) { //calculate actual expiratory volume
    expiratVol = (volumeTotal2 - volumeTotalOld) / 1000;
    volumeTotalOld = volumeTotal2;
  }

}

void BLEStream(){
  // Publishes vo2Max to TV bluetooth
  cheetah.temp = TempC;
  cheetah.hum = 0; //humid
  cheetah.rmv = volumeVEmean;
  cheetah.vo2= vo2Max*100;
  cheetah.feo2 = lastO2*100;
  
  cheetahCharacteristics.setValue((uint8_t*)&cheetah, 10);
  cheetahCharacteristics.notify();
}

void GadgetStream() {
  gadgetBle.writeCO2(vo2Total);
  gadgetBle.writeTemperature(vo2Max);
  gadgetBle.writeHumidity(lastO2);
  gadgetBle.commit();
}
//--------------------------------------------------
void ExcelStream() {
  //HeaderStreamed = 1;// TEST: Deactivation of header
  if (HeaderStreamed == 0) {
    Serial.print("Time");
    Serial.print(",");
    Serial.print("VO2");
    Serial.print(",");
    Serial.print("VO2MAX");
    Serial.print(",");
    Serial.print("VO2total");
    Serial.print(",");
    Serial.print("kcal");
    Serial.print(",");
    Serial.print("Bvol");
    Serial.print(",");
    Serial.print("VEmin");
    Serial.print(",");
    Serial.print("Brate");
    Serial.print(",");
    Serial.print("outO2%");
    Serial.print(",");
    Serial.println("inO2%");
    HeaderStreamed = 1;
  }
  Serial.print(float(TotalTime / 1000), 0);
  Serial.print(",");
  Serial.print(vo2Max);
  Serial.print(",");
  Serial.print(vo2MaxMax);
  Serial.print(",");
  Serial.print(vo2Total);
  Serial.print(",");
  Serial.print(calTotal);
  Serial.print(",");
  Serial.print(volumeExp);
  Serial.print(",");
  Serial.print(volumeVEmean);
  Serial.print(",");
  Serial.print(freqVEmean);
  Serial.print(",");
  Serial.print(lastO2);
  Serial.print(",");
  Serial.println(initialO2);
}
//--------------------------------------------------
void ExcelStreamBT() {
  //HeaderStreamedBT = 1;// TEST: Deactivation of header
  if (HeaderStreamedBT == 0) {
    SerialBT.print("Time");
    SerialBT.print(",");
    SerialBT.print("VO2");
    SerialBT.print(",");
    SerialBT.print("VO2MAX");
    SerialBT.print(",");
    SerialBT.print("VO2total");
    SerialBT.print(",");
    SerialBT.print("kcal");
    SerialBT.print(",");
    SerialBT.print("Bvol");
    SerialBT.print(",");
    SerialBT.print("VEmin");
    SerialBT.print(",");
    SerialBT.print("Brate");
    SerialBT.print(",");
    SerialBT.print("outO2%");
    SerialBT.print(",");
    SerialBT.println("inO2%");
    HeaderStreamedBT = 1;
  }
  SerialBT.print(float(TotalTime / 1000), 0);
  SerialBT.print(",");
  SerialBT.print(vo2Max);
  SerialBT.print(",");
  SerialBT.print(vo2MaxMax);
  SerialBT.print(",");
  SerialBT.print(vo2Total);
  SerialBT.print(",");
  SerialBT.print(calTotal);
  SerialBT.print(",");
  SerialBT.print(volumeExp);
  SerialBT.print(",");
  SerialBT.print(volumeVEmean);
  SerialBT.print(",");
  SerialBT.print(freqVEmean);
  SerialBT.print(",");
  SerialBT.print(lastO2);
  SerialBT.print(",");
  SerialBT.println(initialO2);
}

//--------------------------------------------------

void BatteryBT() {
  //HeaderStreamedBT = 1;// TEST: Deactivation of header
  if (HeaderStreamedBT == 0) {
    SerialBT.print("Time");
    SerialBT.print(",");
    SerialBT.println("Voltage");
    HeaderStreamedBT = 1;
  }
  SerialBT.print(float(TotalTime / 1000), 0);
  SerialBT.print(",");
  SerialBT.println(Battery_Voltage);
}

//--------------------------------------------------

void ReadO2() {
  float oxygenData = Oxygen.ReadOxygenData(COLLECT_NUMBER);
  lastO2 = oxygenData;
  if (lastO2 > initialO2) initialO2 = lastO2; // correction for drift of O2 sensor

  if (DEMO == 1) lastO2 = initialO2 - 4; //TEST+++++++++++++++++++++++++++++++++++++++++++++
  co2 = initialO2 - lastO2;
}

//--------------------------------------------------

void AirDensity () {
  TempC = bmp.readTemperature(); //Temp from baro sensor BMP180
  PresPa = bmp.readPressure();
  rho = PresPa / (TempC + 273.15) / 287.058;  //calculation of air density
}

//--------------------------------------------------

void vo2maxCalc() { //V02max calculation every 5s
  ReadO2();
  AirDensity();// calculates air density

  co2 = initialO2 - lastO2;  //calculated level of CO2 based on Oxygen level loss
  if (co2 < 0) co2 = 0; // correction for sensor drift

  vo2Total = volumeVEmean * (rho/1.225) * co2 * 10; // = vo2 in ml/min (* co2% * 10 for L in ml)
  vo2Max = vo2Total / weightkg; //correction for wt
  if (vo2Max > vo2MaxMax) vo2MaxMax = vo2Max;

  vo2Cal = vo2Total / 1000 * 4.86; //vo2Max liters/min * 4.86 Kcal/liter = kcal/min
  calTotal = calTotal + vo2Cal * TimerVO2diff / 60000; // integral function of calories
  vo2CalDay = vo2Cal * 1440.0; // actual calories/min. * 1440 min. = cal./day
  if (vo2CalDay > vo2CalDayMax) vo2CalDayMax = vo2CalDay;
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

  tft.setCursor(5, 80, 4);
  tft.print("VO2total ");
  tft.setCursor(120, 80, 4);
  tft.println(vo2Total, 0);

  tft.setCursor(5, 105, 4);
  tft.print("kcal ");
  tft.setCursor(120, 105, 4);
  tft.println(calTotal, 0);

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
  tft.print("outO2% ");
  tft.setCursor(120, 105, 4);
  tft.println(lastO2);

}
//--------------------------------------------------------
void tftScreen3() {

  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(5, 5, 4);
  tft.print("Time  ");
  tft.setCursor(5, 30, 4);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.println(TotalTimeMin);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.setCursor(5, 55, 4);
  tft.print("VO2 ");
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(5, 80, 7);
  tft.println(vo2Max, 1);
}

//--------------------------------------------------------



//--------------------------------------------------------
void tftParameters() {

  tft.fillScreen(TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);

  tft.setCursor(5, 5, 4);
  tft.print("*C");
  tft.setCursor(120, 5, 4);
  tft.println(TempC, 1);

  tft.setCursor(5, 30, 4);
  tft.print("hPA");
  tft.setCursor(120, 30, 4);
  tft.println((PresPa / 100));

  tft.setCursor(5, 55, 4);
  tft.print("kg/m3");
  tft.setCursor(120, 55, 4);
  tft.println(rho, 4);

  tft.setCursor(5, 80, 4);
  tft.print("KG");
  tft.setCursor(120, 80, 4);
  tft.println(weightkg, 1);

  tft.setCursor(5, 105, 4);
  tft.print("O2%");
  tft.setCursor(120, 105, 4);
  tft.println(initialO2);
}

//--------------------------------------------------------
void ReadButtons() {
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  if (buttonState1 == LOW) {
    buttonPushCounter1++;
  }
  else {
    buttonPushCounter1 = 0;
  }
  if (buttonState2 == LOW) {
    buttonPushCounter2++;
  }
  else {
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
  tft.drawString(String (weightkg), 48, 48, 7);

  while ((millis() - Timer5s) < 5000) {
    ReadButtons();

    if (buttonPushCounter1 > 0) {
      weightkg = weightkg - 0.5;
      if (buttonPushCounter1 > 8) weightkg = weightkg - 1.5;
      weightChanged = 1;
    }

    if (buttonPushCounter2 > 0) {
      weightkg = weightkg + 0.5;
      if (buttonPushCounter2 > 8) weightkg = weightkg + 1.5;
      weightChanged = 1;
    }

    if (weightkg < 20) weightkg = 20;
    if (weightkg > 200) weightkg = 200;
    if (weightChanged > 0) {
      tft.fillScreen(TFT_BLUE);
      tft.drawString("New weight in kg is:", 10, 10, 4);
      tft.drawString(String (weightkg), 48, 48, 7);
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
  if (Battery_Voltage < 4.3) tft.setTextColor(TFT_BLACK, TFT_GREEN); // battery full
  if (Battery_Voltage < 3.9) tft.setTextColor(TFT_BLACK, TFT_YELLOW); // battery half
  if (Battery_Voltage < 3.7) tft.setTextColor(TFT_WHITE, TFT_RED); // battery critical
  tft.setCursor(0, 0, 4);
  tft.print(String(Battery_Voltage) + "V");
}



void InitBLE() {
//uint8_t new_mac[8] = {0x94, 0x54, 0x93, 0x91, 0x9a, 0x9e};
//esp_base_mac_addr_set(new_mac);
  
  BLEDevice::init("VO2 Master Pro");
  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pCheetah = pServer->createService(cheetahService);
  
  pCheetah->addCharacteristic(&cheetahCharacteristics);
//  pCheetah->addCharacteristic(&inCharacteristics);
//  pCheetah->addCharacteristic(&outCharacteristics);
  
  cheetahDescriptor.setValue("VO2 Data");
  
  cheetahCharacteristics.addDescriptor(&cheetahDescriptor);
  cheetahCharacteristics.addDescriptor(new BLE2902());

  pCheetah->start();

BLEAdvertising *pAdvertising = pServer->getAdvertising();

  pAdvertising->addServiceUUID(cheetahService);
pAdvertising->setScanResponse(true);
pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
pAdvertising->setMinPreferred(0x12);

BLEDevice::startAdvertising();

}


//---------------------------------------------------------
