/// ############################################################################################

/// Set this to the correct printed case venturi diameter:
#define DIAMETER 19

// CO2 sensor support
// Uncomment to use either STC31 or SCD30 CO2 sensors:
#define STC_31
// #define SCD_30

// Uncomment to use either BMP85 or BMP280 barometers:
// #define BMP085
#define BME280
// #define BMP280

// Uncomment to broadcast as sensirion gadget (lots of memory)
// also causes issues for BLE heart rate and cheetah broadcast
// #define GADGET

// EXPERIMENTAL:
// If undefined, use CO2 sensor instead of oxygen sensor otherwise
// calculate CO2 values from O2 data
#define OXYSENSOR

// #define VERBOSE // enable additional logging
// #define DEBUG   // additional debug info

/// ############################################################################################
