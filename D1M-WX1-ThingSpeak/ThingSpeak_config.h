// ThingSpeak_config.h

// This configuration file should reside in the same Arduino
// directory as the weather station file D1M-WX1-APRS-ThingSpeak.ino.

// *******************************************************
// ********************* WIFI LOGON **********************
// *******************************************************

// ENTER YOUR WI-FI SSID
// YOU MUST USE 2.4 GHz WiFi, NOT 5 GHz
const char WIFI_SSID[] = "your_wifi_ssid";

// ENTER YOUR WI-FI PASSWORD
const char WIFI_PASSWORD[] = "your_wifi_password";

// *******************************************************
// **************** STATION FACTORS **********************
// *******************************************************

// If you have performed the voltage calibration steps, enter
// the voltage you measured on your digital multimeter and
// the voltage reported by the ADC on the serial monitor.
// Calibration sketch is D1M-WX1_Calibration.ino
// If you have not performed the calibration, do not change 
// the default values. They should be equal.

const float DMM_VOLTAGE = 4.20;         // voltage displayed on your digital multimeter
const float ADC_VOLTAGE = 4.20;         // voltage reported by the Analog to Digital Converter

// station altitude in meters
// https://www.freemaptools.com/elevation-finder.htm
const float STATION_ELEV = 0.0;

// update interval in seconds
// must be longer than 15 seconds
// suggest 60 seconds for testing, 300 or 600 for use
// maximum sleep is 4,294 seconds (71.5 hours)
const unsigned long SLEEP_INTERVAL = 600;

// *******************************************************
// ******************** THINGSPEAK ***********************
// *******************************************************

// Open a ThingSpeak account at www.thingspeak.com
/* Define fields:
 *  Field 1 Temperature °C
 *  Field 2 Humidity
 *  Field 3 Time Awake
 *  Field 4 Sea Level Pressure
 *  Field 5 Light Intensity
 *  Field 6 Cell Voltage
 *  Field 7 RSSI
 *  Field 8 Temperature °F
 *  Show Location checked
 *  Show Status checked
 */

// ThingSpeak Channel ID & API Write Key
const long CHANNEL_ID = 00000;

const String API_WRITE_KEY = "your_API_write_key";

// ***************** DEFAULT VALUE ******************
// ***************** DO NOT CHANGE ******************
// ****** UNLESS YOU KNOW WHAT YOU ARE DOING ********

// You may change MIN_RSSI if you are consistently
// getting low RSSI warnings. 
// MIN_RSSI should be in the range of -90 to -70
// DO NOT SET IT BELOW -90!!! REPEAT: DO NOT SET IT BELOW -90!!!
const long  MIN_RSSI = -85;                              // warning level for weak WiFi
const float MIN_VCELL = 3.0;                             // warning level for low cell voltage
