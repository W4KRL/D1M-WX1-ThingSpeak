/* D1M-WX1-ThingSpeak.ino
   D1 Mini Weather Station (Solar)
   Posts to ThingSpeak using the REST API
   BH1750 range extended to 121,557 lux with Armborst library

   Set serial monitor to 115,200 baud
   04/19/2020 - reduced from D1M-WX1-APRS-ThingSpeak
   04/05/2020 - Armborst BH1750 library with extended range, changed sleep to WAKE_DEFAULT
              - simplified code in PostToThingSpeak
   09/20/2019 - changed low RSSI to -85 dBm, moved MIN_RSSI to ThingSpeak_config.h
   07/18/2018 - Added https://www.bakke.online/index.php/2017/06/24/esp8266-wifi-power-reduction-avoiding-network-scan/
   TO DO:
      Add wifiManager
      Add Over The Air update
*/
/*_____________________________________________________________________________
   Copyright 2016-2020 Berger Engineering dba IoT KitsÂ©
   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
   https://w4krl.com
   _____________________________________________________________________________
*/

// Incorporates power saving ideas from:
// https://www.bakke.online/index.php/2017/05/21/reducing-wifi-power-consumption-on-esp8266-part-1/
// and
// https://www.bakke.online/index.php/2017/06/24/esp8266-wifi-power-reduction-avoiding-network-scan/#more-376

#include <Wire.h>                    // [builtin] I2C bus
#include <ESP8266WiFi.h>             // [builtin] ESP8266 WiFi
#include <hp_BH1750.h>               // [manager] by Stefan Armborst https://github.com/Starmbi/hp_BH1750
#include <BME280I2C.h>               // Tyler Glenn https://github.com/finitespace/BME280

// Place your configuration files in same folder as this sketch
#include "ThingSpeak_config.h"

// *******************************************************
// ********************** DEFAULTS ***********************
// *******************************************************
// !!!!!!      DO NOT CHANGE THESE DEFAULTS         !!!!!!
const String IOT_SERVER = "api.thingspeak.com";           // ThingSpeak Server

// *******************************************************
// *********************** GLOBALS ***********************
// *******************************************************
const float HPA_TO_INHG = 0.0295299830714; // hPa (mb) to inHg pressure
String unitStatus = "";                    // for weather station status
bool rtcValid = false;                     // RTC check of validity
long startTime = millis();                 // record processor time when awakened
// set APRS telemetry span for number of seconds to xmit header
const int APRS_DEF_SPAN = APRS_TELEM_SPAN / SLEEP_INTERVAL;     // determines how often APRS definitions are sent

// structure to hold sensor measurements/calculations
struct {
  float stationPressure;         // station pressure (hPa) (mb)
  float seaLevelPressure;        // calculated SLP (hPa)
  float celsius;                 // temperature (Â°C)
  float fahrenheit;              // calculated temperature (Â°F)
  float humidity;                // relative humidity (%)
  float lightLevel;              // light intensity (lux)
  float cellVoltage;             // volts
  long  wifiRSSI;                // WiFi signal strength (dBm)
  bool  bme280Fail = false;      // BME280 sensor failure flag
  bool  bh1750Fail = false;      // BH1750 sensor failure flag
  bool  lowVcell = false;        // low Vcell alarm flag
  bool  lowRSSI = false;         // low WiFi signal alarm flag
} sensorData;                    // declare struct

// The ESP8266 RTC memory is arranged into blocks of 4 bytes.
// The RTC data structure MUST be padded to a 4-byte multiple.
// Maximum 512 bytes available.
// https://arduino-esp8266.readthedocs.io/en/latest/libraries.html#esp-specific-apis
// use fixed width types to avoid variations between devices, for example, int
// is two bytes in Arduino UNO and four bytes in ESP8266
struct {
  uint32_t  crc32;               // 4 bytes    4 total
  uint8_t   channel;             // 1 byte,    5 total
  uint8_t   bssid[6];            // 6 bytes,  11 total
  uint8_t   aprsSequence;        // 1 byte,   12 total
  uint8_t   bme280Fail;          // 1 byte,   13 total
  uint8_t   bh1750Fail;          // 1 byte,   14 total
  uint8_t   lowVcell;            // 1 byte,   15 total
  uint8_t   lowRSSI;             // 1 byte,   16 total
  float     timeAwake;           // 4 bytes,  20 total
} rtcData;

// *******************************************************
// ***************** INSTANTIATE OBJECTS *****************
// *******************************************************
BME280I2C myBME280;              // barometric pressure / temperature / humidity sensor
hp_BH1750 myBH1750;              // light level sensor
WiFiClient client;               // WiFi connection

// *******************************************************
// ************************ SETUP ************************
// *******************************************************
void setup() {
  WiFi.mode( WIFI_OFF );         // Bakke
  WiFi.forceSleepBegin();
  delay( 1 );
  Serial.begin(115200);          // initialize the serial port
  Wire.begin();                  // wake up the I2C bus for BH1750 & BME280
  initializeSensors();           // start BME280 and BH1750 sensors
  readSensors();                 // read data into sensorData struct
  printToSerialPort();           // display data on local serial monitor
  readRTCmemory();               // get APRS sequence number & WiFi channel info
  logonToRouter();               // logon to local Wi-Fi
  checkAlarms();                 // compare alarms with previous state

  postToThingSpeak();            // send data to ThingSpeak
  // TODO: could set Wi-Fi OFF at this point
  writeRTCmemory();              // save APRS sequence & WiFi channel info
  enterSleep( SLEEP_INTERVAL );  // go to low power sleep mode
} //setup()

// *******************************************************
// ************************* LOOP ************************
// *******************************************************
void loop() {                    // there is nothing to do here
} // loop()                      // everything is done in setup()

// *******************************************************
// ***************** SUPPORTING FUNCTIONS ****************
// *******************************************************

// *******************************************************
// ***************** Initialize Sensors ******************
// *******************************************************
void initializeSensors() {
  // initialize BME280 pressure/temperature/humidity sensor
  if ( myBME280.begin() == false ) {
    sensorData.bme280Fail = true;
    Serial.println( "BME280 sensor failure." );
  }
  // initialize BH1750 light sensor
  if ( myBH1750.begin( BH1750_TO_GROUND ) == false ) {
    sensorData.bh1750Fail = true;
    Serial.println( "BH1750 sensor failure." );
  }
  myBH1750.calibrateTiming();
} // initializeSensors()

// *******************************************************
// ********* Read sensors into sensorData struct *********
// *******************************************************
void readSensors() {
  // WiFi signal strength is read in logonToRouter()

  if ( sensorData.bme280Fail == false ) {
    // read pressure, temperature and humidity in one command
    myBME280.read(sensorData.stationPressure, sensorData.celsius,
                  sensorData.humidity, BME280::TempUnit_Celsius, BME280::PresUnit_hPa);

    // calculate fahrenheit
    sensorData.fahrenheit = 1.8 * sensorData.celsius + 32.0;

    // calculate the Sea Level Pressure from the station pressure and temperature
    sensorData.seaLevelPressure = calculateSeaLevelPressure(sensorData.celsius,
                                  sensorData.stationPressure, STATION_ELEV);
  }
  if ( sensorData.bh1750Fail == false ) {
    // read light level in lux
    myBH1750.start( BH1750_QUALITY_HIGH, 31 );        // max 121,557 lx, resolution 1.85 lx
    sensorData.lightLevel = myBH1750.getLux();
  }
  // read analog voltage from the Analog to Digital Converter
  // on D1 Mini this is 0 - 1023 for voltages 0 to 3.2V
  // the D1M-WX1 has an external resistor to extend the range to 5.0 Volts
  // a fudgeFactor corrects for voltage divider component variation
  // as measured by the user in the calbration step
  float fudgeFactor = DMM_VOLTAGE / ADC_VOLTAGE;
  sensorData.cellVoltage = 5.0 * analogRead(A0) * fudgeFactor / 1023.0;
  if ( sensorData.cellVoltage < MIN_VCELL ) {
    sensorData.lowVcell = true;
  }
} // readSensors()

// *******************************************************
// ************ Print data to the serial port ************
// *******************************************************
void printToSerialPort() {
  // '\t' is the C++ escape sequence for tab
  // header line
  Serial.println("\n\tÂ°C(Â°F)\t\tRH%\tSP mb\tSLP(in)\t\tLux\tVolt");
  // data line
  Serial.print("Data\t");
  Serial.print(sensorData.celsius, 1);
  Serial.print("(");
  Serial.print(sensorData.fahrenheit, 1);
  Serial.print(")\t");
  Serial.print(sensorData.humidity, 1);
  Serial.print("\t");
  Serial.print(sensorData.stationPressure, 1);
  Serial.print("\t");
  Serial.print(sensorData.seaLevelPressure, 1);
  Serial.print("(");
  Serial.print(sensorData.seaLevelPressure * HPA_TO_INHG, 2);
  Serial.print(")\t");
  Serial.print(sensorData.lightLevel, 1);
  Serial.print("\t");
  Serial.println(sensorData.cellVoltage, 2);
  Serial.println( unitStatus );
  Serial.println("----------------------------------------------------------------------------");
} // printToSerialPort()


// RTC Memory Functions: The ESP8266 internal Real Time Clock has unused memory
// that remains active during the Deep Sleep mode. This sketch stores WiFi connection
// information in RTC memory to speed up connection time.
// *******************************************************
// ******************* Read RTC Memory *******************
// *******************************************************
void readRTCmemory() {
  rtcValid = false;
  if ( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
    // Calculate the CRC of what we just read from RTC memory,
    // but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32(((uint8_t*)&rtcData ) + 4, sizeof( rtcData ) - 4 );
    if ( crc == rtcData.crc32 ) {
      rtcValid = true;
      Serial.print("Channel: "); Serial.print(rtcData.channel); Serial.print(", BSSID: ");
      for ( int j = 0; j < 6; j++ ) {
        Serial.print( rtcData.bssid[j] ); Serial.print(",");
      }
      Serial.print(" Sequence: "); Serial.println(rtcData.aprsSequence);
    }
  }
} // readRTCmemory()

// *******************************************************
// ****************** Write RTC Memory *******************
// *******************************************************
void writeRTCmemory() {
  rtcData.channel = WiFi.channel();         // WiFi channel
  // memcpy explanation: http://arduino.land/FAQ/content/6/30/en/how-to-copy-an-array.html
  memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
  rtcData.aprsSequence;                     // telemetry sequence
  rtcData.bme280Fail = sensorData.bme280Fail;
  rtcData.bh1750Fail = sensorData.bh1750Fail;
  rtcData.lowVcell   = sensorData.lowVcell;
  rtcData.lowRSSI    = sensorData.lowRSSI;
  rtcData.timeAwake  = ( millis() - startTime ) / 1000.0;  // total awake time in seconds
  rtcData.crc32      = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );

  ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );
} // writeRTCmemory()

// *******************************************************
// ******************** Calculate CRC32 ******************
// *******************************************************
// Cribbed from Bakke. Originated by others.
uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while ( length-- ) {
    uint8_t c = *data++;
    for ( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if ( c & i ) {
        bit = !bit;
      }
      crc <<= 1;
      if ( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
} // calculateCRC32()

// *******************************************************
// ************** Logon to your Wi-Fi router *************
// *******************************************************
void logonToRouter() {
  WiFi.forceSleepWake();       // Bakke
  delay( 1 );
  WiFi.persistent( false );    // prevent it from writing logon to flash memory
  WiFi.mode( WIFI_STA );

  int count = 0;
  String msg = "Config data ";
  if ( rtcValid ) {          // The RTC data was good, make a fast connection
    msg += "OK";
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD, rtcData.channel, rtcData.bssid, true );
  } else {                   // The RTC data was not valid, so make a regular connection
    msg += "not OK";
    WiFi.begin( WIFI_SSID, WIFI_PASSWORD );
  }
  Serial.println( msg );     // print RTC data status
  while ( WiFi.status() != WL_CONNECTED ) {
    count++;
    // give up if more than 100 tries (10 seconds)
    if ( count > 100) {
      Serial.print("\nWiFi failed. Restart after sleep. ");
      enterSleep( SLEEP_INTERVAL );   // retry after sleep
      // **********************************************
      // PROCESSING ENDS HERE IF IT FAILS TO CONNECT!!!
      // **********************************************
    }
    delay( 100 );                     // ms delay between reports
    Serial.print(".");
  } // loop while not connected
  // WiFi is sucesfully connected
  Serial.print("\nWiFi connected to IP: ");
  Serial.println(WiFi.localIP().toString());

  sensorData.wifiRSSI = WiFi.RSSI();  // read the Wi-Fi signal strength (long)
  if ( sensorData.wifiRSSI < MIN_RSSI ) {
    sensorData.lowRSSI = true;
  }
} // logonToRouter()

// *******************************************************
// ********************* Enter Sleep *********************
// *******************************************************
void enterSleep(long sleep) {
  WiFi.disconnect( true );
  delay( 1 );
  // sleep is in seconds
  Serial.print("Total time awake: ");
  Serial.println( rtcData.timeAwake );
  Serial.print("Entering deep sleep for ");
  Serial.print( sleep );
  Serial.println(" seconds.");
  // WAKE_RF_DEFAULT wakes with WiFi radio ON
  // WAKE_RF_DISABLED would use less energy
  ESP.deepSleep( sleep * 1000000L, WAKE_RF_DEFAULT );
} // enterSleep()

// *******************************************************
// ****************** Check Alarm status *****************
// *******************************************************
void checkAlarms() {
  if ( sensorData.bme280Fail == true) {
    String msg = "BME280 sensor failure. ";
    unitStatus += msg;
  } else {
    if ( rtcData.bme280Fail == true && sensorData.bme280Fail == false ) {
      String msg = "BME280 failure cleared. ";
      unitStatus += msg;
    }
  }
  if ( sensorData.bh1750Fail == true) {
    String msg = "BH1750 sensor failure. ";
    unitStatus += msg;
  } else {
    if ( rtcData.bh1750Fail == true && sensorData.bh1750Fail == false ) {
      String msg = "BH1750 failure cleared. ";
      unitStatus += msg;
    }
  }
  if ( sensorData.lowVcell == true) {
    String msg = "Low Vcell alarm. ";
    unitStatus += msg;
  } else {
    if ( rtcData.lowVcell == true && sensorData.lowVcell == false ) {
      String msg = "Low Vcell cleared. ";
      unitStatus += msg;
    }
  }
  if ( sensorData.lowRSSI == true) {
    String msg = "Low RSSI alarm. ";
    unitStatus += msg;
  } else {
    if ( rtcData.lowRSSI == true && sensorData.lowRSSI == false ) {
      String msg = "Low RSSI cleared. ";
      unitStatus += msg;
    }
  }
} // checkAlarms()

// *******************************************************
// ********** Post data to ThingSpeak ********************
// *******************************************************
void postToThingSpeak() {
  // assemble and post the data
  if ( client.connect(IOT_SERVER, 80) == true ) {
    Serial.println("ThingSpeak Server connected.");
    // declare dataString as a String and initialize with the API_WRITE_KEY
    String dataString = API_WRITE_KEY;
    // cocatenate each field onto the end of dataString
    dataString += "&field1=" + String(sensorData.celsius);
    dataString += "&field2=" + String(sensorData.humidity);
    dataString += "&field3=" + String(rtcData.timeAwake);
    dataString += "&field4=" + String(sensorData.seaLevelPressure);
    dataString += "&field5=" + String(sensorData.lightLevel);
    dataString += "&field6=" + String(sensorData.cellVoltage);
    dataString += "&field7=" + String(sensorData.wifiRSSI);
    dataString += "&field8=" + String(sensorData.fahrenheit);
    dataString += "&status=" + unitStatus;
    Serial.println(unitStatus);   // show status on local serial monitor

    // find the number of characters in dataString
    String dataStringLength = String(dataString.length());

    // post the data to ThingSpeak
    client.println("POST /update HTTP/1.1");
    client.println("Host: api.thingspeak.com");
    client.println("Connection: close");
    client.println("X-THINGSPEAKAPIKEY: " + API_WRITE_KEY);
    client.println("Content-Type: application/x-www-form-urlencoded");
    client.println("Content-Length: " + dataStringLength);
    client.println("");
    client.print(dataString);

    Serial.println("ThingSpeak data sent.");
  }
  client.stop();
  Serial.println("ThingSpeak disconnected.");
} // postToThingSpeak()

// *******************************************************
// Calculate relative sea level pressure from absolute station pressure in hPa
// temperature in Â°C, elevation in m
// http://www.sandhurstweather.org.uk/barometric.pdf
// http://keisan.casio.com/exec/system/1224575267
// *******************************************************
float calculateSeaLevelPressure(float celsius, float stationPressure, float elevation) {
  float slP = stationPressure / pow(2.718281828, -(elevation / ((273.15 + celsius) * 29.263)));
  return slP;
} // calculateSeaLevelPressure()

// *******************************************************
// *********************** END ***************************
// *******************************************************
