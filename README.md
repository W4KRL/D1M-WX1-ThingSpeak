# D1M-WX1-ThingSpeak

Version V01 - 04/23/2020 - Changed SLEEP_INTERVAL to unsigned long
                         - sprintf for PrintToSerial
                         - Expanded range for light intensity
## Installation
1. Click on the **Clone or download** button and select Download ZIP. The file will download as **D1M-WX1-ThingSpeak-master.zip**. 
2. Unzip the file. The unzipped folder will contain a LICENSE and README.md file and a folder **D1M-WX1-ThingSpeak**. Copy this folder to your Arduino sketchbook folder. If you need to find your sketchbook folder, open the Arduino IDE and use menu File | Preferences. The first line tells you where the sketchbook resides on your computer. 

### Add these Libraries to the Arduino IDE:
* BME280 by Tyler Glenn https://github.com/finitespace/BME280 - download as zip. iT WILL BE SAVED ON YOUR COMPUTER AS bme280-MASTER.ZIP. cHANGE THE NAME TO bme280.ZIP. Use menu **Sketch | Include Library | Add .ZIP...**
* hp_BH1750 by Stefan Armborst - use menu **Sketch | Include Libraries | Manage Libraries...** and search for hp_BH1750. The reference for this excellent, but complicated, library is at https://github.com/Starmbi/hp_BH1750

## Configure the ThingSpeak_config.h file
Information unique to your weather station must be added to the *ThingSpeak_config.h* file. 

### Information needed:
- Your WiFi SSID **(You must use 2.4 GHz not 5 GHz.)**
- Your WiFi password
- Sleep interval in seconds: 60 for testing, 600 or longer for normal service. Maximum is 4,294 seconds
- ThingSpeak channel ID (a numerical value)
- ThingSpeak API Write Key (alphanumeric between quotes)
- OPTIONAL (Values determined from running *D1M-WX1_Calibration.ino*)
  - DMM voltage
  - ADC reading

Save the sketch. Set the PROG/RUN switch to **PROG** and upload to the microcontroller. **Return the switch to RUN after a sucessful upload.**
