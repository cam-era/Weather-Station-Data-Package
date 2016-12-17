/* Mark -1 Weatherstation Receiver
  History
  2016_07_16 use LCD Clock code as base.  Clean up code
  2016_07_17 use TMP36 to report out internal temperature.  This is used for "house" temperature.  Avoid "floating" calculations
  Change time between readings to allow for beacons later.
  2016_07_23  Use the buttons to switch between the states of display.  Not good yet, the display is static.
              Need to tie in the updated measurement
  2016_07_31  Making the button display permanent.  Select "indoors" then the "indoor temp" is displayed
  2016_08_14 include the code from Coordinator A to get data from Xbee. not working
  2016_08_27  making display work
  2016_09_05  adding Wind Speed
  2016_09_10 try to reduce overall size
  2016_09_10 try to used wire.h to send to other arduino with wifi shield.  didnt work, sketch got too big
  2016_09_17 add rain fall
  2016_09_30 add wind direction
  2016_10_07 O  migrate to Arduino Mega, change from sofware serial to hardware for XBee
  2016_10_11 P  Add windGust
  2016_10_29 Q add I2C to a second Uno for Wifi transmission
  2016_12_11 R fixing issues with transmission
  
*/

#include <XBee.h>
#include <Printers.h>
//#include <AltSoftSerial.h>
#include "binary.h"
const byte CONNECTOR_ADDRESS = 42;  //the Uno receiving I2C data
float dhtH, dhtT;  //humidity / temperature from DHT-22
float bmpT, bmpP, p0; //
float sendMillis;
int WindSpeed;
float RainRate;
XBeeWithCallbacks xbee;

//AltSoftSerial SoftSerial;
#define DebugSerial Serial

//#define XBeeSerial SoftSerial (10, 11)
#define XBeeSerial Serial1
#include <Wire.h>
#include <I2C_Anything.h>
#include "RTClib.h" // RTC clock
#include <Adafruit_RGBLCDShield.h>  // shield library
#include <utility/Adafruit_MCP23017.h> //shield library

RTC_DS1307 RTC;
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield(); //LCD shield
const char* myMonths[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};  
int SensorValSum;
int n = 0;
float beaconTime; // timer set on millis(). Apparently, cant be done in int or long
//int beaconSet = 10000; //interval for the master beacon
//int buttons;
float indoorTemp;
int lcdSecondRow;
int windDirection;
int MaxGust, MinGust;
#define DebugSerial Serial
// #define XBeeSerial SoftSerial

void setup () {

  // Setup debug serial output
  DebugSerial.begin(9600);
  DebugSerial.println(F("Starting..."));

  // Setup XBee serial communication
    Serial1.begin(9600);  //PIN 18, 19
  //XBeeSerial.begin(9600);
  xbee.begin(Serial1);
  delay(1);
  DebugSerial.println(F("Xbee started..."));
  
  // Setup callbacks
  xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)&DebugSerial);
  xbee.onResponse(printErrorCb, (uintptr_t)(Print*)&DebugSerial);
  xbee.onZBRxResponse(processRxPacket);

  Wire.begin();
  RTC.begin();
  beaconTime = millis();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // check if the RTC was set
  if (! RTC.isrunning()) {
    Serial.println("Clock is not is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  lcd.clear();
}

void loop () {
  xbee.loop();
  int buttons =  lcd.readButtons(); // reading LCD Button panel.  If not pressed = 0, else choose which value to be displayed
  if (buttons > 0) { //ignore if no button is pressed
    lcd.setCursor(0, 1);
    lcd.print("                 ");  //clear the second row only
    lcdSecondRow = buttons; //set value to button type.
  }

  updateClock();  //update the clock display (row 1)

  if (millis() > (beaconTime + 10000)) {
    updateIndoorTemp(); // use to update the indoor temperature
    beaconTime = millis();
  }
}

void updateIndoorTemp() {

  SensorValSum = 0;

  for (n = 0; n < 200; n++) {
    SensorValSum = SensorValSum + analogRead(A0);
  }
  

  float SensorVal = (SensorValSum / n);
  indoorTemp = (((((SensorVal) / 1024.0) * 5) - 0.5) * 100);  //only for TMP36

}

void updateClock() {
  DateTime now = RTC.now();
  lcd.setCursor(0, 0);
  if (now.hour() < 10) {
    lcd.print(0);
  }
  lcd.print(now.hour(), DEC);
  lcd.setCursor(2, 0);
  lcd.print(":");
  lcd.setCursor(3, 0);
  if (now.minute() < 10) {
    lcd.print(0);
  }
  lcd.print(now.minute(), DEC);
  lcd.setCursor(5, 0);
  lcd.print(":");
  lcd.setCursor(6, 0);
  if (now.second() < 10) {
    lcd.print(0);
     }
  lcd.print(now.second(), DEC);
  lcd.setCursor(9, 0);
  lcd.print(myMonths[now.month() - 1]);
  lcd.setCursor(13, 0);
  lcd.print(now.day());

  lcd.setCursor(0, 1); // second row
  
    
  switch (lcdSecondRow) { // selects which specific information is shown in the second row
    case 1:
      lcd.print("Rain");  //press select button for rain information
      lcd.setCursor(10, 1);
      lcd.print(RainRate);
      break;
    case 2: //press right button, print indoor temperature
      lcd.print("Indoor C");
      lcd.setCursor(10, 1);
      lcd.print(indoorTemp);
      break;
    case 8: //press right button, print indoor temperature
      lcd.print("Out");
      lcd.setCursor(4, 1);
      lcd.print(dhtT);
      lcd.setCursor(10, 1);
      lcd.print(dhtH);
      break;
    case 16: // press left button for wind information
      lcd.print("W");
      lcd.setCursor(2, 1);
      lcd.print(WindSpeed);
      lcd.setCursor(5, 1);
      lcd.print(MaxGust);
      lcd.setCursor(9, 1);
      lcd.print(MinGust);
      lcd.setCursor(12, 1);
      lcd.print(windDirection);
      break;
    case 4: //press down button for pressure
      lcd.print("pressure");
      lcd.setCursor(9, 1);
      lcd.print(p0);

      break;
  }
}

void processRxPacket(ZBRxResponse& rx, uintptr_t) {  //receive the package from Xbee and decode
  Buffer b(rx.getData(), rx.getDataLength());
  uint8_t type = b.remove<uint8_t>();
  DebugSerial.println(b.len());

    lcd.setCursor(0, 1); 
    lcd.print("                 ");  //clear the second row 

  if (type == 1 && b.len() == 40) {
    DebugSerial.print(F("DHT packet received from "));
    printHex(DebugSerial, rx.getRemoteAddress64());
    DebugSerial.println();
    DebugSerial.print(F("Temperature: "));
    dhtT = b.remove<float>();
    DebugSerial.println(dhtT);
    DebugSerial.print(F("Humidity: "));
    dhtH = b.remove<float>();
    DebugSerial.println(dhtH);
    DebugSerial.print(F("BMP Temp  "));
    bmpT = b.remove<float>();
    DebugSerial.println(bmpT);
    DebugSerial.print(F("BMP Pressure  "));
    p0 = b.remove<float>();
    DebugSerial.println(p0);
    DebugSerial.print(F("Windspeed  "));  // s
    WindSpeed = b.remove<float>();
    DebugSerial.println(WindSpeed);
    DebugSerial.print(F("Rain Rate  "));  // s
    RainRate = b.remove<float>();
    DebugSerial.println(RainRate);
    windDirection = b.remove<float>();
    DebugSerial.print(F("Wind Direction  "));  // s
    DebugSerial.println(windDirection);

    MaxGust = b.remove<float>();
    DebugSerial.print(F("MaxGust  "));  // s
    DebugSerial.println(MaxGust);
    
    MinGust = b.remove<float>();
    DebugSerial.print(F("MinGust  "));  // s
    DebugSerial.println(MinGust);
    DebugSerial.println(F("___________________"));

    //start I2C transmission

Wire.beginTransmission (CONNECTOR_ADDRESS);
I2C_writeAnything (dhtH);
I2C_writeAnything (dhtT);
I2C_writeAnything (bmpT);
I2C_writeAnything (p0);
I2C_writeAnything (WindSpeed);
I2C_writeAnything (RainRate);
I2C_writeAnything (windDirection);
I2C_writeAnything (MaxGust);
I2C_writeAnything (MinGust);
I2C_writeAnything (indoorTemp);
Wire.endTransmission (); 

return;

   
  }

  DebugSerial.println(F("Unknown or invalid packet"));
  printResponse(rx, DebugSerial);
}


