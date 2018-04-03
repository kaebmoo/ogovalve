#include "ESP8266WiFi.h"
#include <BlynkSimpleEsp8266.h>
#include <WidgetRTC.h>
#include <EEPROM.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic


#include <Wire.h>
#include "Adafruit_MCP23008.h"

#include <Arduino.h>
#include <TM1637Display.h>

#include <Time.h>
#include "Timer.h"
#include <TimeAlarms.h>

#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

const int FW_VERSION = 1; // 20180326
const char* LASTUPDATE = "1.20180403";
const char* firmwareUrlBase = "http://www.ogonan.com/ogoupdate/";
String mac = "farmcontrol_blynk.ino.d1_mini";

// internet control
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "12345678901234567890abcdefghijkl";
char c_auth[33] = "";           // authen token blynk
bool shouldSaveConfig = false;
WidgetLED led_tank1(30);
WidgetLED led_tank2(31);
WidgetLED led_tank3(32);
WidgetLED led_tank4(33);
WidgetLED led_status(40);
WidgetRTC rtc;

Timer timer1, timer2, timer3, timer4;
int afterState1 = -1;
int afterState2 = -1;
int afterState3 = -1;
int working = 0;
boolean automode = false;
int operation = 0;

boolean schedule = false;

boolean ON1 = false;
boolean ON2 = false;
boolean ON3 = false;
boolean ON4 = false;

boolean bstart1 = false;
boolean bstop1 = false;
boolean bcurrent1 = false;
boolean force1 = false;

boolean bstart2 = false;
boolean bstop2 = false;
boolean bcurrent2 = false;
boolean force2 = false;

boolean bstart3 = false;
boolean bstop3 = false;
boolean bcurrent3 = false;
boolean force3 = false;

boolean bstart4 = false;
boolean bstop4 = false;
boolean bcurrent4 = false;
boolean force4 = false;

unsigned long starttime1;
unsigned long stoptime1;
unsigned long starttime2;
unsigned long stoptime2;
unsigned long starttime3;
unsigned long stoptime3;
unsigned long starttime4;
unsigned long stoptime4;
unsigned long currenttime;

// #define UNO

// UNO 7 segments display
// #define CLK 9
// #define DIO 8
// int ledPin = 13;       // the number of the output pin
// byte buttons[] = {2, 3, 4, 5, 6};

// Wemos D1 mini
#define CLK D3
#define DIO D4
int ledPin = D4;
byte buttons[] = {D0, D5, D6, D7};  // switch
int RELAY1 = D5;
int RELAY2 = D6;
int RELAY3 = D7;
int RELAY4 = D8;

TM1637Display display(CLK, DIO);

// I2C Relayboard test
// connect VDD to power 5V
// connect GND to power GND
// connect SDA to analog 4 (I2C DATA)
// connect SCL to analog 5 (I2C CLOCK)

Adafruit_MCP23008 mcp;


#define DEBOUNCE 10  // button debouncer, how many ms to debounce, 5+ ms is usually plenty

// This handy macro lets us determine how big the array up above is, by checking the size
#define NUMBUTTONS sizeof(buttons)

// we will track if a button is just pressed, just released, or 'currently pressed'
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
byte previous_keystate[NUMBUTTONS], current_keystate[NUMBUTTONS];

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)
int ledState = LOW;                   // ledState used to set the LED

// check wifi status connected
int wifi_connected = 1;


void setup()
{
  byte i;

  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(115200);
  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00 };
  display.setSegments(data);
  display.setBrightness(0x0a);

  data[0] = 0x06;
  data[1] = 0x54;
  data[2] = 0x06;
  data[3] = 0x78;

  display.setSegments(data);

  pinMode(ledPin, OUTPUT);
  Serial.println();

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);
  digitalWrite(RELAY1, LOW);
  digitalWrite(RELAY2, LOW);
  digitalWrite(RELAY3, LOW);
  digitalWrite(RELAY4, LOW);


  delay(2000);

  wifi_connected = setup_wifi();
  if (wifi_connected == 0) {
    delay(500);
    Blynk.config(auth);  // in place of Blynk.begin(auth, ssid, pass);
    Serial.print("Blynk connecting : ");
    boolean result = Blynk.connect(3333);  // timeout set to 10 seconds and then continue without Blynk, 3333 is 10 seconds because Blynk.connect is in 3ms units.
    Serial.println(result);
    if(!Blynk.connected()){
      Serial.println("Not connected to Blynk server");
      Blynk.connect(3333);  // try to connect to server with default timeout
    }
    else {
      Serial.println("Connected to Blynk server");
    }

  }
  else {
    Serial.println("WiFi not connect");
  }

  timer1.every(1000L, checkvalidtime1);
  timer2.every(1000L, checkvalidtime2);
  timer3.every(1000L, checkvalidtime3);
  timer4.every(1000L, checkvalidtime4);

  upintheair();
}

void loop()
{
  int i;
  uint8_t data[] = { 0x3f, 0x73, 0x79, 0x50 };

  if (wifi_connected == 0) {
    Blynk.run();
  }

  timer1.update();
  timer2.update();
  timer3.update();
  timer4.update();
  Alarm.delay(0);
  currenttime = (unsigned long) now();
  

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    if(working == 0) {
      // if the LED is off turn it on and vice-versa:
      if (ledState == LOW) {
        data[0] = 0x3f;
        data[1] = 0x73;
        data[2] = 0x79;
        data[3] = 0x50;
        display.setSegments(data);
        ledState = HIGH;
        // led_status.on();
        //Serial.println(digitalRead(D8));    // wemos input
      } else {
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        display.setSegments(data);
        ledState = LOW;
        // led_status.off();
      }
    }
    else {
      // led_status.on();
    }

    // set the LED with the ledState of the variable:
    // digitalWrite(ledPin, ledState);
  }

}

void upintheair()
{  
  String fwURL = String( firmwareUrlBase );
  fwURL.concat( mac );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );

  Serial.println( "Checking for firmware updates." );
  // Serial.print( "MAC address: " );
  // Serial.println( mac );
  Serial.print( "Firmware version URL: " );
  Serial.println( fwVersionURL );

  HTTPClient httpClient;
  httpClient.begin( fwVersionURL );
  int httpCode = httpClient.GET();
  if( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();

    Serial.print( "Current firmware version: " );
    Serial.println( FW_VERSION );
    Serial.print( "Available firmware version: " );
    Serial.println( newFWVersion );

    int newVersion = newFWVersion.toInt();

    if( newVersion > FW_VERSION ) {
      Serial.println( "Preparing to update" );

      String fwImageURL = fwURL;
      fwImageURL.concat( ".bin" );
      t_httpUpdate_return ret = ESPhttpUpdate.update( fwImageURL );

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
      }
    }
    else {
      Serial.println( "Already on latest version" );
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
  }
  httpClient.end();
  // ESPhttpUpdate.update("www.ogonan.com", 80, "/ogoupdate/farmcontrol_blynk.ino.d1_mini.bin");
}


int setup_wifi() {

  WiFiManager wifiManager;
  String APName;

  EEPROM.begin(512);
  readEEPROM(auth, 60, 32);
  Serial.print("auth token : ");
  Serial.println(auth);
  int saved = eeGetInt(500);
  if (saved == 6550) {
    strcpy(c_auth, auth);
  }
  WiFiManagerParameter custom_c_auth("c_auth", "Auth Token", c_auth, 37);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_c_auth);


  delay(100);
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);


  APName = "ogoControl-"+String(ESP.getChipId());
  if(!wifiManager.autoConnect(APName.c_str()) ) {
    Serial.println("failed to connect and hit timeout");
    delay(1000);
    // reset and try again, or maybe put it to deep sleep
    // ESP.reset();
    // delay(5000);
    return -1;
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");



  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (shouldSaveConfig) {
    strcpy(c_auth, custom_c_auth.getValue());
    strcpy(auth, c_auth);
    Serial.print("auth token : ");
    Serial.println(auth);
    writeEEPROM(auth, 60, 32);
    eeWriteInt(500, 6550);
  }

  return 0;
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void readEEPROM(char* buff, int offset, int len) {
    int i;
    for (i=0;i<len;i++) {
        buff[i] = (char)EEPROM.read(offset+i);
    }
    buff[len] = '\0';
}

void writeEEPROM(char* buff, int offset, int len) {
    int i;
    for (i=0;i<len;i++) {
        EEPROM.write(offset+i,buff[i]);
    }
    EEPROM.commit();
}

void eeWriteInt(int pos, int val) {
    byte* p = (byte*) &val;
    EEPROM.write(pos, *p);
    EEPROM.write(pos + 1, *(p + 1));
    EEPROM.write(pos + 2, *(p + 2));
    EEPROM.write(pos + 3, *(p + 3));
    EEPROM.commit();
}

int eeGetInt(int pos) {
  int val;
  byte* p = (byte*) &val;
  *p        = EEPROM.read(pos);
  *(p + 1)  = EEPROM.read(pos + 1);
  *(p + 2)  = EEPROM.read(pos + 2);
  *(p + 3)  = EEPROM.read(pos + 3);
  return val;
}

byte thisSwitch_justPressed() {
  byte thisSwitch = 255;
  check_switches();  //check the switches &amp; get the current state
  for (byte i = 0; i < NUMBUTTONS; i++) {
    current_keystate[i]=justpressed[i];
    if (current_keystate[i] != previous_keystate[i]) {
      if (current_keystate[i]) thisSwitch=i;
    }
    previous_keystate[i]=current_keystate[i];
  }
  return thisSwitch;
}

void check_switches()
{
  static byte previousstate[NUMBUTTONS];
  static byte currentstate[NUMBUTTONS];
  static long lasttime;
  byte index;
  if (millis() < lasttime) {
     lasttime = millis(); // we wrapped around, lets just try again
  }

  if ((lasttime + DEBOUNCE) > millis()) {
    return; // not enough time has passed to debounce
  }
  // ok we have waited DEBOUNCE milliseconds, lets reset the timer
  lasttime = millis();

  for (index = 0; index < NUMBUTTONS; index++) {
    justpressed[index] = 0;       // when we start, we clear out the "just" indicators
    justreleased[index] = 0;

    currentstate[index] = digitalRead(buttons[index]);   // read the button
    if (currentstate[index] == previousstate[index]) {
      if ((pressed[index] == LOW) && (currentstate[index] == LOW)) {
          // just pressed
          justpressed[index] = 1;
      }
      else if ((pressed[index] == HIGH) && (currentstate[index] == HIGH)) {
          // just released
          justreleased[index] = 1;
      }
      pressed[index] = !currentstate[index];  // remember, digital HIGH means NOT pressed
    }
    //Serial.println(pressed[index], DEC);
    previousstate[index] = currentstate[index];   // keep a running tally of the buttons
  }
}



//     a
//  f     b
//     g
//  e     c
//     d
//   0000 0000
//    gfe dcba
// n  101 0100
// d  101 1110


void relay1_onoff(boolean set)
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(115, (0x80 >> 1), false, 3, 1);


  Serial.println("Valve 1");

  if (set) {
    ON1 = true;
    digitalWrite(RELAY1, HIGH);
    led_tank1.on();
    
  }
  else {
    ON1 = false;
    digitalWrite(RELAY1, LOW);
    led_tank1.off();
    
  }

  // afterState1 = timer1.after(CLEANDELAY0, tank1_state2);

}


void relay2_onoff(boolean set)
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(215, (0x80 >> 1), false, 3, 1);


  Serial.println("Valve 2");

  if (set) {
    ON2 = true;
    digitalWrite(RELAY2, HIGH);
    led_tank2.on();
    working = 1;
  }
  else {
    ON2 = false;
    digitalWrite(RELAY2, LOW);
    led_tank2.off();
    working = 0;
  }

  // afterState1 = timer1.after(CLEANDELAY0, tank2_state2);

}

void relay3_onoff(boolean set)
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(315, (0x80 >> 1), false, 3, 1);


  Serial.println("Valve 3");

  if (set) {
    ON3 = true;
    digitalWrite(RELAY3, HIGH);
    led_tank3.on();
    
  }
  else {
    ON3 = false;
    digitalWrite(RELAY3, LOW);
    led_tank3.off();
    
  }

  // afterState1 = timer1.after(CLEANDELAY0, tank3_state2);

}

void relay4_onoff(boolean set)
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(415, (0x80 >> 1), false, 3, 1);


  Serial.println("Valve 4");


  if (set) {
    ON4 = true;
    digitalWrite(RELAY4, HIGH);
    led_tank4.on();
    
  }
  else {
    ON4 = false;
    digitalWrite(RELAY4, LOW);
    led_tank4.off();
    
  }
  // afterState1 = timer1.after(CLEANDELAY0, tank4_state2);

}

void syncSchedule()
{
  String syncTime = String(hour()) + ":" + minute() + ":" + second();
  Serial.print("Synchronize time: ");
  Serial.println(syncTime);
  
  Blynk.syncVirtual(V10);
  Blynk.syncVirtual(V11);
  Blynk.syncVirtual(V12);
  Blynk.syncVirtual(V13);
}

void checkvalidtime1()
{
    Serial.println(String("start1: ")+bstart1+String(" stop1: ")+bstop1+String(" current1: ")+bcurrent1+String(" force1: ")+force1);
    Serial.println(String("current: ")+currenttime+String(" start: ")+starttime1+String(" stop: ")+stoptime1);
    if (bstart1 && bstop1 && bcurrent1 && !force1) {
      if ( (currenttime >= starttime1) && (currenttime <= stoptime1) ) {
        if (!ON1) {
          relay1_onoff(true);
          Blynk.virtualWrite(V1, 1);
        }
      }
      else if (ON1) {
        relay1_onoff(false);
        Blynk.virtualWrite(V1, 0);
        
      }
    }
}

void checkvalidtime2()
{
    Serial.println(String("start2: ")+bstart2+String(" stop2: ")+bstop2+String(" current2: ")+bcurrent2+String(" force2: ")+force2);
    Serial.println(String("current: ")+currenttime+String(" start2: ")+starttime2+String(" stop2: ")+stoptime2);
    if (bstart2 && bstop2 && bcurrent2 && !force2) {
      if ( (currenttime >= starttime2) && (currenttime <= stoptime2) ) {
        if (!ON2) {
          relay2_onoff(true);
          Blynk.virtualWrite(V2, 1);
        }
      }
      else if (ON2) {
        relay2_onoff(false);
        Blynk.virtualWrite(V2, 0);
        
      }
    }
}

void checkvalidtime3()
{
    if (bstart3 && bstop3 && bcurrent3 && !force3) {
      if ( (currenttime >= starttime3) && (currenttime <= stoptime3) ) {
        if (!ON3) {
          relay3_onoff(true);
          Blynk.virtualWrite(V3, 1);
        }
      }
      else if (ON3) {
        relay3_onoff(false);
        Blynk.virtualWrite(V3, 0);
        
      }
    }
}

void checkvalidtime4()
{
    if (bstart4 && bstop4 && bcurrent4 && !force4) {
      if ( (currenttime >= starttime4) && (currenttime <= stoptime4) ) {
        if (!ON4) {
          relay4_onoff(true);
          Blynk.virtualWrite(V4, 1);
        }
      }
      else if (ON4) {
        relay4_onoff(false);
        Blynk.virtualWrite(V4, 0);
        
      }
    }
}

BLYNK_WRITE(V1)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 1 just pressed");
    
    relay1_onoff(true);
    force1 = true;
    bstart1 = false;
    bstop1 = false;
  }
  else {
    relay1_onoff(false);
    force1 = true;
    bstart1 = false;
    bstop1 = false;
  }

}

BLYNK_WRITE(V2)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 2 just pressed");
    
    relay2_onoff(true);
    force2 = true;
    bstart2 = false;
    bstop2 = false;
  }
  else {
    relay2_onoff(false);
    force2 = true;
    bstart2 = false;
    bstop2 = false;
  }
}

BLYNK_WRITE(V3)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 3 just pressed");
    
    relay3_onoff(true);
    force3 = true;
    bstart3 = false;
    bstop3 = false;
  }
  else {
    relay3_onoff(false);
    force3 = true;
    bstart3 = false;
    bstop3 = false;
  }

}

BLYNK_WRITE(V4)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 4 just pressed");
    
    relay4_onoff(true);
    force4 = true;
    bstart4 = false;
    bstop4 = false;
  }
  else {
    relay4_onoff(false);
    force4 = true;
    bstart4 = false;
    bstop4 = false;
  }
}


BLYNK_WRITE(V20)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    delay(500);
    ESP.reset();
    delay(5000);
  }
}

BLYNK_WRITE(V10)
{

  long startTimeInSecs = param[0].asLong();
  Serial.print("Start time in secs: ");
  Serial.println(startTimeInSecs);
  Serial.println();

  TimeInputParam t(param);
  struct tm c_time;
  time_t t_of_day;

  // Process start time

  if (t.hasStartTime())
  {
    Serial.println(String("Start: ") +
                   t.getStartHour() + ":" +
                   t.getStartMinute() + ":" +
                   t.getStartSecond());



     Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStartHour();
     c_time.tm_min = t.getStartMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Start seconds since the Epoch: ") + t_of_day);
     starttime1 = t_of_day;
     bstart1 = true;
  }
  else if (t.isStartSunrise())
  {
    Serial.println("Start at sunrise");
  }
  else if (t.isStartSunset())
  {
    Serial.println("Start at sunset");
  }
  else
  {
    // Do nothing
  }

  // Process stop time

  if (t.hasStopTime())
  {
    Serial.println(String("Stop: ") +
                   t.getStopHour() + ":" +
                   t.getStopMinute() + ":" +
                   t.getStopSecond());
    Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStopHour();
     c_time.tm_min = t.getStopMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Stop seconds since the Epoch: ") + t_of_day);
     stoptime1 = t_of_day;
     bstop1 = true;
  }
  else if (t.isStopSunrise())
  {
    Serial.println("Stop at sunrise");
  }
  else if (t.isStopSunset())
  {
    Serial.println("Stop at sunset");
  }
  else
  {
    // Do nothing: no stop time was set
  }

  // Process timezone
  // Timezone is already added to start/stop time

  Serial.println(String("Time zone: ") + t.getTZ());

  // Get timezone offset (in seconds)
  Serial.println(String("Time zone offset: ") + t.getTZ_Offset());

  if (bstart1 && bstop1) {
    force1 = false;
  }


  // weekday();         // day of the week (1-7), Sunday is day 1
  // 1. Sunday, 2. Mon, 3. Tue, ...
  Serial.println(String("Weekday ") + weekday());
  int iWeekday;
  iWeekday = weekday() - 1;
  if (iWeekday == 0) {
    iWeekday = 7;
  }

  int WorkingDay[7] = {0,0,0,0,0,0,0};

  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  for (int i = 1; i <= 7; i++) {
    if (t.isWeekdaySelected(i)) {
      Serial.println(String("Day ") + i + " is selected");
      WorkingDay[i-1] = 1;
    }
  }

  if (WorkingDay[iWeekday-1] == 1) {
    Serial.println("Working day");
    bcurrent1 = true;
  }
  else {
    bcurrent1 = false;
  }

  // setTime((time_t) now());
  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  Serial.print("Current time: ");
  Serial.println(currentTime);
  Serial.println();
  Serial.println(String("start1: ")+bstart1+String(" stop1: ")+bstop1+String(" current1: ")+bcurrent1+String(" force1: ")+force1);   
}

BLYNK_WRITE(V11)
{

  long startTimeInSecs = param[0].asLong();
  Serial.print("Start time in secs: ");
  Serial.println(startTimeInSecs);
  Serial.println();

  TimeInputParam t(param);
  struct tm c_time;
  time_t t_of_day;

  // Process start time

  if (t.hasStartTime())
  {
    Serial.println(String("Start: ") +
                   t.getStartHour() + ":" +
                   t.getStartMinute() + ":" +
                   t.getStartSecond());



     Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStartHour();
     c_time.tm_min = t.getStartMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Start seconds since the Epoch: ") + t_of_day);
     starttime2 = t_of_day;
     bstart2 = true;
  }
  else if (t.isStartSunrise())
  {
    Serial.println("Start at sunrise");
  }
  else if (t.isStartSunset())
  {
    Serial.println("Start at sunset");
  }
  else
  {
    // Do nothing
  }

  // Process stop time

  if (t.hasStopTime())
  {
    Serial.println(String("Stop: ") +
                   t.getStopHour() + ":" +
                   t.getStopMinute() + ":" +
                   t.getStopSecond());
    Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStopHour();
     c_time.tm_min = t.getStopMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Stop seconds since the Epoch: ") + t_of_day);
     stoptime2 = t_of_day;
     bstop2 = true;
  }
  else if (t.isStopSunrise())
  {
    Serial.println("Stop at sunrise");
  }
  else if (t.isStopSunset())
  {
    Serial.println("Stop at sunset");
  }
  else
  {
    // Do nothing: no stop time was set
  }

  // Process timezone
  // Timezone is already added to start/stop time

  Serial.println(String("Time zone: ") + t.getTZ());

  // Get timezone offset (in seconds)
  Serial.println(String("Time zone offset: ") + t.getTZ_Offset());

  if (bstart2 && bstop2) {
    force2 = false;
  }


  // weekday();         // day of the week (1-7), Sunday is day 1
  // 1. Sunday, 2. Mon, 3. Tue, ...
  Serial.println(String("Weekday ") + weekday());
  int iWeekday;
  iWeekday = weekday() - 1;
  if (iWeekday == 0) {
    iWeekday = 7;
  }

  int WorkingDay[7] = {0,0,0,0,0,0,0};

  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  for (int i = 1; i <= 7; i++) {
    if (t.isWeekdaySelected(i)) {
      Serial.println(String("Day ") + i + " is selected");
      WorkingDay[i-1] = 1;
    }
  }

  if (WorkingDay[iWeekday-1] == 1) {
    Serial.println("Working day");
    bcurrent2 = true;
  }
  else {
    bcurrent2 = false;
  }

  // setTime((time_t) now());
  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  Serial.print("Current time: ");
  Serial.println(currentTime);
  Serial.println();
  Serial.println(String("start2: ")+bstart2+String(" stop2: ")+bstop2+String(" current2: ")+bcurrent2+String(" force2: ")+force2);
}

BLYNK_WRITE(V12)
{

  long startTimeInSecs = param[0].asLong();
  Serial.print("Start time in secs: ");
  Serial.println(startTimeInSecs);
  Serial.println();

  TimeInputParam t(param);
  struct tm c_time;
  time_t t_of_day;

  // Process start time

  if (t.hasStartTime())
  {
    Serial.println(String("Start: ") +
                   t.getStartHour() + ":" +
                   t.getStartMinute() + ":" +
                   t.getStartSecond());



     Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStartHour();
     c_time.tm_min = t.getStartMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Start seconds since the Epoch: ") + t_of_day);
     starttime3 = t_of_day;
     bstart3 = true;
  }
  else if (t.isStartSunrise())
  {
    Serial.println("Start at sunrise");
  }
  else if (t.isStartSunset())
  {
    Serial.println("Start at sunset");
  }
  else
  {
    // Do nothing
  }

  // Process stop time

  if (t.hasStopTime())
  {
    Serial.println(String("Stop: ") +
                   t.getStopHour() + ":" +
                   t.getStopMinute() + ":" +
                   t.getStopSecond());
    Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStopHour();
     c_time.tm_min = t.getStopMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Stop seconds since the Epoch: ") + t_of_day);
     stoptime3 = t_of_day;
     bstop3 = true;
  }
  else if (t.isStopSunrise())
  {
    Serial.println("Stop at sunrise");
  }
  else if (t.isStopSunset())
  {
    Serial.println("Stop at sunset");
  }
  else
  {
    // Do nothing: no stop time was set
  }

  // Process timezone
  // Timezone is already added to start/stop time

  Serial.println(String("Time zone: ") + t.getTZ());

  // Get timezone offset (in seconds)
  Serial.println(String("Time zone offset: ") + t.getTZ_Offset());

  if (bstart3 && bstop3) {
    force3 = false;
  }


  // weekday();         // day of the week (1-7), Sunday is day 1
  // 1. Sunday, 2. Mon, 3. Tue, ...
  Serial.println(String("Weekday ") + weekday());
  int iWeekday;
  iWeekday = weekday() - 1;
  if (iWeekday == 0) {
    iWeekday = 7;
  }

  int WorkingDay[7] = {0,0,0,0,0,0,0};

  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  for (int i = 1; i <= 7; i++) {
    if (t.isWeekdaySelected(i)) {
      Serial.println(String("Day ") + i + " is selected");
      WorkingDay[i-1] = 1;
    }
  }

  if (WorkingDay[iWeekday-1] == 1) {
    Serial.println("Working day");
    bcurrent3 = true;
  }
  else {
    bcurrent3 = false;
  }

  // setTime((time_t) now());
  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  Serial.print("Current time: ");
  Serial.println(currentTime);
  Serial.println();
  Serial.println(String("start3: ")+bstart3+String(" stop3: ")+bstop3+String(" current3: ")+bcurrent3+String(" force3: ")+force3);
}

BLYNK_WRITE(V13)
{

  long startTimeInSecs = param[0].asLong();
  Serial.print("Start time in secs: ");
  Serial.println(startTimeInSecs);
  Serial.println();

  TimeInputParam t(param);
  struct tm c_time;
  time_t t_of_day;

  // Process start time

  if (t.hasStartTime())
  {
    Serial.println(String("Start: ") +
                   t.getStartHour() + ":" +
                   t.getStartMinute() + ":" +
                   t.getStartSecond());



     Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStartHour();
     c_time.tm_min = t.getStartMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Start seconds since the Epoch: ") + t_of_day);
     starttime4 = t_of_day;
     bstart4 = true;
  }
  else if (t.isStartSunrise())
  {
    Serial.println("Start at sunrise");
  }
  else if (t.isStartSunset())
  {
    Serial.println("Start at sunset");
  }
  else
  {
    // Do nothing
  }

  // Process stop time

  if (t.hasStopTime())
  {
    Serial.println(String("Stop: ") +
                   t.getStopHour() + ":" +
                   t.getStopMinute() + ":" +
                   t.getStopSecond());
    Serial.println(String("Year: ") + year() + String(" Month: ") + month() + String(" Day: ") + day());
     c_time.tm_year = year()-1900;
     c_time.tm_mon= month()-1;
     c_time.tm_mday = day();
     c_time.tm_hour = t.getStopHour();
     c_time.tm_min = t.getStopMinute();
     c_time.tm_sec = 0;
     c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
     t_of_day = mktime(&c_time);
     // printf("seconds since the Epoch: %ld\n", (long) t_of_day)
     Serial.println(String("Stop seconds since the Epoch: ") + t_of_day);
     stoptime4 = t_of_day;
     bstop4 = true;
  }
  else if (t.isStopSunrise())
  {
    Serial.println("Stop at sunrise");
  }
  else if (t.isStopSunset())
  {
    Serial.println("Stop at sunset");
  }
  else
  {
    // Do nothing: no stop time was set
  }

  // Process timezone
  // Timezone is already added to start/stop time

  Serial.println(String("Time zone: ") + t.getTZ());

  // Get timezone offset (in seconds)
  Serial.println(String("Time zone offset: ") + t.getTZ_Offset());

  if (bstart4 && bstop3) {
    force4 = false;
  }


  // weekday();         // day of the week (1-7), Sunday is day 1
  // 1. Sunday, 2. Mon, 3. Tue, ...
  Serial.println(String("Weekday ") + weekday());
  int iWeekday;
  iWeekday = weekday() - 1;
  if (iWeekday == 0) {
    iWeekday = 7;
  }

  int WorkingDay[7] = {0,0,0,0,0,0,0};

  // Process weekdays (1. Mon, 2. Tue, 3. Wed, ...)
  for (int i = 1; i <= 7; i++) {
    if (t.isWeekdaySelected(i)) {
      Serial.println(String("Day ") + i + " is selected");
      WorkingDay[i-1] = 1;
    }
  }

  if (WorkingDay[iWeekday-1] == 1) {
    Serial.println("Working day");
    bcurrent4 = true;
  }
  else {
    bcurrent4 = false;
  }

  // setTime((time_t) now());
  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  Serial.print("Current time: ");
  Serial.println(currentTime);
  Serial.println();
  Serial.println(String("start4: ")+bstart4+String(" stop4: ")+bstop4+String(" current4: ")+bcurrent4+String(" force4: ")+force4);
}


BLYNK_CONNECTED()
{
  Serial.println("Blynk Connected");
  rtc.begin();
  
  // Blynk.syncAll();
  
  Blynk.syncVirtual(V10);
  Blynk.syncVirtual(V11);
  Blynk.syncVirtual(V12);
  Blynk.syncVirtual(V13);
  
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V4);

  
  if(!schedule) {
    Serial.println("Sync. Schedule");
    Alarm.alarmRepeat(0,0,0, syncSchedule);  
    schedule = true; 
  }
  
}
