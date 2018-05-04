/*
MIT License

Copyright (c) 2017 kaebmoo

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

*/

#define BLYNKLOCAL

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

#include <Adafruit_NeoPixel.h>

#include <ArduinoJson.h>
#include <TimeLord.h>

float const LATITUDE = 18.786741;
float const LONGITUDE = 100.782217;


#define BLYNK_MAX_READBYTES 1024

#define PIN            D2
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, PIN, NEO_GRB + NEO_KHZ800);

const int FW_VERSION = 2; // 20180410
const char* LASTUPDATE = "2.20180410";
const char* firmwareUrlBase = "http://www.ogonan.com/ogoupdate/";
String firmwareName = "farmcontrol_blynk_netpie.ino.d1_mini";

// internet control
// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "12345678901234567890abcdefghijkl";
char c_auth[33] = "";           // authen token blynk
bool shouldSaveConfig = false;
WidgetLED led_tank1(50);
WidgetLED led_tank2(51);
WidgetLED led_tank3(52);
WidgetLED led_tank4(53);
WidgetLED led_status(54);
WidgetRTC rtc;

BlynkTimer blynk_timer;
Timer timer1, timer2, timer3, timer4, timer_display, timer_sequence;
int sequence_id = -1;
int afterState1 = -1;
int afterState2 = -1;
int afterState3 = -1;
int working = 0;
boolean automode = false;
int operation = 0;
bool blynkConnectedResult = false;

boolean schedule = false;
boolean WET1 = false;
boolean WET2 = false;
boolean WET3 = false;
boolean WET4 = false;

boolean ON1 = false;
boolean ON2 = false;
boolean ON3 = false;
boolean ON4 = false;

int sunriseOnOff = -1;
int sunsetOnOff = -1;

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

int workingTime, stopPeriod, repeatsTime, timesPerDay;

unsigned long timezoneOffset;
time_t sunriseTime, sunsetTime;
AlarmId alarmIdTime[16] = {dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID, \
                          dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID,dtINVALID_ALARM_ID};

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
int blynkreconnect = 0;


// netpie.io
#include <AuthClient.h>
#include <MicroGear.h>
#include <MQTTClient.h>
#include <SHA1.h>

#define APPID "ogoControl"
#define KEY "SdrEJ2HTAfQmbCB"
#define SECRET "tMh2gL9C7o2VxHaYYgl5FdVep"

WiFiClient client;

MicroGear microgear(client);
String channelName = "aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07";
char *myRoom = "/ogoControl/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07";
char *iamWet1 = "/ogoControl/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/1/wet";
char *iamWet2 = "/ogoControl/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/2/wet";
char *iamWet3 = "/ogoControl/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/3/wet";
char *iamWet4 = "/ogoControl/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/4/wet";
char *ALIAS = "ogoControl-4ch-Somboon";

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

  pixels.begin(); // This initializes the NeoPixel library.
  pixels.setBrightness(64);

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

  workingTime = 28800;
  stopPeriod = 1440;
  repeatsTime = workingTime + stopPeriod;
  timesPerDay = 24;



  delay(2000);

  wifi_connected = setupWifi();
  // wifi_connected = WiFi.begin("Redmi", "12345678"); // WL_CONNECTED
  if (wifi_connected == 0) {
    delay(500);

    #ifdef BLYNKLOCAL
    Blynk.config(auth, "ogoservice.ogonan.com", 80);  // in place of Blynk.begin(auth, ssid, pass);
    #else
    Blynk.config(auth);  // in place of Blynk.begin(auth, ssid, pass);
    #endif
    Serial.print("Blynk connecting : ");
    boolean result = Blynk.connect(3333);  // timeout set to 10 seconds and then continue without Blynk, 3333 is 10 seconds because Blynk.connect is in 3ms units.
    Serial.println(result);
    if(!Blynk.connected()) {
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
  // timer_display.every(1000L, display_zone1, 1);

  upintheair();
  blynk_timer.setInterval(60000L, checkBlynkConnection);
  blynk_timer.setInterval(60000L, syncSchedule);
  syncZone2();
  display_zone1();

  #ifdef NETPIE
  microgear.setEEPROMOffset(512);
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);

  microgear.resetToken();
  microgear.init(KEY, SECRET, ALIAS);
  microgear.connect(APPID);

  String room = "/room/";
  room.concat(channelName);
  room.concat("/1/wet");
  char *cstrSubscribe = new char[room.length() + 1];
  strcpy(cstrSubscribe, room.c_str());
  microgear.subscribe(cstrSubscribe); // "/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/*/wet"
  Serial.println(String("Subscribe: ") + cstrSubscribe);
  /*
  microgear.subscribe("/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/1/wet");
  */
  microgear.subscribe("/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/2/wet");
  microgear.subscribe("/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/3/wet");
  microgear.subscribe("/room/aiJZ77WVXc5YIP0HVrOtvTyzHMiPlR07/4/wet");
  #endif

}

void loop()
{
  int i, blynk_connection;
  uint8_t data[] = { 0x3f, 0x73, 0x79, 0x50 };

  blynk_connection = Blynk.connected();
  if (blynk_connection) {
    Blynk.run();
  }

  timer1.update();
  timer2.update();
  timer3.update();
  timer4.update();
  // timer_display.update();
  timer_sequence.update();
  blynk_timer.run();

  Alarm.delay(0);
  currenttime = (unsigned long) now();


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;



    if(!ON1 && !ON2 && !ON3 && !ON4) {
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

        if (blynk_connection) {
          pixels.setPixelColor(0, 255, 0, 0); // red #FF0000
        }
        else {
          pixels.setPixelColor(0, 255, 165, 0); // orange #FFA500
        }
        pixels.show();

      } else {
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        display.setSegments(data);
        ledState = LOW;
        // led_status.off();

        pixels.setPixelColor(0, 0, 0, 0);
        pixels.show();
      }
    }
    else {
      // led_status.on();
    }

    // set the LED with the ledState of the variable:
    // digitalWrite(ledPin, ledState);
  }

  #ifdef NETPIE
  if (microgear.connected()) {
    microgear.loop();
  }
  #endif
}


void display_zone1()
{
  // if(sequence_id != -1) {
  //   timer_sequence.stop(sequence_id);
  // }
  if(ON1) {
    pixels.setPixelColor(0, 255, 20, 147); // deep pink #FF1493
    pixels.show();
    sequence_id = timer_sequence.after(1000L, display_zone2);
  }
  else {
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
    sequence_id = timer_sequence.after(500L, display_zone2);
  }

  Serial.println(String("Sequence ID Zone 1: ") + sequence_id);
}

void display_zone2()
{
  // timer_sequence.stop(sequence_id);
  if(ON2) {
    pixels.setPixelColor(0, 255, 255, 0); // yellow #FFFF00
    pixels.show();
    sequence_id = timer_sequence.after(1000L, display_zone3);
  }
  else {
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
    sequence_id = timer_sequence.after(500L, display_zone3);
  }

  Serial.println(String("Sequence ID Zone 2: ") + sequence_id);
}

void display_zone3()
{
  // timer_sequence.stop(sequence_id);
  if (ON3) {
    pixels.setPixelColor(0, 0, 128, 0); // green #008000
    pixels.show();
    sequence_id = timer_sequence.after(1000L, display_zone4);
  }
  else {
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
    sequence_id = timer_sequence.after(500L, display_zone4);
  }

  Serial.println(String("Sequence ID Zone 3: ") + sequence_id);
}

void display_zone4()
{
  // timer_sequence.stop(sequence_id);
  if (ON4) {
    pixels.setPixelColor(0, 0, 0, 255); // blue #0000FF
    pixels.show();
    sequence_id = timer_sequence.after(1000L, display_zone1);
  }
  else {
    pixels.setPixelColor(0, 0, 0, 0);
    pixels.show();
    sequence_id = timer_sequence.after(500L, display_zone1);
  }

  Serial.println(String("Sequence ID Zone 4: ") + sequence_id);
}

void checkBlynkConnection() {
  int mytimeout;

  Serial.println("Check Blynk connection.");
  blynkConnectedResult = Blynk.connected();
  if (!blynkConnectedResult) {
    Serial.println("Blynk not connected");

    mytimeout = millis() / 1000;
    Serial.println("Blynk trying to reconnect.");
    while (!blynkConnectedResult) {
      blynkConnectedResult = Blynk.connect(3333);
      if((millis() / 1000) > mytimeout + 3) { // try for less than 4 seconds
        Serial.println("Blynk reconnect timeout.");
        break;
      }
    }
  }
  if (blynkConnectedResult) {
      Serial.println("Blynk connected");
  }
  else {
    Serial.println("Blynk not connected");
    blynkreconnect++;
    Serial.print("blynkreconnect: ");
    Serial.println(blynkreconnect);
    if (blynkreconnect >= 10) {
      // delay(60000);
      // ESP.reset();
    }
  }

  #ifdef NETPIE
  if (!microgear.connected()) {
    Serial.println("netpie connection lost, reconnect...");
    microgear.connect(APPID);
  }
  else {
    Serial.println("netpie connected");
  }
  #endif

}



void upintheair()
{
  String fwURL = String( firmwareUrlBase );
  fwURL.concat( firmwareName );
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );

  Serial.println( "Checking for firmware updates." );
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


int setupWifi() {

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

  }
  else {
    ON2 = false;
    digitalWrite(RELAY2, LOW);
    led_tank2.off();

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
        if (WET1 == false) {
          if (!ON1) {
            relay1_onoff(true);
            Blynk.virtualWrite(V1, 1);
          }
        }
        else if (WET1 == true && ON1) {
          relay1_onoff(false);
          Blynk.virtualWrite(V1, 0);
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
      if ( (currenttime >= starttime2) && (currenttime <= stoptime2) ) { // ยังอยู่ในเวลาที่ตั้งไว้
        if (WET2 == false) {  // ถ้าความชื้นไม่สูง
          if (!ON2) {
            relay2_onoff(true);
            Blynk.virtualWrite(V2, 1);
          }
        }
        else if (WET2 == true && ON2) { // ความชื้นสูง และ relay เปิด ให้สั่งปิด
          relay2_onoff(false);
          Blynk.virtualWrite(V2, 0);
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
    Serial.println(String("start3: ")+bstart3+String(" stop3: ")+bstop3+String(" current3: ")+bcurrent3+String(" force3: ")+force3);
    Serial.println(String("current: ")+currenttime+String(" start3: ")+starttime3+String(" stop3: ")+stoptime3);
    if (bstart3 && bstop3 && bcurrent3 && !force3) {
      if ( (currenttime >= starttime3) && (currenttime <= stoptime3) ) {
        if (WET3 == false) {
          if (!ON3) {
            relay3_onoff(true);
            Blynk.virtualWrite(V3, 1);
          }
        }
        else if (WET3 == true && ON3) {
          relay3_onoff(false);
          Blynk.virtualWrite(V3, 0);
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
    Serial.println(String("start4: ")+bstart4+String(" stop4: ")+bstop4+String(" current4: ")+bcurrent4+String(" force4: ")+force4);
    Serial.println(String("current: ")+currenttime+String(" start4: ")+starttime4+String(" stop4: ")+stoptime4);
    if (bstart4 && bstop4 && bcurrent4 && !force4) {
      if ( (currenttime >= starttime4) && (currenttime <= stoptime4) ) {
        if (WET4 == false) {
          if (!ON4) {
            relay4_onoff(true);
            Blynk.virtualWrite(V4, 1);
          }
        }
        else if (WET4 == true && ON4) {
          relay4_onoff(false);
          Blynk.virtualWrite(V4, 0);
        }
      }
      else if (ON4) {
        relay4_onoff(false);
        Blynk.virtualWrite(V4, 0);
      }
    }
}

void zone1On()
{
  relay1_onoff(true);
  Blynk.virtualWrite(V1, 1);
}

void zone1Off()
{
  relay1_onoff(false);
  Blynk.virtualWrite(V1, 0);
}

void zone2start()
{
  char tz[] = "Asia/Bangkok";

  // Blynk.virtualWrite(V11, 0, 0, 23, 59);
  repeatsTime = (workingTime * 60) + (stopPeriod * 60);
  Serial.print("Working time: ");
  Serial.println(workingTime);
  Serial.print("Stop period time: ");
  Serial.println(stopPeriod);
  Serial.print("Repeats time: ");
  Serial.println(repeatsTime);
  Serial.print("Time per day: ");
  Serial.println(timesPerDay);

  // Alarm.timerRepeat(repeatsTime, zone2Repeats);            // timer for every n seconds, zone2Repeats run after n seconds
  zone2Repeats();
  blynk_timer.setTimer(repeatsTime * 1000L, zone2Repeats, timesPerDay-1);
}

void zone2Repeats() {
  unsigned long workingTimeout;

  Serial.println("zone 2 timer: ");
  Serial.print("timers per day: ");
  Serial.println( timesPerDay );

  Serial.print("working time: ");
  Serial.println( workingTime );

  relay2_onoff(true);
  Blynk.virtualWrite(V2, 1);
  workingTimeout = workingTime * 60 * 1000;
  blynk_timer.setTimeout(workingTimeout, zone2Off);
}

void zone2Off() {
  relay2_onoff(false);
  Blynk.virtualWrite(V2, 0);
}

BLYNK_WRITE(V1)
{
  int pinValue = param.asInt();

  Serial.print("Pin Value: ");
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

  Serial.print("Pin Value: ");
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

  Serial.print("Pin Value: ");
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

  Serial.print("Pin Value: ");
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


BLYNK_WRITE(V69)
{
  int pinValue = param.asInt();

  Serial.print("Pin Value: ");
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
  Serial.print("V1: Start time in secs: ");
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
  timezoneOffset = t.getTZ_Offset();

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
  Serial.println();
}

BLYNK_WRITE(V11)
{

  long startTimeInSecs = param[0].asLong();
  Serial.print("V2: Start time in secs: ");
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
  timezoneOffset = t.getTZ_Offset();

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
  Serial.println();
}

BLYNK_WRITE(V12)
{

  long startTimeInSecs = param[0].asLong();
  Serial.print("V3: Start time in secs: ");
  Serial.println(startTimeInSecs);
  Serial.println();

  TimeInputParam t(param);
  struct tm c_time;
  time_t t_of_day;

  sunriseOnOff = -1;
  sunsetOnOff = -1;
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
    sunriseOnOff = 1;
    bstart3 = true;
    // starttime3
  }
  else if (t.isStartSunset())
  {
    Serial.println("Start at sunset");
    sunsetOnOff = 1;
    bstart3 = true;
    // starttime3
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
    sunriseOnOff = 0;
    bstop3 = true;
    // stoptime3
  }
  else if (t.isStopSunset())
  {
    Serial.println("Stop at sunset");
    sunsetOnOff = 0;
    bstop3 = true;
    // stoptime3
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
  timezoneOffset = t.getTZ_Offset();

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

  if ( (sunriseOnOff != -1) || (sunsetOnOff != -1) ) {

    // sunriseOnOff = -1 sunriseOnOff = 1 (start) sunriseOnOff = 0 (stop)
    if (sunriseOnOff != -1 && bcurrent3 == true) {
      if (sunriseOnOff) {
        // start at sunrise
        starttime3 = sunriseTime;
        if (stoptime3 <= sunriseTime) {
          stoptime3 = stoptime3 + 86400;  // + 24hr
        }
      }
      else {
        // stop at sunrise
        stoptime3 = sunriseTime;
        if (starttime3 >= sunriseTime) {
          stoptime3 = sunriseTime + 86400;
        }
      }
    }

    if (sunsetOnOff != -1 && bcurrent3 == true) {
      if (sunsetOnOff) {
        // start at sunset
        starttime3 = sunsetTime;
        if (stoptime3 <= sunsetTime) { // ? stop at sunrise or stop before sunset
          stoptime3 = stoptime3 + 86400;
        }
  
      }
      else {
        // stop at sunset
        stoptime3 = sunsetTime;
        if (starttime3 >= sunsetTime) {
          stoptime3 = sunsetTime + 86400;
        }
      }
    }
  }

  // setTime((time_t) now());
  String currentTime = String(hour()) + ":" + minute() + ":" + second();
  Serial.print("Current time: ");
  Serial.println(currentTime);
  Serial.println();
  Serial.println(String("start3: ")+bstart3+String(" stop3: ")+bstop3+String(" current3: ")+bcurrent3+String(" force3: ")+force3);
  Serial.println();
}

BLYNK_WRITE(V13)
{

  long startTimeInSecs = param[0].asLong();
  Serial.print("V4: Start time in secs: ");
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
  timezoneOffset = t.getTZ_Offset();

  if (bstart4 && bstop4) {
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
  Serial.println();
}

void syncZone2()
{
  // assigned zone 2 repeats time
  Blynk.syncVirtual(V15);
  Blynk.syncVirtual(V16);
  Blynk.syncVirtual(V17);
  Blynk.syncVirtual(V40);
}

void syncSunriseSunset()
{

  Serial.println("Synchronize sunrise sunset");

  struct tm c_time;
  time_t t_of_day;
  TimeLord nan;

  nan.TimeZone(7 * 60);
  nan.Position(LATITUDE, LONGITUDE);

  c_time.tm_year = year()-1900;
  c_time.tm_mon= month()-1;
  c_time.tm_mday = day();
  c_time.tm_hour = 0;
  c_time.tm_min = 0;
  c_time.tm_sec = 0;
  c_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
  t_of_day = mktime(&c_time);

  byte today[] = {  0, 0, 12, day(), month(), year() }; // store today's date (at noon) in an array for TimeLord to use

  if (nan.SunRise(today)) // if the sun will rise today (it might not, in the [ant]arctic)
  {
  Serial.print("Sunrise: ");
  Serial.print((int) today[tl_hour]);
  Serial.print(":");
  Serial.println((int) today[tl_minute]);
  }
  sunriseTime = (today[tl_hour]*60*60) + (today[tl_minute] * 60);

  if (nan.SunSet(today)) // if the sun will set today (it might not, in the [ant]arctic)
  {
  Serial.print("Sunset: ");
  Serial.print((int) today[tl_hour]);
  Serial.print(":");
  Serial.println((int) today[tl_minute]);
  }
  sunsetTime = (today[tl_hour]*60*60) + (today[tl_minute] * 60);

  sunriseTime = t_of_day + sunriseTime;
  sunsetTime = t_of_day + sunsetTime;
  Serial.println(String("Sunrise Local timezone: ") + sunriseTime);
  Serial.println(String("Sunset Local timezone: ") + sunsetTime);


  /*
  String webhookdata = param.asStr();
  StaticJsonBuffer<1024> jsonBuffer;
  struct tm sunrise_time, sunset_time;
  char buf[255];

  memset(&sunrise_time, 0, sizeof(struct tm));
  memset(&sunset_time, 0, sizeof(struct tm));
  memset(&buf, 0, sizeof(buf));
  sunrise_time.tm_year = year()-1900;
  sunrise_time.tm_mon= month()-1;
  sunrise_time.tm_mday = day();
  sunrise_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
  sunriseTime = mktime(&sunrise_time) - timezoneOffset;

  sunset_time.tm_year = year()-1900;
  sunset_time.tm_mon= month()-1;
  sunset_time.tm_mday = day();
  sunset_time.tm_isdst = -1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
  sunsetTime = mktime(&sunset_time) - timezoneOffset;

  Serial.println(webhookdata);
  JsonObject& root = jsonBuffer.parseObject(webhookdata);
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }
  String results = root["status"];
  Serial.println(String("Status: ") + results);

  String sunrise = root["results"]["sunrise"];
  String sunset = root["results"]["sunset"];

  Serial.println(String("Sunrise: ") + sunrise);
  Serial.println(String("Sunset: ") + sunset);


  strptime(sunrise.c_str(), "H:%M:%S %p", &sunrise_time);
  sunriseTime = mktime(&sunrise_time);
  Serial.println(String("Sunrise time: ") + sunriseTime);
  sunriseTime += timezoneOffset;
  Serial.println(String("Sunrise Local timezone: ") + sunriseTime);

  sunrise_time = *localtime(&sunriseTime);
  strftime(buf, sizeof(buf), "%H:%M", &sunrise_time);
  Serial.println(String("Sunrise time: ") + buf);
  Serial.println(String("Hour: ") + sunrise_time.tm_hour);
  Serial.println(String("Minute: ") + sunrise_time.tm_min);

  strptime(sunset.c_str(), "%H:%M:%S %p", &sunset_time);
  sunsetTime = mktime(&sunset_time);
  Serial.println(String("Sunset time: ") + sunsetTime);
  sunsetTime += timezoneOffset;
  Serial.println(String("Sunset Local timezone: ") + sunsetTime);

  sunset_time = *localtime(&sunsetTime);
  strftime(buf, sizeof(buf), "%H:%M", &sunset_time);
  Serial.println(String("Sunset time: ") + buf);
  Serial.println(String("Hour: ") + sunset_time.tm_hour);
  Serial.println(String("Minute: ") + sunset_time.tm_min);

  */

    /*
    if (alarmIdSunrise == dtINVALID_ALARM_ID) {
      alarmIdSunrise = Alarm.alarmOnce(sunrise_time.tm_hour, sunrise_time.tm_min, 0, sunriseCall);
      Serial.println("sunriseCall Activated");
    }
    else {
      Alarm.free(alarmIdSunrise);
      alarmIdSunrise = dtINVALID_ALARM_ID;
      alarmIdSunrise = Alarm.alarmOnce(sunrise_time.tm_hour, sunrise_time.tm_min, 0, sunriseCall);
      Serial.println("sunriseCall Activated");
    }
    */




    /*
    if (alarmIdSunset == dtINVALID_ALARM_ID) {
      alarmIdSunset = Alarm.alarmOnce(sunset_time.tm_hour, sunset_time.tm_min, 0, sunsetCall);
      Serial.println("sunsetCall Activated");
    }
    else {
      Alarm.free(alarmIdSunset);
      alarmIdSunset = dtINVALID_ALARM_ID;
      alarmIdSunset = Alarm.alarmOnce(sunset_time.tm_hour, sunset_time.tm_min, 0, sunsetCall);
      Serial.println("sunsetCall Activated");
    }
    */

}


// Attach virtual serial terminal to Virtual Pin V1
WidgetTerminal terminal(V0);

BLYNK_WRITE(V20)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 0);
  }

  // terminal.clear();
  terminal.print("param 0: ");
  terminal.println(param[0].asLong());

  terminal.print("param 1: ");
  terminal.println(param[1].asLong());

  terminal.print("Buffer:");
  terminal.println();
  terminal.write(param.getBuffer(), param.getLength());
  terminal.println();
  // Ensure everything is sent
  terminal.flush();

  // Blynk.virtualWriteBinary(V10, param.getBuffer(), param.getLength());
}


BLYNK_WRITE(V21)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 1);
  }

}

BLYNK_WRITE(V22)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 2);
  }

}

BLYNK_WRITE(V23)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 3);
  }

}


BLYNK_WRITE(V24)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 4);
  }
}

BLYNK_WRITE(V25)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 5);
  }
}

BLYNK_WRITE(V26)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 6);
  }
}

BLYNK_WRITE(V27)
{

  TimeInputParam t(param);
  int start = 0;
  int stop = 0;

  if (t.hasStartTime()) {
    start = 1;
  }

  if (t.hasStopTime()) {
    stop = 1;
  }

  if (start == 1 && stop == 1) {
    callAlarmTimer(t.getStartHour(), t.getStartMinute(), t.getStopHour(), t.getStopMinute(), 7);
  }
}


int callAlarmTimer(unsigned int startHour, unsigned int startMin, unsigned int stopHour, unsigned int stopMin, unsigned int timeId)
{
  if (timeId > 7 ) {
    return -1;
  }
  Serial.print("Max Alarm: ");
  Serial.println(dtNBR_ALARMS);

  if (alarmIdTime[timeId] == dtINVALID_ALARM_ID) {
      alarmIdTime[timeId] = Alarm.alarmOnce(startHour, startMin, 0, zone1On);
      Serial.println("Start Time Activated :-)");
  }
  else {
    Alarm.free(alarmIdTime[timeId]);
    alarmIdTime[timeId] = dtINVALID_ALARM_ID;
    alarmIdTime[timeId] = Alarm.alarmOnce(startHour, startMin, 0, zone1On);
    Serial.println("Start Time Activated");
  }

  if (alarmIdTime[timeId+8] == dtINVALID_ALARM_ID) {
      alarmIdTime[timeId+8] = Alarm.alarmOnce(stopHour, stopMin, 0, zone1Off);
      Serial.println("Stop Time Activated :-)");
  }
  else {
    Alarm.free(alarmIdTime[timeId+8]);
    alarmIdTime[timeId+8] = dtINVALID_ALARM_ID;
    alarmIdTime[timeId+8] = Alarm.alarmOnce(stopHour, stopMin, 0, zone1Off);
    Serial.println("Stop Time Activated");
  }
}

BLYNK_WRITE(V15)
{
  // จำนวนครั้งต่อวัน
  // times per day
  timesPerDay = param.asInt();

  Serial.print("Times per day: ");
  Serial.println(timesPerDay);
  Serial.println();
}


BLYNK_WRITE(V16)
{
  // working time minute
  workingTime = param.asInt();

  Serial.print("Working: ");
  Serial.println(workingTime);
  Serial.println();

}

BLYNK_WRITE(V17)
{
  // stop period minute
  stopPeriod = param.asInt();

  Serial.print("Stop period: ");
  Serial.println(stopPeriod);
  Serial.println();
}

BLYNK_WRITE(V40)
{
  // เวลาที่เริ่มทำงาน start time on zone 2
  TimeInputParam t(param);
  long startTimeInSecs = param[0].asLong();

  Serial.print("start time in secs: ");
  Serial.println(startTimeInSecs);
  Serial.println();
  if (t.hasStartTime()) {
    Serial.print("Hour: ");
    Serial.print(t.getStartHour());
    Serial.print(" Min: ");
    Serial.println(t.getStartMinute());
  }

  Alarm.alarmOnce(t.getStartHour(), t.getStartMinute(), 0, zone2start);

}





BLYNK_CONNECTED()
{
  Serial.println("Synchronize Virtual Pin");
  Serial.println("Start Synchronize Time");
  rtc.begin();
  for(int i = 0; i < 10; i++) {
    Blynk.run();
  }

  // Blynk.syncAll();
  Blynk.virtualWrite(V1, digitalRead(RELAY1));
  Blynk.virtualWrite(V2, digitalRead(RELAY2));
  Blynk.virtualWrite(V3, digitalRead(RELAY3));
  Blynk.virtualWrite(V4, digitalRead(RELAY4));

  // Synchonize button
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V2);
  Blynk.syncVirtual(V3);
  Blynk.syncVirtual(V4);

  // Synchonize timer
  Blynk.syncVirtual(V10);
  Blynk.syncVirtual(V11);
  syncSunriseSunset();
  Blynk.syncVirtual(V12);
  Blynk.syncVirtual(V13);

  
  // Synchonize timer zone 1
  Blynk.syncVirtual(V20);
  Blynk.syncVirtual(V21);
  Blynk.syncVirtual(V22);
  Blynk.syncVirtual(V23);
  Blynk.syncVirtual(V24);
  Blynk.syncVirtual(V25);
  Blynk.syncVirtual(V26);
  Blynk.syncVirtual(V27);

  // add schedule check at midnight
  if(!schedule) {
    Serial.println("Sync. Schedule");
    // Alarm.alarmRepeat(0,0,0, syncSchedule);
    Alarm.alarmRepeat(0,0,0, syncSunriseSunset);
    Alarm.alarmRepeat(0,0,0, syncZone2);
    schedule = true;
  }

}

#ifdef NETPIE
void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
  Serial.print("Incoming message --> ");
  Serial.print(topic);
  Serial.print(" : ");

  char strState[msglen];
  for (int i = 0; i < msglen; i++) {
    strState[i] = (char)msg[i];
    Serial.print((char)msg[i]);
  }
  Serial.println();

  String stateStr = String(strState).substring(0, msglen);

  if(strcmp(topic, iamWet1) == 0) {
    if (stateStr == "0") {
      Serial.println("Wet1 = 0");
      WET1 = false;
    } else if (stateStr == "1") {
      Serial.println("Wet1 = 1");
      WET1 = true;
    }
  }
  else if (strcmp(topic, iamWet2) == 0) {
    if (stateStr == "0") {
      Serial.println("Wet2 = 0");
      WET2 = false;
    } else if (stateStr == "1") {
      Serial.println("Wet2 = 1");
      WET2 = true;
    }

  }
  else if (strcmp(topic, iamWet3) == 0) {
    if (stateStr == "0") {
      Serial.println("Wet3 = 0");
      WET3 = false;
    } else if (stateStr == "1") {
      Serial.println("Wet3 = 1");
      WET3 = true;
    }
  }
  else if (strcmp(topic, iamWet4) == 0) {
    if (stateStr == "0") {
      Serial.println("Wet4 = 0");
      WET4 = false;
    } else if (stateStr == "1") {
      Serial.println("Wet4 = 1");
      WET4 = true;
    }
  }

}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
  char *aliasename;

  aliasename = (char *) malloc(sizeof("ogoControl-")+sizeof(String(ESP.getChipId()).c_str() ));
  strcpy(aliasename, "ogoControl-");
  strcat(aliasename, (const char *) String(ESP.getChipId()).c_str() );
  Serial.println("Connected to NETPIE...");
  Serial.print("Aliase name: ");
  Serial.println(aliasename);
  microgear.setName(aliasename);
  free(aliasename);
}

#endif
