#include "ESP8266WiFi.h"
#include <BlynkSimpleEsp8266.h>
#include <EEPROM.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic


#include <Wire.h>
#include "Adafruit_MCP23008.h"

#include <Arduino.h>
#include <TM1637Display.h>


#include "Timer.h"

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

Timer timer1, timer2, timer3;
int afterState1 = -1;
int afterState2 = -1;
int afterState3 = -1;
int working = 0;
boolean automode = false;
int operation = 0;

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

// 5 minutes = 300000 ms
// 1 minute = 60000 ms
int CLEANDELAY0 = 500;
int CLEANDELAY1 = 300000;
int CLEANDELAY2 = 60000;
int DELAYSTATE = 5000;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)
int ledState = LOW;             // ledState used to set the LED

// wemos
int wemosautostart = 1;

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
  Serial.print("Button checker with ");
  Serial.print(NUMBUTTONS, DEC);
  Serial.println(" buttons");

  // Make input & enable pull-up resistors on switch pins
  for (i=0; i < NUMBUTTONS; i++) {
    pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
  }

  pinMode(D8, INPUT);
  digitalWrite(D8, LOW);

  // mcp.begin(1); // address = 0 (valid: 0-7)
  setup_relayboard(0);

  while (!Serial); // wait for serial port to connect. Needed for Leonardo only
  Serial.println("I2C Relayboard test - press keys 12345678 (toggle relay) C (clear all)");
  relay_reset();
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
      led_tank1.off();
      led_tank2.off();
      led_tank3.off();
      led_tank4.off();
    }
    
  } 
  else {
    Serial.println("WiFi not connect");
  }
   
  
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
        led_status.on();
        //Serial.println(digitalRead(D8));    // wemos input
      } else {
        data[0] = 0x00;
        data[1] = 0x00;
        data[2] = 0x00;
        data[3] = 0x00;
        display.setSegments(data);
        ledState = LOW;
        led_status.off();
      }
    }
    else {
      led_status.on();
    }

    // set the LED with the ledState of the variable:
    // digitalWrite(ledPin, ledState);
  }



  byte thisSwitch=thisSwitch_justPressed();
  switch(thisSwitch)
  {
    case 0:
      Serial.println("switch 1 just pressed");
      automode = false;
      
      setup_relayboard(0);
      if (working == 0) {
        tank1_clean();
      }
      break;
    case 1:
      Serial.println("switch 2 just pressed");
      automode = false;
      
      setup_relayboard(0);
      if (working == 0) {
        tank2_clean();
      }
      break;
    case 2:
      Serial.println("switch 3 just pressed");
      automode = false;
      
      setup_relayboard(1);
      if(working == 0) {
        tank3_clean();
      }
      break;
    case 3:
      Serial.println("switch 4 just pressed");
      automode = false;
      
      setup_relayboard(1);
      if (working == 0) {
        tank4_clean();
      }
      break;
    case 4:
      // automatic clean
      Serial.println("switch 5 just pressed");
      automode = true;      
      if (working == 0) {
        setup_relayboard(0);
        operation = 1;
        tank1_clean();
        // tank2_clean();
        // setup_relayboard(1);
        // tank3_clean();
        // tank4_clean();
      }
      break;
    case 5:
      Serial.println("switch 6 just pressed"); break;
  }

  // wemos auto start D8 input
  if (digitalRead(D8) == 1 && wemosautostart == 1) {
    Serial.println("switch 5 just pressed");
    automode = true;
    
    // automatic clean
    if (working == 0) {
      setup_relayboard(0);
      operation = 1;
      tank1_clean();
      // tank2_clean();
      // setup_relayboard(1);
      // tank3_clean();
      // tank4_clean();
    }
  }
  // end wemos auto start

  relay_serial_control();


}

int setup_wifi() {

  WiFiManager wifiManager;
  String APName;

  EEPROM.begin(512);
  readEEPROM(auth, 60, 32);
  Serial.print("auth token : ");
  Serial.println(auth);
  WiFiManagerParameter custom_c_auth("c_auth", "Auth Token", c_auth, 37);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_c_auth);


  delay(100);
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);


  APName = "OgoControl-"+String(ESP.getChipId());
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



void setup_relayboard(int board)
{
  mcp.begin(board);
  mcp.writeGPIO(0); // set OLAT to 0
  mcp.pinMode(0, OUTPUT); // set IODIR to 0
  mcp.pinMode(1, OUTPUT);
  mcp.pinMode(2, OUTPUT);
  mcp.pinMode(3, OUTPUT);
  mcp.pinMode(4, OUTPUT);
  mcp.pinMode(5, OUTPUT);
  mcp.pinMode(6, OUTPUT);
  mcp.pinMode(7, OUTPUT);
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

void tank_operate()
{
  relay_reset();
}

void tank_automaintenance()
{

}

void stop_timer()
{
  if (afterState1 != -1) {
    timer1.stop(afterState1);
    afterState1 = -1;
  }
  if (afterState2 != -1) {
    timer2.stop(afterState2);
    afterState2 = -1;
  }
  if (afterState3 != -1) {
    timer3.stop(afterState3);
    afterState3 = -1;
  }
}

void tank1_state2()
{
  display.showNumberDecEx(123, (0x80 >> 1), false, 3, 1);
  
  Serial.println("Cleaning start...");
  Serial.println("State 2 close valve 1,5; open valve 2,3");
  relay_gpio(0b00001001);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b00001011);
  // 5 minute clean
  // delay(CLEANDELAY1);
  afterState2 = timer2.after(CLEANDELAY1, tank1_state3);
  if (afterState1 != -1) {
    timer1.stop(afterState1);
    afterState1 = -1;
  }
}

void tank1_state3()
{
  display.showNumberDecEx(114, (0x80 >> 1), false, 3, 1);
  
  Serial.println("State 3 close valve 2,3,5; open valve 1,4");
  relay_gpio(0b00001000);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b00001100);
  // 1 minute clean
  // delay(CLEANDELAY2);
  afterState3 = timer3.after(CLEANDELAY2, tank_state4);
  if (afterState2 != -1) {
    timer2.stop(afterState2);
    afterState2 = -1;
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
void tank_state4()
{
  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00 };
  
  data[0] = 0x79;
  data[1] = 0x54;
  data[2] = 0x5E;
  data[3] = 0x00;
  display.setSegments(data);
        
  Serial.println("State 4 cleaning ended.");
  Serial.println();
  relay_reset();
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  led_tank1.off();
  led_tank2.off();
  led_tank3.off();
  led_tank4.off();
  
  if (afterState3 != -1) {
    timer3.stop(afterState3);
    afterState3 = -1;
  }
  working = 0;
  if (automode) {
    if (operation == 1) {
      tank2_clean();
      operation = 2;
    }
    else if (operation == 2) {
      setup_relayboard(1);
      tank3_clean();
      operation = 3;
    }
    else if (operation == 3) {
      tank4_clean();
      operation = 0;
      automode = false;
    }
  }
  
}


void tank2_state2()
{
  display.showNumberDecEx(223, (0x80 >> 1), false, 3, 1);
  
  Serial.println("Cleaning start...");
  Serial.println("State 2 close valve 1,5; open valve 2,3");
  relay_gpio(0b10010000);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b10110000);
  // delay(CLEANDELAY1);
  afterState2 = timer2.after(CLEANDELAY1, tank2_state3);
  if (afterState1 != -1) {
    timer1.stop(afterState1);
    afterState1 = -1;
  }
}

void tank2_state3()
{
  display.showNumberDecEx(214, (0x80 >> 1), false, 3, 1);
  
  Serial.println("State 3 close valve 2,3; open valve 1,4");
  relay_gpio(0b10000000);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b11000000);
  // 1 minute clean
  // delay(CLEANDELAY2);
  afterState3 = timer3.after(CLEANDELAY2, tank_state4);
  if (afterState2 != -1) {
    timer2.stop(afterState2);
    afterState2 = -1;
  }
}

void tank3_state2()
{
  display.showNumberDecEx(323, (0x80 >> 1), false, 3, 1);
  
  Serial.println("Cleaning start...");
  Serial.println("State 2 close valve 1,5; open valve 2,3");
  relay_gpio(0b00001001);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b00001011);
  // 5 minute clean
  // delay(CLEANDELAY1);
  afterState2 = timer2.after(CLEANDELAY1, tank3_state3);
  if (afterState1 != -1) {
    timer1.stop(afterState1);
    afterState1 = -1;
  }
}

void tank3_state3()
{
  display.showNumberDecEx(314, (0x80 >> 1), false, 3, 1);
  
  Serial.println("State 3 close valve 2,3,5; open valve 1,4");
  relay_gpio(0b00001000);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b00001100);
  // 1 minute clean
  // delay(CLEANDELAY2);
  afterState3 = timer3.after(CLEANDELAY2, tank_state4);
  if (afterState2 != -1) {
    timer2.stop(afterState2);
    afterState2 = -1;
  }
}

void tank4_state2()
{
  display.showNumberDecEx(423, (0x80 >> 1), false, 3, 1);
  
  Serial.println("Cleaning start...");
  Serial.println("State 2 close valve 1,5; open valve 2,3");
  relay_gpio(0b10010000);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b10110000);
  // delay(CLEANDELAY1);
  afterState2 = timer2.after(CLEANDELAY1, tank4_state3);
  if(afterState1 != -1) {
    timer1.stop(afterState1);
    afterState1 = -1;
  }
}

void tank4_state3()
{
  display.showNumberDecEx(414, (0x80 >> 1), false, 3, 1);
  
  Serial.println("State 3 close valve 2,3; open valve 1,4");
  relay_gpio(0b10000000);
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  relay_gpio(0b11000000);
  // 1 minute clean
  // delay(CLEANDELAY2);
  afterState3 = timer3.after(CLEANDELAY2, tank_state4);
  if(afterState2 != -1) {
    timer2.stop(afterState2);
    afterState2 = 1;
  }
}

void tank1_clean()
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(115, (0x80 >> 1), false, 3, 1);
  
  Serial.println("Reset");
  relay_reset();
  
  Serial.println("Tank 1");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b00001001);
  working = 1;
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  afterState1 = timer1.after(CLEANDELAY0, tank1_state2);
  
  led_tank1.on();
  led_tank2.off();
  led_tank3.off();
  led_tank4.off();

}


void tank2_clean()
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(215, (0x80 >> 1), false, 3, 1);

  Serial.println("Reset");
  relay_reset();
  
  Serial.println("Tank 2");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b10010000);
  working = 1;
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  afterState1 = timer1.after(CLEANDELAY0, tank2_state2);
  
  led_tank1.off();
  led_tank2.on();
  led_tank3.off();
  led_tank4.off();

}

void tank3_clean()
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(315, (0x80 >> 1), false, 3, 1);

  Serial.println("Reset");
  relay_reset();
  
  Serial.println("Tank 3");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b00001001);
  working = 1;
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  afterState1 = timer1.after(CLEANDELAY0, tank3_state2);
  
  led_tank1.off();
  led_tank2.off();
  led_tank3.on();
  led_tank4.off();
}

void tank4_clean()
{
  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(415, (0x80 >> 1), false, 3, 1);
  
  Serial.println("Reset");
  relay_reset();
  
  Serial.println("Tank 4");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b10010000);
  working = 1;
  delay(DELAYSTATE);
  if (wifi_connected == 0) {
    Blynk.run();  
  }
  afterState1 = timer1.after(CLEANDELAY0, tank4_state2);
  
  led_tank1.off();
  led_tank2.off();
  led_tank3.off();
  led_tank4.on();

}


void relay_status()
{
  for (int i = 0; i < 8; i++) {
    Serial.print(i+1);
    Serial.print(mcp.digitalRead(i) ? ": ON  " : ": OFF ");
  }
  Serial.println();
}

void relay_gpio(uint8_t gpio)
{
  mcp.writeGPIO(gpio);
  for (int i = 0; i < 8; i++) {
    Serial.print(i+1);
    Serial.print(mcp.digitalRead(i) ? ": ON  " : ": OFF ");
  }
  Serial.println();

}

void relay_onoff(char port, uint8_t of)
{
  int i;

  if ((port > 0) && (port < 9)) {
    mcp.digitalWrite(port-1, of);

  }
  for (i = 0; i < 8; i++) {
    Serial.print(i+1);
    Serial.print(mcp.digitalRead(i) ? ": ON  " : ": OFF ");
  }
  Serial.println();
}

void relay_reset()
{
  int i;

  for (i = 0; i < 8; i++) {
    mcp.digitalWrite(i, LOW);
  }
  delay(500);
  for (i = 0; i < 8; i++) {
    Serial.print(i+1);
    Serial.print(mcp.digitalRead(i) ? ": ON  " : ": OFF ");
  }
  Serial.println();
}


void relay_control(char input)
{
  int i;
  static uint8_t olat;

  if ((input > '0') && (input < '9'))
  {
    Serial.print("Toggle ");
    Serial.println(input);
    olat ^= 1 << ((input - 1) & 15);
    mcp.writeGPIO(olat);
  }
  if (input == 'c')
  {
    Serial.println("Clear");
    olat = 0;
    mcp.writeGPIO(olat);
  }
  for (i = 0; i < 8; i++)
  {
    Serial.print(i+1);
    Serial.print((olat & (1 << i) ? ": ON  " : ": OFF "));
  }
  Serial.println();
}

void relay_serial_control()
{
  char input;
  int i;
  static uint8_t olat;

  if (Serial.available() > 0)
  {
    input = Serial.read();
    if ((input > '0') && (input < '9'))
    {
      Serial.print("Toggle ");
      Serial.println(input);
      olat ^= 1 << ((input - 1) & 15);
      mcp.writeGPIO(olat);
    }
    if (input == 'c')
    {
      Serial.println("Clear");
      olat = 0;
      mcp.writeGPIO(olat);
    }
    for (i = 0; i < 8; i++)
    {
      Serial.print(i+1);
      Serial.print((olat & (1 << i) ? ": ON  " : ": OFF "));
    }
    Serial.println();
  }

}

BLYNK_WRITE(V1)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 1 just pressed");
    automode = false;
    
    setup_relayboard(0);
    if (working == 0) {
      tank1_clean();
    }
  }
  
}

BLYNK_WRITE(V2)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 2 just pressed");
    automode = false;
    
    setup_relayboard(0);
    if (working == 0) {
      tank2_clean();
    }
  }
}

BLYNK_WRITE(V3)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 3 just pressed");
    automode = false;
    
    setup_relayboard(1);
    if(working == 0) {
      tank3_clean();
    }
  }
}

BLYNK_WRITE(V4)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    Serial.println("switch 4 just pressed");
    automode = false;
    
    setup_relayboard(1);
    if (working == 0) {
      tank4_clean();
    }
  }
}

BLYNK_WRITE(V10)
{
  int pinValue = param.asInt();

  Serial.println(pinValue);
  if (pinValue == 1) {
    // automatic clean
    Serial.println("switch 5 just pressed");
    automode = true;      
    if (working == 0) {
      setup_relayboard(0);
      operation = 1;
      tank1_clean();
    }
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
