#include <Wire.h>
#include "Adafruit_MCP23008.h"

#include <Arduino.h>
#include <TM1637Display.h>

#include "Timer.h"

Timer timer1, timer2, timer3;
int afterState1 = -1;
int afterState2 = -1;
int afterState3 = -1;
int working = 0;

// UNO 7 segments display
#define CLK 9
#define DIO 8


TM1637Display display(CLK, DIO);

// I2C Relayboard test
// connect VDD to power 5V
// connect GND to power GND
// connect SDA to analog 4 (I2C DATA)
// connect SCL to analog 5 (I2C CLOCK)

Adafruit_MCP23008 mcp;

int buttonPin = 2;         // the number of the input pin
int stopPin = 3;
int ledPin = 13;       // the number of the output pin

#define DEBOUNCE 10  // button debouncer, how many ms to debounce, 5+ ms is usually plenty
byte buttons[] = {2, 3, 4, 5, 6};

// This handy macro lets us determine how big the array up above is, by checking the size
#define NUMBUTTONS sizeof(buttons)

// we will track if a button is just pressed, just released, or 'currently pressed'
byte pressed[NUMBUTTONS], justpressed[NUMBUTTONS], justreleased[NUMBUTTONS];
byte previous_keystate[NUMBUTTONS], current_keystate[NUMBUTTONS];

// 5 minutes = 300000 ms
// 1 minute = 60000 ms
int CLEANDELAY1 = 10000;
int CLEANDELAY2 = 5000;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 1000;           // interval at which to blink (milliseconds)
int ledState = LOW;             // ledState used to set the LED

void setup()
{
  byte i;

  // start serial port at 9600 bps and wait for port to open:
  Serial.begin(9600);
  uint8_t data[] = { 0x00, 0x00, 0x00, 0x00 };
  display.setSegments(data);
  display.setBrightness(0x0a);

  data[0] = 0x06;
  data[1] = 0x54;
  data[2] = 0x06;
  data[3] = 0x78;

  display.setSegments(data);

  pinMode(ledPin, OUTPUT);

  Serial.print("Button checker with ");
  Serial.print(NUMBUTTONS, DEC);
  Serial.println(" buttons");

  // Make input & enable pull-up resistors on switch pins
  for (i=0; i < NUMBUTTONS; i++) {
    pinMode(buttons[i], INPUT);
    digitalWrite(buttons[i], HIGH);
  }

  // mcp.begin(1); // address = 0 (valid: 0-7)
  setup_relayboard(0);



  while (!Serial); // wait for serial port to connect. Needed for Leonardo only
  Serial.println("I2C Relayboard test - press keys 12345678 (toggle relay) C (clear all)");
  relay_reset();
  delay(2000);
}

void loop()
{
  int i;
  uint8_t data[] = { 0x3f, 0x73, 0x79, 0x50 };

  timer1.update();
  timer2.update();
  timer3.update();

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) {
      data[0] = 0x3f;
      data[1] = 0x73;
      data[2] = 0x79;
      data[3] = 0x50;
      display.setSegments(data);
      ledState = HIGH;
    } else {
      data[0] = 0x00;
      data[1] = 0x00;
      data[2] = 0x00;
      data[3] = 0x00;
      display.setSegments(data);
      ledState = LOW;
    }

    // set the LED with the ledState of the variable:
    // digitalWrite(ledPin, ledState);
  }



  byte thisSwitch=thisSwitch_justPressed();
  switch(thisSwitch)
  {
    case 0:
      Serial.println("switch 1 just pressed");
      setup_relayboard(0);
      if (working == 0) {
        tank1_clean();
      }
      break;
    case 1:
      Serial.println("switch 2 just pressed");
      setup_relayboard(0);
      if (working == 0) {
        tank2_clean();
      }
      break;
    case 2:
      Serial.println("switch 3 just pressed");
      setup_relayboard(1);
      if(working == 0) {
        tank3_clean();
      }
      break;
    case 3:
      Serial.println("switch 4 just pressed");
      setup_relayboard(1);
      if (working == 0) {
        tank4_clean();
      }
      break;
    case 4:
      Serial.println("switch 5 just pressed");
      // automatic clean
      if (working == 0) {
        setup_relayboard(0);
        tank1_clean();
        tank2_clean();
        setup_relayboard(1);
        tank3_clean();
        tank4_clean();
      }
      break;
    case 5:
      Serial.println("switch 6 just pressed"); break;
  }

  relay_serial_control();


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
  relay_gpio(0b00001100);
  // 1 minute clean
  // delay(CLEANDELAY2);
  afterState3 = timer3.after(CLEANDELAY2, tank_state4);
  if (afterState2 != -1) {
    timer2.stop(afterState2);
    afterState2 = -1;
  }

}

void tank_state4()
{
  Serial.println("State 4 cleaning ended.");
  Serial.println();
  relay_reset();
  delay(500);
  if (afterState3 != -1) {
    timer3.stop(afterState3);
    afterState3 = -1;
  }
  working = 0;
}


void tank2_state2()
{
  display.showNumberDecEx(223, (0x80 >> 1), false, 3, 1);
  Serial.println("Cleaning start...");
  Serial.println("State 2 close valve 1,5; open valve 2,3");
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
  Serial.println("Reset");
  relay_reset();
  delay(500);

  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);

  display.showNumberDecEx(115, (0x80 >> 1), false, 3, 1);
  Serial.println("Tank 1");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b00001001);
  working = 1;
  // delay(CLEANDELAY1);
  afterState1 = timer1.after(CLEANDELAY1, tank1_state2);

}


void tank2_clean()
{
  Serial.println("Reset");
  relay_reset();
  delay(500);

  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(215, (0x80 >> 1), false, 3, 1);
  Serial.println("Tank 2");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b10010000);
  working = 1;
  // delay(CLEANDELAY1);
  afterState1 = timer1.after(CLEANDELAY1, tank2_state2);

}

void tank3_clean()
{
  Serial.println("Reset");
  relay_reset();
  delay(500);

  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(315, (0x80 >> 1), false, 3, 1);
  Serial.println("Tank 3");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b00001001);
  working = 1;
  // delay(CLEANDELAY1);
  afterState1 = timer1.after(CLEANDELAY1, tank3_state2);
}

void tank4_clean()
{
  Serial.println("Reset");
  relay_reset();
  delay(500);

  uint8_t data[] = { 0x78, 0x00, 0x00, 0x00 }; // t
  display.setSegments(data);
  display.showNumberDecEx(415, (0x80 >> 1), false, 3, 1);
  Serial.println("Tank 4");
  Serial.println("State 1 close valve 1, 5");
  relay_gpio(0b10010000);
  working = 1;
  // delay(CLEANDELAY1);
  afterState1 = timer1.after(CLEANDELAY1, tank4_state2);

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
