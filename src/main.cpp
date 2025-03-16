/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>

#define  I2C_ADDR 0x3A

#define  SWITCH1  18  // PA01
#define  SWITCH2  19 // PA02
#define  SWITCH3  20 // PA03
#define  SWITCH4  2 // PA06
#define  PWM1  12  // PC00
#define  PWM2  13 // PC01
#define  PWM3  0 // PA04
#define  PWM4  1 // PA05

#define  NUM_BOARDS 4
Adafruit_seesaw ledArcades[NUM_BOARDS] = { Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire) };

void setup() {

  Serial.begin(115200);
  Wire.setSDA(0);
  Wire.setSCL(1);
  
for ( int i = 0; i < NUM_BOARDS; i++ ) {
      if ( !ledArcades[i].begin( I2C_ADDR + i ) ) {
      Serial.println(F("LED Arcade not found!"));
      while (1) delay(10);
      } 
    }
    Serial.println(F("LED Arcade boards started"));
  
    for ( int i = 0; i < NUM_BOARDS; i++ ) {
      ledArcades[i].pinMode(SWITCH1, INPUT_PULLUP);
      ledArcades[i].pinMode(SWITCH2, INPUT_PULLUP);
      ledArcades[i].pinMode(SWITCH3, INPUT_PULLUP);
      ledArcades[i].pinMode(SWITCH4, INPUT_PULLUP);
      ledArcades[i].analogWrite(PWM1, led_low);
      ledArcades[i].analogWrite(PWM2, led_low);
      ledArcades[i].analogWrite(PWM3, led_low);
      ledArcades[i].analogWrite(PWM4, led_low);  
    }



  Serial.println(F("seesaw started OK!"));
  ss.pinMode(SWITCH1, INPUT_PULLUP);
  ss.pinMode(SWITCH2, INPUT_PULLUP);
  ss.pinMode(SWITCH3, INPUT_PULLUP);
  ss.pinMode(SWITCH4, INPUT_PULLUP);
  ss.analogWrite(PWM1, 127);
  ss.analogWrite(PWM2, 127);
  ss.analogWrite(PWM3, 127);
  ss.analogWrite(PWM4, 127);
}

uint8_t incr = 0;

void loop() {
  if (! ss.digitalRead(SWITCH1)) {
    Serial.println("Switch 1 pressed");
    ss.analogWrite(PWM1, incr);
    incr += 5;
  } else {
    ss.analogWrite(PWM1, 0);
  }
  
  if (! ss.digitalRead(SWITCH2)) {
    Serial.println("Switch 2 pressed");
    ss.analogWrite(PWM2, incr);
    incr += 5;
  } else {
    ss.analogWrite(PWM2, 0);
  }
  
  if (! ss.digitalRead(SWITCH3)) {
    Serial.println("Switch 3 pressed");
    ss.analogWrite(PWM3, incr);
    incr += 5;
  } else {
    ss.analogWrite(PWM3, 0);
  }
  
  if (! ss.digitalRead(SWITCH4)) {
    Serial.println("Switch 4 pressed");
    ss.analogWrite(PWM4, incr);
    incr += 5;
  } else {
    ss.analogWrite(PWM4, 0);
  }
  delay(10);
}
