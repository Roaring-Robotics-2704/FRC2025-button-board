/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include <Adafruit_TinyUSB.h>

#define  DEFAULT_I2C_ADDR 0x3A

#define  SWITCH1  18  // PA01
#define  SWITCH2  19 // PA02
#define  SWITCH3  20 // PA03
#define  SWITCH4  2 // PA06
#define  PWM1  12  // PC00
#define  PWM2  13 // PC01
#define  PWM3  0 // PA04
#define  PWM4  1 // PA05

#define NUM_BOARDS 4
int led_low = 0;  //min pwm brightness
int led_med = 60; 
int led_high = 220; // max brightness

Adafruit_seesaw ledArcades[NUM_BOARDS] = { Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire) };
int switches[4] = {SWITCH1, SWITCH2, SWITCH3, SWITCH4};
int lights[4] = {PWM1, PWM2, PWM3, PWM4};

void setup() {

  Serial.begin(115200);
  Wire.setSDA(0);
  Wire.setSCL(1);
  
  while (!Serial) delay(10);   // wait until serial port is opened

  Serial.println(F("Iterating through all I2c ports"));

  for ( int i = 0; i < NUM_BOARDS; i++ ) {
      if ( !ledArcades[i].begin( DEFAULT_I2C_ADDR + i ) ) {
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
      ledArcades[i].analogWrite(PWM1, 127);
      ledArcades[i].analogWrite(PWM2, 127);
      ledArcades[i].analogWrite(PWM3, 127);
      ledArcades[i].analogWrite(PWM4, 127);  
    }

  // uint16_t pid;
  // uint8_t year, mon, day;
  
  // ss.getProdDatecode(&pid, &year, &mon, &day);
  // Serial.print("seesaw found PID: ");
  // Serial.print(pid);
  // Serial.print(" datecode: ");
  // Serial.print(2000+year); Serial.print("/"); 
  // Serial.print(mon); Serial.print("/"); 
  // Serial.println(day);

  // if (pid != 5296) {
  //   Serial.println(F("Wrong seesaw PID"));
  //   while (1) delay(10);
  // }

  Serial.println(F("seesaw started OK!"));
}

uint8_t incr = 0;

void loop() {
  for ( int i = 0; i < NUM_BOARDS; i++ ) {
      for ( int j = 0; j< 4; j++) {
        if (!ledArcades[i].digitalRead(switches[j])) {
          ledArcades[i].analogWrite(lights[j], led_high);
          Serial.println("button " + (String)j + " on board " + (String)i + "Pressed");
        } else {
          ledArcades[i].analogWrite(lights[j],led_low);
        }
      }
  }
  delay(10);
}
