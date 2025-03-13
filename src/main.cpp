#include <Arduino.h>
#include <PicoGamepad.h>


PicoGamepad gamepad;

// 16 bit integer for holding input values
int val;

void setup() {  
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);

  // X Potentiometer on pin 26
  pinMode(26, INPUT);
  // Y Potentiometer on pin 27
  pinMode(27, INPUT);

  // Button on pin 
  pinMode(28, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
}
