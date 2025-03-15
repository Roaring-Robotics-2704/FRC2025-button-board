/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2021 NeKuNeKo for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include "Adafruit_seesaw.h"



// ----- LED Arcade 1x4 STEMMA QT board pins-----
// pin definitions on each LED Arcade 1x4
#define  SWITCH1  18  // PA01
#define  SWITCH2  19 // PA02
#define  SWITCH3  20 // PA03
#define  SWITCH4  2 // PA04
#define  PWM1  12  // PC00
#define  PWM2  13 // PC01
#define  PWM3  0 // PA04
#define  PWM4  1 // PA05

#define  I2C_BASE_ADDR 0x3A //  boards are in order, 0x3A, 0x3B, 0x3C, 0x3D
#define  NUM_BOARDS 4

Adafruit_seesaw ledArcades[ NUM_BOARDS ];

//----- board variables
int boardNum = 0;  //used to read each board
int switchNum = 0; //used to read each switch
int boardSwitchNum = 0; //unique button ID accross all boards/buttons
int led_low = 10;  //min pwm brightness
int led_med = 60; 
int led_high = 220; // max brightness

bool lastButtonState[16] ;
bool currentButtonState[16] ;

/* This sketch demonstrates USB HID gamepad use.
 * This sketch is only valid on boards which have native USB support
 * and compatibility with Adafruit TinyUSB library. 
 * For example SAMD21, SAMD51, nRF52840.
 * 
 * Make sure you select the TinyUSB USB stack if you have a SAMD board.
 * You can test the gamepad on a Windows system by pressing WIN+R, writing Joy.cpl and pressing Enter.
 */

// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};
// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload defined in src/class/hid/hid.h
// - For Gamepad Button Bit Mask see  hid_gamepad_button_bm_t
// - For Gamepad Hat    Bit Mask see  hid_gamepad_hat_t
hid_gamepad_report_t gp;

void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  // Setup HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
  
      //----- LED Arcade 1x4 setup-----
    //
    for ( int i = 0; i < NUM_BOARDS; i++ ) {
      if ( !ledArcades[i].begin( I2C_BASE_ADDR + i ) ) {
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
    // brighten default root note
    //ledArcades[0].analogWrite(PWM1, led_high);
    //----- end LED Arcade 1x4 setup-----
}

void loop() {

  for ( int i = 0; i < NUM_BOARDS; i++ ) {

      ledArcades[i].analogWrite(PWM1, led_high);
      ledArcades[i].analogWrite(PWM2, led_high);
      ledArcades[i].analogWrite(PWM3, led_high);
      ledArcades[i].analogWrite(PWM4, led_high);  
    }
  
  delay(10); // delay in loop to slow serial output
  //buttonCheck();

#if defined(IRQ_PIN)
  if(!digitalRead(IRQ_PIN)) {
    return;
  }
#endif



  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }

  // Remote wakeup
  if ( TinyUSBDevice.suspended())
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    TinyUSBDevice.remoteWakeup();
  }

  if (!usb_hid.ready()) return;

  // Reset buttons
  Serial.println("No pressing buttons");
  gp.buttons = 0;
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(2000);


  // Test buttons (up to 32 buttons)
  for (int i = 0; i < 32; ++i) {
    Serial.print("Pressing button ");
    Serial.println(i);
    gp.buttons = (1U << i);
    usb_hid.sendReport(0, &gp, sizeof(gp));
    delay(1000);
  }
  // */
}
