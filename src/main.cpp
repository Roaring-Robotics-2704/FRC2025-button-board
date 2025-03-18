/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include <Adafruit_TinyUSB.h>

#define DEFAULT_I2C_ADDR 0x3A

#define SWITCH1 18 // PA01
#define SWITCH2 19 // PA02
#define SWITCH3 20 // PA03
#define SWITCH4 2  // PA06
#define PWM1 12    // PC00
#define PWM2 13    // PC01
#define PWM3 0     // PA04
#define PWM4 1     // PA05

#define NUM_BOARDS 4
int led_low = 0; // min pwm brightness
int led_med = 60;
int led_high = 220; // max brightness



Adafruit_seesaw ledArcades[NUM_BOARDS] = {Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire), Adafruit_seesaw(&Wire)};
int switches[4] = {SWITCH1, SWITCH2, SWITCH3, SWITCH4};
int lights[4] = {PWM1, PWM2, PWM3, PWM4};


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



int heightOrder[4] = {0,2,1,3};

bool heightSelector[2][4] = {{true, false, false, false}, {false, false, false, false}};
bool buttons[8] = {false, false, false, false, false, false, false, false};

void clearHeightSelector() {
  for (int x=0; x<2; x++) {
    for (int y=0; y<4; y++)
      heightSelector[x][y] = false;
  };
}
void updateHeightSelectorLights() {
  for (int x=0; x<2; x++) {
    for (int y=0; y<4; y++) {
      if (heightSelector[x][heightOrder[y]] == true) {
              ledArcades[y].analogWrite(lights[x],led_high);
      } else {
        ledArcades[y].analogWrite(lights[x],led_low);
      }
    }
  };
}

int getHeightSelector() {
  int i = 0;
  for (int x=0; x<2; x++) {
    for (int y=0; y<4; y++) {
      i++;
      if (heightSelector[x][y]) return i;
        
    }
  };
  return 0;
}
int getButtons() {

  //int i = 0;
  //i |= digitalRead(2)<<8;// it will then gamepad button 7
  //i |= digitalRead(10);    //  button 0
  uint16_t i;
  i |= (buttons[0] ? HIGH : LOW) << 0;
  i |= (buttons[0] ? HIGH : LOW) << 1;
  i |= (buttons[1] ? HIGH : LOW) << 2;
  i |= (buttons[1] ? HIGH : LOW) << 3;
  i |= (buttons[2] ? HIGH : LOW) << 4;
  i |= (buttons[2] ? HIGH : LOW) << 5;
  i |= (buttons[3] ? HIGH : LOW) << 6;
  i |= (buttons[3] ? HIGH : LOW) << 7;
  i |= (buttons[4] ? HIGH : LOW) << 8;
  i |= (buttons[4] ? HIGH : LOW) << 9;
  i |= (buttons[5] ? HIGH : LOW) << 10;
  i |= (buttons[5] ? HIGH : LOW) << 11;
  i |= (buttons[6] ? HIGH : LOW) << 12;
  i |= (buttons[6] ? HIGH : LOW) << 13;
  i |= (buttons[7] ? HIGH : LOW) << 14;
  i |= (buttons[7] ? HIGH : LOW) << 15;

  // for (int x=0; x<8; x++) {
  //      i |=  (buttons[x] ? HIGH: LOW) << x;
  //  };
  // for (int w=0; w<(32-8); w++) {
  //   i |= LOW << (w+8);
  // }
  return i;
}

void setup()
{
  Wire.setSDA(0);
  Wire.setSCL(1);

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


  for (int i = 0; i < NUM_BOARDS; i++)
  {
    if (!ledArcades[i].begin(DEFAULT_I2C_ADDR + i))
    {
      while (1)
        delay(10);
    }
  }

  for (int i = 0; i < NUM_BOARDS; i++)
  {
    ledArcades[i].pinMode(SWITCH1, INPUT_PULLUP);
    ledArcades[i].pinMode(SWITCH2, INPUT_PULLUP);
    ledArcades[i].pinMode(SWITCH3, INPUT_PULLUP);
    ledArcades[i].pinMode(SWITCH4, INPUT_PULLUP);
    ledArcades[i].analogWrite(PWM1, 127);
    ledArcades[i].analogWrite(PWM2, 127);
    ledArcades[i].analogWrite(PWM3, 127);
    ledArcades[i].analogWrite(PWM4, 127);
  }


}

uint8_t incr = 0;

void loop()
{


  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  // not enumerated()/mounted() yet: nothing to do
  if (!TinyUSBDevice.mounted()) {
    return;
  }

//  // Remote wakeup
//  if ( TinyUSBDevice.suspended() && btn )
//  {
//    // Wake up host if we are in suspend mode
//    // and REMOTE_WAKEUP feature is enabled by host
//    TinyUSBDevice.remoteWakeup();
//  }

  if (!usb_hid.ready()) return;
  for (int i = 0; i < NUM_BOARDS; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      if (j == 1 || j == 0)
      {
        if (!ledArcades[i].digitalRead(switches[j])){
          clearHeightSelector();
          heightSelector[j][heightOrder[i]] = true;
        }
      }
      else
      {
        int x = i*j;
        if (!ledArcades[i].digitalRead(switches[j]))
        {
          ledArcades[i].analogWrite(lights[j], led_high);
          buttons[x] = true;
        }
        else
        {
          ledArcades[i].analogWrite(lights[j], led_low);
          buttons[x] = false;

        }
      }
    }
  }
  updateHeightSelectorLights();
  gp.hat = getHeightSelector();
  // for (int i = 0; i < 32; ++i) {
  //   Serial.print("Pressing button ");
  //   Serial.println(i);
  //   gp.buttons = (1U << i);
  //   usb_hid.sendReport(0, &gp, sizeof(gp));
  //   delay(1000);
  // }
  gp.buttons = getButtons();
  usb_hid.sendReport(0, &gp, sizeof(gp));
  delay(10);
}



