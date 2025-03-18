/*
 * This example shows how read arcade buttons and PWM the LEDs on the Adafruit Arcade QT!
 */

#include "Adafruit_seesaw.h"
#include <seesaw_neopixel.h>
#include <Adafruit_TinyUSB.h>

#define TOGGLE_HEIGHT_SELECTOR false

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
    TUD_HID_REPORT_DESC_GAMEPAD()};

// USB HID object
Adafruit_USBD_HID usb_hid;

// Report payload defined in src/class/hid/hid.h
// - For Gamepad Button Bit Mask see  hid_gamepad_button_bm_t
// - For Gamepad Hat    Bit Mask see  hid_gamepad_hat_t
hid_gamepad_report_t gp;

int heightOrder[4] = {0, 2, 1, 3};

bool heightSelector[2][4] = {{true, false, false, false}, {false, false, false, false}};
bool buttons[4][4] = {{false, false, false, false}, {false, false, false, false}, {false, false, false, false}, {false, false, false, false}};

void clearHeightSelector()
{
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 4; y++)
      heightSelector[x][y] = false;
  };
}
void updateHeightSelectorLights()
{
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 4; y++)
    {
      if (heightSelector[x][heightOrder[y]] == true)
      {
        ledArcades[y].analogWrite(lights[x], led_high);
      }
      else
      {
        ledArcades[y].analogWrite(lights[x], led_low);
      }
    }
  };
}

int getHeightSelector()
{
  int i = 0;
  for (int x = 0; x < 2; x++)
  {
    for (int y = 0; y < 4; y++)
    {
      i++;
      if (heightSelector[x][y])
        return i;
    }
  };
  return 0;
}
int getButtons()
{

  uint16_t i;

  i = 0;
  for (int x = 0; x < 4; x++)
  {
    for (int y = 0; y < 4; y++)
    {
      if (buttons[x][y])
      {                            // Swap x and y to fix the order
        i |= (1 << ((x * 2) + y)); // Adjust bit position accordingly
      }
    }
  }

  return i;
}

void setup()
{
  Wire.setSDA(0);
  Wire.setSCL(1);

  if (!TinyUSBDevice.isInitialized())
  {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  // Setup HID
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted())
  {
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
  if (!TinyUSBDevice.mounted())
  {
    return;
  }

  //  // Remote wakeup
  //  if ( TinyUSBDevice.suspended() && btn )
  //  {
  //    // Wake up host if we are in suspend mode
  //    // and REMOTE_WAKEUP feature is enabled by host
  //    TinyUSBDevice.remoteWakeup();
  //  }

  if (!usb_hid.ready())
    return;
  for (int i = 0; i < NUM_BOARDS; i++)
  {
    for (int j = 0; j < 4; j++)
    { // Iterate over 4 buttons per board
      if (j == 0 || j == 1)
      { // Height selectors
        if (!ledArcades[i].digitalRead(switches[j]))
        {
          clearHeightSelector();
          heightSelector[j][heightOrder[i]] = true;
        }
      }
      else
      {                                 // Regular buttons
        int buttonRow = heightOrder[i]; // Map board index to row
        int buttonCol = j - 2;          // Map button index to column (j = 2, 3 -> col = 0, 1)
        if (!ledArcades[i].digitalRead(switches[j]))
        {
          ledArcades[i].analogWrite(lights[j], led_high);
          buttons[buttonRow][buttonCol] = true; // Correctly map to 2x4 array
        }
        else
        {
          ledArcades[i].analogWrite(lights[j], led_low);
          buttons[buttonRow][buttonCol] = false; // Correctly map to 2x4 array
        }
      }
    }
  }

  // Update height selector lights after processing all boards
  updateHeightSelectorLights();

  // Update HID report after processing all buttons
  gp.hat = getHeightSelector();
  gp.buttons = getButtons();
  usb_hid.sendReport(0, &gp, sizeof(gp));

  if (!TOGGLE_HEIGHT_SELECTOR) {
  // Clear height selector for the next loop iteration
  clearHeightSelector();
  }

  delay(10);
}
