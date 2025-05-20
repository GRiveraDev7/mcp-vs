// *****************
// Example ATmega2560 Project
// File: ATmega2560Project.c
// An example file for second year mechatronics project
// *****************

#include "Controller.h"  // Include this .c file's header

// Constants
#define ADC_MAX_VALUE     253
#define SEND_INTERVAL_MS  100

int main(void)
{
  // Initialization
  serial2_init();
  serial0_init();
  milliseconds_init();
  adc_init();
  lcd_init();
  _delay_ms(20);

  uint8_t vertical1 = 0;
  uint8_t vertical2 = 0;
  uint8_t horizontal1 = 0;
  uint8_t horizontal2 = 0;
  uint8_t receivedDataRange[3]; // Received data array
  int current_ms = 0;
  int last_send_ms = 0;
  char received_string[100] = {};

  while (1)
  {
    current_ms = milliseconds_now();

    // Sending section
    if ((current_ms - last_send_ms) >= SEND_INTERVAL_MS)
    {
      // Joystick 1
      vertical1   = (adc_read(15) >> 2);
      horizontal1 = (adc_read(14) >> 2);

      if (vertical1 > ADC_MAX_VALUE) {
        vertical1 = ADC_MAX_VALUE;
      }
      if (horizontal1 > ADC_MAX_VALUE) {
        horizontal1 = ADC_MAX_VALUE;
      }

      // Joystick 2
      vertical2   = (adc_read(0) >> 2);
      horizontal2 = (adc_read(1) >> 2);

      if (vertical2 > ADC_MAX_VALUE) {
        vertical2 = ADC_MAX_VALUE;
      }
      if (horizontal2 > ADC_MAX_VALUE) {
        horizontal2 = ADC_MAX_VALUE;
      }

      // Function takes number of bytes followed by data bytes
      serial2_write_bytes(4, vertical1, horizontal1, vertical2, horizontal2);
      last_send_ms = current_ms;
    }

    // Receiving section: battery levels, range sensor readings in cm,
    // autonomous/manual, light level/frequency, low battery voltage alert
    if (serial2_available())
    {
      serial2_get_data(receivedDataRange, 3); // Fill array with 3 bytes
      lcd_clrscr();

      lcd_home();  // Same as lcd_goto(0)
      sprintf(received_string, "L:%ucm R:%ucm", receivedDataRange[0], receivedDataRange[2]);
      lcd_puts(received_string);  // First line

      lcd_goto(0x40);  // Cursor to second line
      sprintf(received_string, "F:%ucm", receivedDataRange[1]);
      lcd_puts(received_string);
    }
  }

  return 1;
}
