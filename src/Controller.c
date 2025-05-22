#include "Controller.h"  // Include this .c file's header

#define ADC_MAX_VALUE     253
#define SEND_INTERVAL_MS  100
#define INT_INTERVAL_MS  1000

uint8_t mode = 0;
int current_ms = 0;

char serial_string[120] = {0};

int main(void)
{
  // Initialization
  serial2_init();
  serial0_init();
  milliseconds_init();
  adc_init();
  lcd_init();
  _delay_ms(20);

  // INT0 (PD0) and INT1 (PD1) input + pull-up
  DDRD &= ~((1 << PD0) | (1 << PD1));     // Set as input
  PORTD |= (1 << PD0) | (1 << PD1);       // Enable pull-up resistors

  // Configure interrupts
  // INT0 -> Falling edge: ISC01 = 1, ISC00 = 0
  EICRA |= (1 << ISC01);
  EICRA &= ~(1 << ISC00);

  // INT1 -> Falling edge: ISC11 = 1, ISC10 = 0
  EICRA |= (1 << ISC11);
  EICRA &= ~(1 << ISC10);

  // Enable INT0 and INT1
  EIMSK |= (1 << INT0) | (1 << INT1);

  sei();  // Enable global interrupts

  uint8_t vertical1 = 0, vertical2 = 0;
  uint8_t horizontal1 = 0, horizontal2 = 0;
  uint8_t receivedDataRange[4]; // Received data array
  int last_send_ms = 0;
  char received_string[100] = {};

  while (1)
  {
    current_ms = milliseconds_now();

    // Sending joystick data every interval
    if ((current_ms - last_send_ms) >= SEND_INTERVAL_MS)
    {
      vertical1   = (adc_read(15) >> 2);
      horizontal1 = (adc_read(14) >> 2);
      vertical2   = (adc_read(0) >> 2);
      horizontal2 = (adc_read(1) >> 2);

      // Clamp values
      if (vertical1 > ADC_MAX_VALUE) vertical1 = ADC_MAX_VALUE;
      if (horizontal1 > ADC_MAX_VALUE) horizontal1 = ADC_MAX_VALUE;
      if (vertical2 > ADC_MAX_VALUE) vertical2 = ADC_MAX_VALUE;
      if (horizontal2 > ADC_MAX_VALUE) horizontal2 = ADC_MAX_VALUE;

      serial2_write_bytes(4, vertical1, horizontal1, vertical2, mode);
      last_send_ms = current_ms;

      sprintf(serial_string, "\n Mode: %i", mode);
    serial0_print_string(serial_string);
    }

    // Receiving data to display on LCD
    if (serial2_available())
    {
      serial2_get_data(receivedDataRange, 4);
      lcd_clrscr();
      lcd_home();
      sprintf(received_string, "L:%ucm R:%ucm", receivedDataRange[0], receivedDataRange[1]);
      lcd_puts(received_string);

      lcd_goto(0x40);
      sprintf(received_string, "F:%ucm L:%u.%uHz", receivedDataRange[2], receivedDataRange[3]/10, receivedDataRange[3]%10);
      lcd_puts(received_string);
    }
  }

  return 1;
}

int last_int_ms = 0;

// INT0: Button 1 -> Auto Mode Toggle
ISR(INT0_vect)
{
  current_ms = milliseconds_now();
  if ((current_ms-last_int_ms) > INT_INTERVAL_MS){
      if(mode == 1){
        mode = 0;
      }else{
        mode = 1;
      }
      last_int_ms = current_ms;
    }
}

// INT1: Button 2 -> Light Mode Toggle
ISR(INT1_vect)
{
  current_ms = milliseconds_now();
  if ((current_ms-last_int_ms) > INT_INTERVAL_MS){
      if(mode == 2){
        mode = 0;
      }else{
        mode = 2;
      }
      last_int_ms = current_ms;
    }
}
