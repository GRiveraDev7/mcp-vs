// *****************
// Example ATmega2560 Project
// File: ATmega2560Project.c
// An example file for second year mechatronics project
// *****************

#include "Robot.h"

// Constants
#define MAX_SENSOR_SIDE      50
#define MAX_SENSOR_FRONT     100
#define SEND_INTERVAL_MS     100
#define SERVO_OFFSET         952
#define SERVO_SCALE          4
#define PWM_SCALE            8000 / 126
#define NO_LIGHT_VALUE       220

// PI Control Constants
#define KP                  20.0f
#define KI                  0.02f
#define KP_LIGHT            7.5f
#define KI_LIGHT            0.01f
#define TOO_CLOSE_FORWARD   15
#define TOO_CLOSE_SIDE      10

// Function Prototypes
void auto_drive(uint16_t sensor_forward, uint16_t sensor_left, uint16_t sensor_right);
void motor_drive(uint16_t speed, uint16_t direction);
void motor_init();
void servo(uint16_t control);
void follow_light_drive(uint16_t right_light_sensor, uint16_t left_light_sensor);

// Globals
int16_t error = 0;
int16_t integral = 0;
int16_t control = 0;
int16_t lm = 0;
int16_t rm = 0;

int main(void)
{
    // Initialization
    serial2_init();
    serial0_init();
    milliseconds_init();
    adc_init();
    motor_init();
    _delay_ms(10);

    uint8_t receivedData[4];
    char serial_string[120] = {0};
    int current_ms = 0;
    int last_send_ms = 0;
    int battery = 0;

    // Distance Sensors
    uint8_t left_sensor = 0;
    uint8_t right_sensor = 0;
    uint8_t front_sensor = 0;

    // Light Sensors
    uint8_t left_light = 0;
    uint8_t right_light = 0;

    while (1)
    {
        current_ms = milliseconds_now();

        // Sending Section
        if ((current_ms - last_send_ms) >= SEND_INTERVAL_MS)
        {
            // Distance Sensors
            left_sensor  = 280 / ((adc_read(0) >> 2) / 2);
            front_sensor = 280 / ((adc_read(1) >> 2) / 2);
            right_sensor = 280 / ((adc_read(2) >> 2) / 2);

            // Light Sensors
            right_light = (adc_read(4) >> 2);
            left_light = (adc_read(5) >> 2);

            if (left_sensor > MAX_SENSOR_SIDE) left_sensor = MAX_SENSOR_SIDE;
            if (right_sensor > MAX_SENSOR_SIDE) right_sensor = MAX_SENSOR_SIDE;
            if (front_sensor > MAX_SENSOR_FRONT) front_sensor = MAX_SENSOR_FRONT;

            battery = adc_read(3 >> 2);

            serial2_write_bytes(4, left_sensor, front_sensor, right_sensor);
            last_send_ms = current_ms;

            follow_light_drive(right_light, left_light);
        }

        // Receiving Section
        if (serial2_available())
        {
            serial2_get_data(receivedData, 4);
        }

        // servo(receivedData[2]);
        // motor_drive(receivedData[0], receivedData[1]);
        // auto_drive(front_sensor, left_sensor, right_sensor);
        
    }

    return 0;
}

// ================================
// Motor & Servo Control Functions
// ================================
void servo(uint16_t control)
{
    OCR1C = SERVO_OFFSET + (control * SERVO_SCALE);
}

void motor_drive(uint16_t horizontal_direction, uint16_t speed)
{
    rm = speed + horizontal_direction - 253;
    lm = speed - horizontal_direction;

    OCR1A = (int32_t)abs(rm) * PWM_SCALE;
    OCR1B = (int32_t)abs(lm) * PWM_SCALE;

    // Right Motor Direction
    if (rm >= 0) {
        PORTA |= (1 << PA1);
        PORTA &= ~(1 << PA0);
    } else {
        PORTA &= ~(1 << PA1);
        PORTA |= (1 << PA0);
    }

    // Left Motor Direction
    if (lm >= 0) {
        PORTB |= (1 << PB1);
        PORTB &= ~(1 << PB0);
    } else {
        PORTB &= ~(1 << PB1);
        PORTB |= (1 << PB0);
    }
}

void motor_init()
{
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1);
    TCCR1B = (1 << WGM13) | (1 << CS11);
    ICR1 = 20000;

    DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB5) | (1 << PB6) | (1 << PB7);
    DDRA |= (1 << PA0) | (1 << PA1);
}

// ======================
// Autonomous Drive Logic
// ======================
void auto_drive(uint16_t sensor_forward, uint16_t sensor_left, uint16_t sensor_right)
{
    char serial_string[120] = {0};

    if (sensor_forward < TOO_CLOSE_FORWARD)
    {
        // Turn to one side
        sprintf(serial_string, "\n I am here instead");
        serial0_print_string(serial_string);

        if (sensor_left > sensor_right) {
            motor_drive(126, 20);   // Turn Right
        } else {
            motor_drive(126, 230);  // Turn Left
        }
    }
    else if (sensor_left < TOO_CLOSE_SIDE || sensor_right < TOO_CLOSE_SIDE)
    {
        // PI Control
        sprintf(serial_string, "\n I am here broooo");
        serial0_print_string(serial_string);

        error = (int16_t)sensor_left - (int16_t)sensor_right;
        integral += error;

        float control_f = KP * error; // + KI * integral;

        if (control_f > 126) control_f = 126;
        if (control_f < -126) control_f = -126;

        control = (int16_t)control_f;

        motor_drive(50, 127 - control); // Apply control
    }
    else
    {   
      // Drive straight
        sprintf(serial_string, "\n I am here");
        serial0_print_string(serial_string);

        motor_drive(0, 127);
    }
}
void follow_light_drive(uint16_t right_light_sensor, uint16_t left_light_sensor)
{
    char serial_string[120] = {0};

    if (right_light_sensor > NO_LIGHT_VALUE && left_light_sensor > NO_LIGHT_VALUE)
    {
        // Turn to one side
        sprintf(serial_string, "\n I am here instead");
        serial0_print_string(serial_string);
        motor_drive(126, 20);   // Turn Right
    }
    else{
        // PI Control for light
        error = (int16_t)right_light_sensor - (int16_t)left_light_sensor - 4; // 4 is the offset for the light sensor
        integral += error;

        float control_f = KP_LIGHT * error; // + KI * integral;

        if (control_f > 126) control_f = 126;
        if (control_f < -126) control_f = -126;

        control = (int16_t)control_f;
        if(control>0) sprintf(serial_string, "\n Turning Left, control: %i, sensor_left: %u, sensor_right: %u", control, left_light_sensor, right_light_sensor);
        else sprintf(serial_string, "\n Turning Right, control: %i", control);
        serial0_print_string(serial_string);

        motor_drive(50, 127 - control);
    }
}

