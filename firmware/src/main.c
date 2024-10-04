#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "math.h"

#include "pid.h"

#define abs(x) ((x) < 0 ? -(x) : (x))

// UART configuration
#define UART_ID uart0
#define BAUD_RATE 38400
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define BUFFER_SIZE 50

#define PI 3.141592

volatile float acceleration = 0.0f; // Revolutions per second squared
volatile float velocity = 0;        // Revolutions per second
volatile int32_t position = 0;      // Steps
volatile float angle = 0;           // Angle

#define STATE_IDLE 0x01
#define STATE_READY 0x02
#define STATE_READ 0x04

uint8_t c;
char buffer[BUFFER_SIZE];
int uart_buf_index = 0;
uint8_t state = STATE_IDLE;

void uart_callback()
{

    while (uart_is_readable(UART_ID))
    {
        c = uart_getc(UART_ID);

        switch (state)
        {
        case STATE_IDLE:
        {
            if (c != '$')
                break;
            state = STATE_READY;
            break;
        }

        case STATE_READY:
        {
            if (c == ',')
            {
                uart_buf_index = 0;
                state = STATE_READ;
            }
            break;
        }

        case STATE_READ:
        {
            if (c == ',')
            {
                buffer[uart_buf_index++] = '\0';
                angle = -(float)(atof(buffer) - 180);
                state = STATE_IDLE;
                break;
            }
            if (c == '.' || (c >= '0' && c <= '9'))
            {
                buffer[uart_buf_index++] = c;
            }
            break;
        }
        }
    }
}

/*
 * Core 0 is responsible for acceleration control
 */
void core0_main()
{
    // USB serial initialization
    stdio_init_all();
    while (!stdio_usb_connected())
    {
        sleep_ms(10);
    }
    printf("USB serial initialized\n");

    // UART initialization (for encoder sensor reading)
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, false);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, uart_callback); // UART IRQ handler 등록
    irq_set_enabled(UART_IRQ, true);                    // UART IRQ 라인 활성화
    uart_set_irq_enables(UART_ID, true, false);         // UART RX IRQ 활성화, UART TX IRQ 비활성화
    printf("UART initialized\n");

    // Stop switch variables
    const uint STOP_SWITCH_PIN = 16;
    gpio_init(STOP_SWITCH_PIN);
    gpio_set_dir(STOP_SWITCH_PIN, GPIO_IN);

    // Stepper position initialization step
    {
        // Fast homing
        printf("Initializing...\n");
        velocity = 3;
        while (!gpio_get(STOP_SWITCH_PIN))
            ;
        velocity = 0;

        // Slightly back off from the switch
        position = 100;
        velocity = -3;
        while (position > 0)
            ;

        // Precise homing
        velocity = 0.25;
        while (!gpio_get(STOP_SWITCH_PIN))
            ;
        position = 1900; // Full distance is 3800 steps
        velocity = 0;

        float a = 90; // Maximum acceleration available
        float min_speed_const = -999990;
        while (position > 0)
        {
            float min_speed_dist = -sqrt(2 * a * position / 400.f);
            if (velocity <= min_speed_dist)
            {
                velocity = min_speed_dist;
            }
            else if (velocity <= min_speed_const)
            {
                velocity = min_speed_const;
            }
            else
            {
                acceleration = -a;
            }
        }
        velocity = 0;
        acceleration = 0;
    }

    // Wait until abs angle is small enough
    while (abs(angle) > 1.0f)
    {
        sleep_ms(1);
    };

    // Log variables
    uint32_t last_log_time = time_us_32();
    uint32_t log_interval = 10000; // 10ms

    // balancing PID variables
    uint32_t last_control_time = time_us_32();
    uint32_t control_interval = 1000; // 1ms;
    float last_error = 0;
    float kP = 10.f;
    float kD = 1.0f;

    // Position PID variables
    float last_position_error = 0;
    float kPPos = 4.f / 300.f;
    float kDPos = 0.2f / 300.f;

    // Main loop
    while (true)
    {
        // Update velocity
        uint32_t current_time = time_us_32();
        if (current_time - last_control_time >= control_interval)
        {
            uint32_t dt = current_time - last_control_time;
            last_control_time += control_interval;

            float posErr = position;
            float dPosErr = (posErr - last_position_error) * 1000000.f / dt;
            last_position_error = posErr;
            float posCompensation = posErr * kPPos + dPosErr * kDPos;

            float err = angle + posCompensation;
            float dErr = (err - last_error) * 1000000.f / dt;
            last_error = err;
            acceleration = err * kP + dErr * kD;
        }

        // // Log acceleration every second
        // if (current_time - last_log_time >= log_interval)
        // {
        //     last_log_time += log_interval;
        //     printf("P:%ldV:%f\n", position, velocity);
        //     printf("%f\n", acceleration);
        // }

        // If the stop switch is pressed, set both velocity and acceleration to 0
        if (gpio_get(STOP_SWITCH_PIN) || position > 1900 || position < -1900 || abs(angle) > 45)
        {
            velocity = 0;
            acceleration = 0;
        }
    }
}

/*
 * Core 1 is responsible for driving the stepper motor
 */
void core1_main()
{
    // Initialize GPIO pins for stepper motor
    const uint PIN_A = 10;
    const uint PIN_B = 11;
    const uint PIN_C = 12;
    const uint PIN_D = 13;

    gpio_init(PIN_A);
    gpio_init(PIN_B);
    gpio_init(PIN_C);
    gpio_init(PIN_D);

    gpio_set_dir(PIN_A, GPIO_OUT);
    gpio_set_dir(PIN_B, GPIO_OUT);
    gpio_set_dir(PIN_C, GPIO_OUT);
    gpio_set_dir(PIN_D, GPIO_OUT);

    // Pin output map for each step
    const uint8_t pin_map[8][4] = {
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 0, 1},
    };

    // Step motor variables
    uint64_t max_interval = 50000; // 100ms
    uint64_t time_us = 0;
    uint64_t time_last_us = 0;

    // Velocity control variables
    uint32_t update_interval = 1000; // 1ms
    uint32_t last_update_time = time_us_32();

    while (true)
    {
        time_us = time_us_64();
        if (abs(velocity) > 0.01)
        {
            float interval = (1000000.0f / 400.f) / (abs(velocity));

            // Step the motor if the time since the last step is greater than the interval
            if (time_us - time_last_us > interval)
            {
                time_last_us = time_us;
                for (int i = 0; i < 4; i++)
                {
                    gpio_put(PIN_A + i, pin_map[position & 0x07][i]);
                }
                position += velocity > 0 ? 1 : -1;
            }
        }

        // Update velocity every 1ms
        time_us = time_us_64();
        if (time_us - last_update_time >= update_interval)
        {
            velocity += acceleration * (time_us - last_update_time) / 1000000.0f;
            last_update_time += update_interval;
        }

        // Limit the step holding time to max_interval
        if (time_us - time_last_us > max_interval)
        {
            for (int i = 0; i < 4; i++)
            {
                gpio_put(PIN_A + i, 0);
            }
        }
    }
}

/*
 * Entrypoint
 */
int main()
{
    multicore_launch_core1(core1_main);
    core0_main();
    return 0;
}