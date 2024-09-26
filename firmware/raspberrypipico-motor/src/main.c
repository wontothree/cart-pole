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

#define BUFFER_SIZE 100

#define PI 3.141592

volatile float acceleration = 0.0f; // Revolutions per second squared
volatile float velocity = 0;        // Revolutions per second
volatile int32_t position = 0;      // Steps

/**
 * UART RX test IRQ Handler
 */

// uart 입력이 들어올 때마다 자동으로 호출되는 함수
// IRQ handler
void uart_callback()
{
    while (uart_is_readable(UART_ID))
    {
        static uint8_t ch;
        ch = uart_getc(UART_ID);
        static char buffer[BUFFER_SIZE];
        static int index = 0;

        buffer[index++] = ch;

        if (ch == '\n')
        {
            buffer[index] = '\0';

            // select the second - angle
            if (strncmp(buffer, "$ANG", 4) == 0)
            {
                int comma_count = 0;
                int second_value_start = 0;

                for (int i = 0; buffer[i] != '\0'; i++)
                {
                    if (buffer[i] == ',')
                    {
                        comma_count++;
                        if (comma_count == 1)
                        {
                            second_value_start = i + 1;
                            break;
                        }
                    }
                }
                // find index of the second
                int second_value_end = second_value_start;
                while (buffer[second_value_end] != ',' && buffer[second_value_end] != '\0')
                {
                    second_value_end++;
                }

                // obtain the second
                char second_value_str[BUFFER_SIZE];
                strncpy(second_value_str, buffer + second_value_start, second_value_end - second_value_start);
                second_value_str[second_value_end - second_value_start] = '\0';

                double second_value_double = atof(second_value_str);

                printf("%lf\n", second_value_double);
            }

            index = 0;
        }
    }
}

/**
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

    // ,,,
    //    uart_set_hw_flow(UART_ID, false, false);
    //  uart_set_fifo_enabled(UART_ID, false);
    //   int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // q_set_exclusive_handler(UART_IRQ, uart_callback); // UART IRQ handler 등록
    //  irq_set_enabled(UART_IRQ, true);                    // UART IRQ 라인 활성화
    //    uart_set_irq_enables(UART_ID, true, false);         // UART RX IRQ 활성화, UART TX IRQ 비활성화
    printf("UART initialized\n");

    // Acceleration control variables
    int32_t sign = 1;
    int32_t acceleration_buffer = 0;

    // Log variables
    uint32_t last_log_time = time_us_32();
    uint32_t log_interval = 10000; // 10ms

    // Stop switch variables
    const uint STOP_SWITCH_PIN = 16;
    gpio_init(STOP_SWITCH_PIN);
    gpio_set_dir(STOP_SWITCH_PIN, GPIO_IN);

    // Initialization step
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

    // Main loop
    while (true)
    {
        // Update velocity
        uint32_t current_time = time_us_32();

        // Read acceleration from USB serial
        int c = getchar_timeout_us(0);
        if (c == '-')
        {
            sign = -1;
        }
        else if (c >= '0' && c <= '9')
        {
            acceleration_buffer = acceleration_buffer * 10 + (c - '0');
        }
        else if (c == '\r' || c == '\n')
        {
            acceleration = sign * acceleration_buffer / 100.0f;
            sign = 1;
            acceleration_buffer = 0;
        }

        // Log acceleration every second
        if (current_time - last_log_time >= log_interval)
        {
            last_log_time += log_interval;
            printf("P:%ldV:%f\n", position, velocity);
        }

        // If the stop switch is pressed, set both velocity and acceleration to 0
        if (gpio_get(STOP_SWITCH_PIN))
        {
            velocity = 0;
            acceleration = 0;
        }

        if (position < -1900)
        {
            velocity = 0;
            acceleration = 0;
        }
    }
}

/**
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

        // // Update velocity every 1ms
        time_us = time_us_64();
        if (time_us - last_update_time >= update_interval)
        {
            velocity += acceleration * (time_us - last_update_time) / 1000000.0f;
            last_update_time += update_interval;
        }

        // pid
        setPID(1.0, 0, 0);

        setTargetAngle(0);
        if (time_us - last_update_time >= update_interval)
        {

            // float acceleration = controlByPID(angle);
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
