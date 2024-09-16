#ifndef UART_HPP
#define UART_HPP

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>  // for sprintf

#include "globals.hpp"

void initialize_uart(unsigned int baud);

void send_uart_char(char c);
void send_uart_string(const char* str);
void send_uart_int(int16_t num);

// UART 수신 인터럽트 핸들러 선언
ISR(USART_RX_vect);

#endif // UART_HPP
