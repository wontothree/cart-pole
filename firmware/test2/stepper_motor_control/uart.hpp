#ifndef UART_HPP
#define UART_HPP

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>  // sprintf 사용을 위해 추가

void initialize_uart(unsigned int baud);

void uart_send_char(char c);
void uart_send_string(const char* str);
void uart_send_int(int16_t num);

// UART 수신 인터럽트 핸들러 선언
ISR(USART_RX_vect);

// 전역 변수 선언
extern volatile int direction;        // 1 : clockwise, -1 : counter-clockwise
extern volatile bool isDirectionChanged;
extern float target_velcocity;

#endif // UART_HPP
