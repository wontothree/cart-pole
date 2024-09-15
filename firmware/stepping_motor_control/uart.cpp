#include "uart.hpp"
#include <Arduino.h>

#define UDR0 (*(volatile uint8_t *)(0xC6))   // USART Data Register
#define UCSRA (*(volatile uint8_t *)(0xC0))  // USART Control and Status Register A
#define RXC0 7                               // USART Receive Complete bit position

void init_UART(unsigned int baud) {
    unsigned int ubrr = F_CPU/16/baud - 1;  // UBRR 계산 (F_CPU는 CPU 주파수)

    // Baud rate 설정
    UBRR0H = (unsigned char)(ubrr >> 8);   // 상위 8비트 설정
    UBRR0L = (unsigned char)ubrr;          // 하위 8비트 설정

    // 송신 및 수신 활성화, 8비트 데이터 크기 설정
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // RXEN0: 수신 활성화, TXEN0: 송신 활성화, RXCIE0: 수신 인터럽트 활성화
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // UCSZ01, UCSZ00: 8비트 데이터 프레임 설정
}
