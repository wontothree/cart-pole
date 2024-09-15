#include "uart.hpp"

// 전역 변수 정의
volatile int direction = 1;        // 1 : clockwise, -1 : counter-clockwise
volatile bool isDirectionChanged = false;
float target_velcocity = 1;

// UART 초기화 함수 구현
void initialize_uart(unsigned int baud) {
    unsigned int ubrr = F_CPU/16/baud - 1;  // UBRR 계산 (F_CPU는 CPU 주파수)

    // Baud rate 설정
    UBRR0H = (unsigned char)(ubrr >> 8);   // 상위 8비트 설정
    UBRR0L = (unsigned char)ubrr;          // 하위 8비트 설정

    // 송신 및 수신 활성화, 8비트 데이터 크기 설정
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // RXEN0: 수신 활성화, TXEN0: 송신 활성화, RXCIE0: 수신 인터럽트 활성화
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // UCSZ01, UCSZ00: 8비트 데이터 프레임 설정
}

// UART 송신 함수 구현
void send_uart_char(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // 데이터 레지스터가 비어 있는지 확인
    UDR0 = c; // 데이터 레지스터에 문자 전송
}

void send_uart_string(const char* str) {
    while (*str) {
        send_uart_char(*str++);
    }
}

void send_uart_int(int16_t num) {
    char buffer[6]; // -5000에서 5000까지의 숫자를 문자열로 표현하기 위한 버퍼
    snprintf(buffer, sizeof(buffer), "%d", num); // 정수를 문자열로 변환
    send_uart_string(buffer); // 문자열 전송
}

// UART 수신 인터럽트 핸들러 구현
ISR(USART_RX_vect) {
    uint8_t receivedByte = UDR0;
    if ((receivedByte >= '0') && (receivedByte <= '9')) {
        target_velcocity = (receivedByte - '0') * 0.4f;
    } else if (receivedByte == ' ') { // 스페이스바 입력
        isDirectionChanged = true; // 방향 전환 플래그 설정
    }
}
