    #include "uart.hpp"

    volatile char tx_buffer[BUFFER_SIZE];
    volatile uint8_t tx_head = 0;
    volatile uint8_t tx_tail = 0;

    void initialize_uart(unsigned int baud) {
        unsigned int ubrr = F_CPU/16/baud - 1;  // UBRR 계산 (F_CPU는 CPU 주파수)

        // Baud rate 설정
        UBRR0H = (unsigned char)(ubrr >> 8);   // 상위 8비트 설정
        UBRR0L = (unsigned char)ubrr;          // 하위 8비트 설정

        // 송신 및 수신 활성화, 8비트 데이터 크기 설정
        UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);  // RXEN0: 수신 활성화, TXEN0: 송신 활성화, RXCIE0: 수신 인터럽트 활성화
        UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);                // UCSZ01, UCSZ00: 8비트 데이터 프레임 설정
    }

    // test
    void enqueue_uart_char(char c) {
        uint8_t next_head = (tx_head + 1) % BUFFER_SIZE;
        while (next_head == tx_tail); // 버퍼가 가득 찰 때까지 대기
        tx_buffer[tx_head] = c;
        tx_head = next_head;
        UCSR0B |= (1 << UDRIE0); // 송신 인터럽트 활성화
    }

    void send_uart_char(char c) {
        enqueue_uart_char(c);
        // while (!(UCSR0A & (1 << UDRE0)));               // is data register empty
        // UDR0 = c;                                       // send character to data register
        // if (UCSR0A & (1 << UDRE0)) {
        //     UDR0 = c; // 데이터 레지스터에 문자 전송
        // }
    }

    void send_uart_string(const char* str) {
        while (*str) {
            send_uart_char(*str++);
        }
    }

    void send_uart_int(uint16_t num) {                  // 0 ~ 65,535
        char buffer[6];                                 // 5 자리
        snprintf(buffer, sizeof(buffer), "%u", num);    // integer to string
        send_uart_string(buffer);                       // send string
    }

    // velocity
    // UART 수신 인터럽트 핸들러
    ISR(USART_RX_vect) {
        uint8_t receivedByte = UDR0;
        if ((receivedByte >= '0') && (receivedByte <= '9')) {
            target_velocity = (receivedByte - '0'); // * 0.4f;
        } else if (receivedByte == ' ') { // 스페이스바 입력
            isDirectionChanged = true; // 방향 전환 플래그 설정
        }
    }

    // // acceleration
    // // UART 수신 인터럽트 핸들러
    // ISR(USART_RX_vect) {
    //     uint8_t receivedByte = UDR0;
    //     if ((receivedByte >= '0') && (receivedByte <= '9')) {
    //         target_current_linear_acceleration = (receivedByte - '0');
    //     } else if (receivedByte == ' ') { // 스페이스바 입력
    //         isDirectionChanged = true; // 방향 전환 플래그 설정
    //     }
    // }

    // UART 송신 인터럽트 핸들러
    ISR(USART_UDRE_vect) {
        if (tx_tail != tx_head) {
            UDR0 = tx_buffer[tx_tail];
            tx_tail = (tx_tail + 1) % BUFFER_SIZE;
        } else {
            UCSR0B &= ~(1 << UDRIE0); // 버퍼가 비어있으면 송신 인터럽트 비활성화
        }
    }
