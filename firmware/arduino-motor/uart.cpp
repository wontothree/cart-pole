#include "uart.hpp"
#include <avr/io.h>
#include <stdio.h> // for sprintf
#include <Arduino.h>

#define F_CPU 16000000UL

#define QUEUE_SIZE 256

int queue[QUEUE_SIZE];
int front = 0; // front는 큐의 시작을 가리킵니다.
int rear = 0;  // rear는 큐의 끝을 가리킵니다.
int count = 0; // 현재 큐에 있는 요소의 개수

static bool push(uint8_t value)
{
    if (count == QUEUE_SIZE)
    {
        // 큐가 가득 찬 경우
        return false;
    }
    queue[rear] = value;
    rear = (rear + 1) % QUEUE_SIZE;
    count++;
    return true;
}

static int pop()
{
    if (count == 0)
    {
        // 큐가 비어있는 경우
        return -1; // -1을 반환하여 큐가 비어있음을 알림
    }
    int value = queue[front];
    front = (front + 1) % QUEUE_SIZE; // front는 순환하도록 함
    count--;
    return value;
}

void initializeUart(uint32_t baud)
{
    unsigned int ubrr = F_CPU / 16 / baud - 1;

    // Baud Rate 설정 (UBRR 레지스터)
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)ubrr;

    // 송신 및 수신 허용 설정 (TXEN0, RXEN0)
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // 데이터 전송 형식 설정: 비동기 모드, 8비트 데이터, 1스톱 비트
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

bool putByte(uint8_t data)
{
    return push(data);
}

bool putString(char *str)
{
    for (; *str; str++)
    {
        if (!push(*str))
            return;
    }
    return true;
}

bool getByte(uint8_t *data)
{
    if (!(UCSR0A & (1 << RXC0)))
        return false;
    *data = UDR0;
    return true;
}

void processUart()
{
    if (!(UCSR0A & (1 << UDRE0)))
        return;
    int data = pop();
    if (data < 0)
        return;
    UDR0 = data;
}