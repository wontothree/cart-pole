#pragma once

#include <stdint.h>
#include <stdbool.h>

void initializeUart(uint32_t baudrate);
bool putByte(uint8_t data);
bool putString(char *str);
bool getByte(uint8_t *data);
void processUart();
