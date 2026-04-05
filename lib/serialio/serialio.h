#ifndef SERIALIO_H
#define SERIALIO_H

#include <stdint.h>
#include <stdio.h>

#define BAUDRATE 19200
#define BRC      ((F_CPU / (BAUDRATE * 16UL)) - 1)

#define OUTPUT_BUFFER_SIZE 100
#define INPUT_BUFFER_SIZE  32

int8_t initSerialComs(void);
void clearSerialInputBuffer(void);
int uartGetCharBlocking(FILE *stream);

#endif
