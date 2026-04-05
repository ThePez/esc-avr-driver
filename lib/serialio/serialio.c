/*
 * Copyright (c) 2026 Jack Cairns
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "serialio.h"

#include <avr/interrupt.h>
#include <avr/io.h>

/* ========================================================================== */
/* Function Prototypes                                                        */
/* ========================================================================== */

static int uartPutChar(char, FILE *stream);
static int uartGetChar(FILE *stream);

/* ========================================================================== */
/* USART Variables                                                            */
/* ========================================================================== */

static volatile char outputBuffer[OUTPUT_BUFFER_SIZE]; // UART output buffer
static volatile uint8_t outputHead;                    // buffer insert position
static volatile uint8_t outputTail;                    // buffer read position

static volatile char inputBuffer[INPUT_BUFFER_SIZE]; // UART input buffer
static volatile uint8_t inputHead;                   // buffer insert position
static volatile uint8_t inputTail;                   // buffer read position

// File stream used to imitate stdin/stdout on a normal PC
static FILE myStream =
    FDEV_SETUP_STREAM(uartPutChar, uartGetChar, _FDEV_SETUP_RW);

/* ========================================================================== */
/* Functions                                                                  */
/* ========================================================================== */

int8_t initSerialComs(void) {
    // Initialize our buffers
    inputHead = 0;
    inputTail = 0;
    outputHead = 0;
    outputTail = 0;
    // Setup BAUDRATE
    UBRR0L = BRC;
    UBRR0H = (BRC >> 8);
    // Enable the RX and TX UART
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    // Enable receive complete interrupt
    UCSR0B |= (1 << RXCIE0);
    // Redirect stdin and stdout to the UART
    stdout = &myStream;
    stdin = &myStream;
    clearSerialInputBuffer();
    sei();
    return 0;
}

int8_t serialInputAvailable(void) {
    return inputHead != inputTail;
}

void clearSerialInputBuffer(void) {
    inputHead = 0;
    inputTail = 0;
}

static int uartPutChar(char character, FILE *stream) {
    if (character == '\n') {
        // Convert newline into carriage return
        uartPutChar('\r', stream);
    }

    uint8_t interruptsWereEnabled = bit_is_set(SREG, SREG_I);
    cli();
    outputBuffer[outputHead] = character;
    outputHead = (outputHead + 1) % OUTPUT_BUFFER_SIZE; // Wrap when end reached
    UCSR0B |= (1 << UDRIE0);                            // Enable send interrupt
    if (interruptsWereEnabled) {
        sei();
    }

    return 0;
}

int uartGetCharBlocking(FILE *stream) {
    while (inputHead == inputTail) {
        // Wait for data
    }

    return uartGetChar(stream);
}

static int uartGetChar(FILE *stream) {
    (void)stream;
    if (inputHead == inputTail) {
        return -1;
    }

    uint8_t interruptsWereEnabled = bit_is_set(SREG, SREG_I);
    cli();
    char c = inputBuffer[inputTail];
    inputTail = (inputTail + 1) % INPUT_BUFFER_SIZE; // Wrap when end reached
    if (interruptsWereEnabled) {
        sei();
    }

    return c;
}

/* ISR(USART_UDRE_vect)
 * --------------------
 * Interrupt service routine for handling the output of data over USART.
 */
ISR(USART0_UDRE_vect) {
    if (outputHead != outputTail) {
        // We have data to output...
        UDR0 = outputBuffer[outputTail];
        outputTail =
            (outputTail + 1) % OUTPUT_BUFFER_SIZE; // Wrap when end is reached
    } else {
        UCSR0B &= ~(1 << UDRIE0); // No data to send so disable the interrupt
    }
}

/* ISR(USART_RX_vect)
 * ------------------
 * Interrupt service routine for handling data being received over USART.
 */
ISR(USART0_RX_vect) {
    char c = UDR0;
    if (c == '\r') {
        // Turn carriage return into newline
        c = '\n';
    }

    inputBuffer[inputHead] = c;
    // Wrap insert position when end is reached
    inputHead = (inputHead + 1) % INPUT_BUFFER_SIZE;
}
