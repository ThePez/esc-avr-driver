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

/* initSerialComs()
 * ----------------
 * Initialises the UART peripheral at the baud rate defined by BRC in
 * serialio.h. Enables RX and TX, enables the receive complete interrupt,
 * and redirects stdin/stdout to the UART stream so printf/scanf work
 * transparently over serial.
 */
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

/* serialInputAvailable()
 * ----------------------
 * Returns non-zero if there is at least one character waiting in the
 * input ring buffer, zero if the buffer is empty.
 */
int8_t serialInputAvailable(void) {
    return inputHead != inputTail;
}

/* clearSerialInputBuffer()
 * ------------------------
 * Discards all pending input by resetting the ring buffer head and tail
 * to zero. Any characters received before this call are lost.
 */
void clearSerialInputBuffer(void) {
    inputHead = 0;
    inputTail = 0;
}

/* uartPutChar()
 * -------------
 * Called by the FILE stream on every printf/putchar. Inserts the character
 * into the output ring buffer and enables the UDRE interrupt to start
 * draining it. Newline characters are expanded to \r\n so terminal
 * emulators display correctly.
 */
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

/* uartGetCharBlocking()
 * ---------------------
 * Blocking wrapper around uartGetChar(). Spins until at least one character
 * is available in the input ring buffer before returning it. Use only where
 * stalling the CPU is acceptable.
 */
int uartGetCharBlocking(FILE *stream) {
    while (inputHead == inputTail) {
        // Wait for data
    }

    return uartGetChar(stream);
}

/* uartGetChar()
 * -------------
 * Non-blocking read from the input ring buffer. Returns the next available
 * character or -1 if the buffer is empty. Called by the FILE stream on
 * every scanf/getchar.
 */
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

/* ISR(USART0_UDRE_vect)
 * ---------------------
 * Fired when the UART data register is empty and ready for the next byte.
 * Writes the next character from the output ring buffer to UDR0. Disables
 * itself when the buffer is fully drained to avoid spurious interrupts.
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

/* ISR(USART0_RX_vect)
 * -------------------
 * Fired when a byte has been received and is ready in UDR0. Stores the
 * character in the input ring buffer. Carriage returns are converted to
 * newlines so line-oriented input works consistently regardless of the
 * terminal sending \r or \r\n.
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
