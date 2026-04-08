/*
 * Copyright (c) 2026 Jack Cairns
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "drv8305.h"
#include <stdint.h>

#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/wdt.h>

#define SPI0_MOSI PB3
#define SPI0_MISO PB4
#define SPI0_CLK  PB5

#define DRV8305_CS_HIGH() (PORTB |= (1 << DRV8305_CS_PIN))
#define DRV8305_CS_LOW()  (PORTB &= ~(1 << DRV8305_CS_PIN))

/**
 * @brief Initialises the SPI0 peripheral and chip-select line for the DRV8305.
 *
 * @details Configures SCK (B5), MOSI (B3), and the DRV8305 chip-select pin as
 *          outputs and MISO (B4) as an input. The chip-select is deasserted
 *          immediately after. SPI is enabled in master mode with MSB-first
 *          transfer, idle-low clock polarity, and setup-on-lead/sample-on-trail
 *          phase (mode 1). The clock prescaler is set via SPR0=1 and SPI2X=1,
 *          yielding a fosc/8 SCK frequency.
 */
void drv8305SpiInit(void) {
    // Set up SPI communication as a master
    // B5 (SCK), B3 (MOSI), B2 (DRV8305 CS) as outputs, B4 (MISO) as input
    DDRB = (DDRB & ~(1 << SPI0_MISO)) | (1 << SPI0_CLK) | (1 << SPI0_MOSI) |
           (1 << DRV8305_CS_PIN);

    DRV8305_CS_HIGH();

    // Set up the SPI control registers SPCR
    // - SPE bit = 1 : SPI is enabled
    // - MSTR bit = 1 : Master Mode
    // - DORD bit = 0 : MSB first
    // - CPOL = 0 : SCK low when idle
    // - CPHA = 1 : lead:setup, trail:sample
    // Clock div /8
    SPCR0 = (1 << SPE) | (1 << MSTR) | (1 << CPHA) | (1 << SPR0);
    SPSR0 = (1 << SPI2X);
}

/**
 * @brief Transmits one byte over SPI0 and returns the simultaneously received
 *        byte.
 *
 * @details Loads @p byte into the SPI data register to begin the transfer, then
 *          spins until the SPIF interrupt flag is set, resetting the watchdog
 *          on each iteration to prevent a timeout during a long wait. The
 *          received byte is read from SPDR0 once the transfer is complete,
 *          which also clears SPIF.
 *
 * @param byte The byte to transmit.
 * @return The byte shifted in from the slave during the same transfer.
 */
static uint8_t spiSendByte(uint8_t byte) {
    SPDR0 = byte;

    while ((SPSR0 & (1 << SPIF)) == 0) {
        wdt_reset();
    }

    return SPDR0;
}

/**
 * @brief Transmits a 16-bit word over SPI0 as two consecutive bytes and returns
 *        the received word.
 *
 * @details Asserts the DRV8305 chip-select before the transfer and deasserts it
 *          afterwards. Two NOP instructions are inserted after asserting CS to
 *          satisfy any setup-time requirement before the first clock edge. The
 *          high byte is sent first, followed by the low byte, and the two
 *          received bytes are reassembled into a 16-bit return value.
 *
 * @param word The 16-bit value to transmit, high byte first.
 * @return The 16-bit value received from the slave during the transfer.
 */
static uint16_t spiSendWord(uint16_t word) {
    DRV8305_CS_LOW();
    // No ops might not be needed... Will have to test first
    _NOP();
    _NOP();
    uint8_t high_ret = spiSendByte(word >> 8);
    uint8_t low_ret = spiSendByte(word & 0xFF);
    DRV8305_CS_HIGH();
    return ((uint16_t)high_ret << 8) | low_ret;
}

/**
 * @brief Writes an 11-bit message to a DRV8305 register over SPI.
 *
 * @details Constructs a 16-bit SPI frame with the write bit (MSB) cleared, the
 *          4-bit register address in bits 14:11, and the 11-bit message payload
 *          in bits 10:0. The DRV8305 returns the previous contents of the
 *          addressed register in the same transaction.
 *
 * @param addr    4-bit register address (bits 3:0 used).
 * @param message 11-bit data payload to write (bits 10:0 used).
 * @return The 16-bit response word received from the DRV8305 during the
 *         transfer, containing the prior register value in bits 10:0.
 */
uint16_t drv8305SpiWrite(uint8_t addr, uint16_t message) {
    uint16_t word = ((addr & 0x0F) << 11) | (message & 0x07FF);
    return spiSendWord(word); // 0 in MSB, addr in next 4, then message
}

/**
 * @brief Reads an 11-bit value from a DRV8305 register over SPI.
 *
 * @details The DRV8305 read protocol requires two SPI transactions. The first
 *          sets the read bit (bit 15) and encodes the 4-bit register address in
 *          bits 14:11; the device latches the address but the data returned in
 *          this frame is not valid. A second dummy transaction with a zero
 *          payload clocks out the actual register contents, of which only the
 *          lower 11 bits are returned to the caller.
 *
 * @param addr 4-bit register address to read (bits 3:0 used).
 * @return The 11-bit register value in bits 10:0.
 */
uint16_t drv8305SpiRead(uint8_t addr) {
    uint16_t word = (1 << 15) | ((addr & 0x0F) << 11);
    spiSendWord(word);              // Send 1 in MSB and addr, dummy data
    return spiSendWord(0) & 0x07FF; // Dummy transaction to get response
}
