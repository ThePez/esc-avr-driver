/*
 * Copyright (c) 2026 Jack Cairns
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DRV8305_H
#define DRV8305_H

#include <stdint.h>

#define DRV8305_CS_PIN PB2

void drv8305SpiInit(void);
uint16_t drv8305SpiRead(uint8_t addr);
uint16_t drv8305SpiWrite(uint8_t addr, uint16_t message);

#endif
