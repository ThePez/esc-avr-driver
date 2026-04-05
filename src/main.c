/*
 * Copyright (c) 2026 Jack Cairns
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "serialio.h"
#include "timers.h"

#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

void run_phase(void);
void toggle_phase(void);

uint8_t phase = 0;

int main(void) {
    // Start timers
    initSystemTick();
    initTimer1_inputCapture();

    // Start Serial
    initSerialComs();

    // Set PC0 to PC5 as outputs
    DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) |
            (1 << PC5);
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) |
               (1 << PC5));

    sei();
    printf_P(PSTR("System ready!\n"));
    uint32_t prev = getSysTick();

    while (1) {
        // printf_P(PSTR("Counter: %u\n"), counter++);
        if (pwmDataReady() && getSysTick() - prev > 1000) {
            toggle_phase();
            printf_P(PSTR("PW: %uus Throttle: %u\n"), getPulseWidth(),
                     getThrottle());
            printf_P(PSTR("Phase: %d\n"), phase + 1);
            prev = getSysTick();
        }

        // Update LED pwm to match throttle
        set_led_duty(getThrottle());
        run_phase();
    }

    return 0;
}

// Need to setup 3 pins for PWM

// 3 pins will be straight on/off

// 3 pins for the resistor COM network

// 1 pin for the COM point

// 1 pin for speed input

//  1     2     3     4     5     6
// AB -> AC -> BC -> BA -> CA -> CB

// PC0 AH PC1 AL, PC2 BH PC3 BL, PC4 CH PC5 CL

void phase0(void) {
    // A High, B Low
    PORTC |= (1 << PC0) | (1 << PC3);
    PORTC &= ~((1 << PC1) | (1 << PC2) | (1 << PC4) | (1 << PC5));
}

void phase1(void) {
    // A High, C Low
    PORTC |= (1 << PC0) | (1 << PC5);
    PORTC &= ~((1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4));
}

void phase2(void) {
    // B High, C Low
    PORTC |= (1 << PC2) | (1 << PC5);
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC3) | (1 << PC4));
}

void phase3(void) {
    // B High, A Low
    PORTC |= (1 << PC2) | (1 << PC1);
    PORTC &= ~((1 << PC0) | (1 << PC3) | (1 << PC4) | (1 << PC5));
}

void phase4(void) {
    // C High, A Low
    PORTC |= (1 << PC4) | (1 << PC1);
    PORTC &= ~((1 << PC0) | (1 << PC2) | (1 << PC3) | (1 << PC5));
}

void phase5(void) {
    // C High, B Low
    PORTC |= (1 << PC4) | (1 << PC3);
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC5));
}

void toggle_phase(void) {
    phase = (phase + 1) % 6;
}

void run_phase(void) {
    switch (phase) {
    case 0:
        phase0();
        break;
    case 1:
        phase1();
        break;
    case 2:
        phase2();
        break;
    case 3:
        phase3();
        break;
    case 4:
        phase4();
        break;
    case 5:
        phase5();
        break;
    }
}
