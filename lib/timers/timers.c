/*
 * Copyright (c) 2026 Jack Cairns
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "timers.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

/* ========================================================================== */
/* Defines                                                                    */
/* ========================================================================== */

// Timer1 input capture: 16MHz / 8 = 2MHz -> 1 tick = 0.5us
// Pulse width 1ms = 2000 ticks, 2ms = 4000 ticks
// Period 20ms = 40000 ticks -- fits in uint16_t (max 65535)
#define TIMER1_PRESCALER 8
#define TICKS_PER_US     (F_CPU / TIMER1_PRESCALER / 1000000UL) // = 2

// Timer3/4 phase correct PWM: 16MHz / (2 * 1 * 400) = 20kHz
#define PWM_TOP 400

/* ========================================================================== */
/* Input Capture State                                                        */
/* ========================================================================== */

static volatile uint16_t risingEdgeTime;
static volatile uint16_t pulseWidth;
static volatile uint16_t period;
static volatile uint8_t newDataReady;

/* ========================================================================== */
/* System Tick                                                                */
/* ========================================================================== */

static volatile uint32_t systemTicks = 0;

/* ========================================================================== */
/* Init Functions                                                             */
/* ========================================================================== */

/* initSystemTick()
 * ----------------
 * Sets up Timer0 in CTC mode to generate a 1ms system tick.
 * 16MHz / 64 / 250 = 1000Hz
 */
void initSystemTick(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    TCCR0A = (1 << WGM01);              // CTC mode
    OCR0A = 249;                        // 0-indexed, 250 counts = 1ms
    TIMSK0 = (1 << OCIE0A);             // Enable compare match interrupt
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler /64

    if (interrupts) {
        sei();
    }
}

/* initTimer1_inputCapture()
 * -------------------------
 * Sets up Timer1 for RC PWM input capture on ICP1 (PB0 / D8).
 * Prescaler /8 -> 2MHz tick -> 0.5us resolution.
 * Captures rising and falling edges to measure pulse width and period.
 */
void initTimer1_inputCapture(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    // PB0 (ICP1) as input with pullup
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);

    TCNT1 = 0;
    TCCR1A = 0;

    // Enable the input capture vector
    TIMSK1 = (1 << ICIE1);

    // Noise canceller on, rising edge first, prescaler /8
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);

    newDataReady = 0;

    if (interrupts) {
        sei();
    }
}

/* initTimer2_led_pwm()
 * ----------------
 * Sets up Timer2 in Fast PWM mode on OC2A (PB3 / D11) for LED testing.
 * 16MHz / 1 / 256 = 62.5kHz - above flicker threshold for LED testing.
 */
void initTimer2_led_pwm(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    // PB3 (OC2A / D11) as output
    DDRB |= (1 << PB3);

    // Fast PWM, non-inverting on OC2A
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS20); // No prescaler -> 62.5kHz

    OCR2A = 0; // Start at 0% duty

    if (interrupts) {
        sei();
    }
}

/* initTimer_gate_pwm()
 * ----------------
 * Sets up Timer3 in phase correct PWM mode on OC3A (PD0) and OC3B (PD2).
 * Sets up Timer4 in phase correct PWM mode on OC4A (PD1).
 * 16MHz / (2 * 1 * 400) = 20kHz.
 * For use with gate drivers -- Phase A and Phase B high side.
 * For use with gate drivers -- Phase C high side.
 */
void initTimer_gate_pwm(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    // OC3A (PD0) and OC3B (PD2) as outputs
    DDRD |= (1 << PD0) | (1 << PD2);
    // OC4A (PD1) as output
    DDRD |= (1 << PD1);

    // Phase correct PWM mode 10 (TOP = ICR3), non-inverting OC3A and OC3B
    TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31);
    // Phase correct PWM mode 10 (TOP = ICR4), non-inverting OC4A
    TCCR4A = (1 << COM4A1) | (1 << WGM41);

    ICR3 = PWM_TOP;
    ICR4 = PWM_TOP;
    OCR3A = 0;
    OCR3B = 0;
    OCR4A = 0;

    // Start the timers
    TCCR3B = (1 << WGM33) | (1 << CS30); // No prescaler
    TCCR4B = (1 << WGM43) | (1 << CS40); // No prescaler

    if (interrupts) {
        sei();
    }
}

/* ========================================================================== */
/* Disable Functions                                                          */
/* ========================================================================== */

void disableTimer0(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TIMSK0 = 0;

    if (interrupts) {
        sei();
    }
}

void disableTimer1(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    TCCR1B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TIMSK1 = 0;

    if (interrupts) {
        sei();
    }
}

void disableTimer2(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    TCCR2B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TIMSK2 = 0;

    if (interrupts) {
        sei();
    }
}

void disableTimer3(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    TCCR3B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TIMSK3 = 0;

    if (interrupts) {
        sei();
    }
}

void disableTimer4(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    TCCR4B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TIMSK4 = 0;

    if (interrupts) {
        sei();
    }
}

void disableTimers(void) {
    disableTimer0();
    disableTimer1();
    disableTimer2();
    disableTimer3();
    disableTimer4();
}

/* ========================================================================== */
/* Duty Setters                                                               */
/* ========================================================================== */

/* setLEDDuty()
 * ------------
 * Set Timer2 PWM duty cycle for LED testing.
 * throttlePerMille: 0-1000 maps to 0-100% duty
 */
void set_led_duty(uint16_t throttlePerMille) {
    OCR2A = (uint8_t)((uint32_t)throttlePerMille * 255 / 1000);
}

/* setGateDuty()
 * -------------
 * Set duty cycle on all three gate driver PWM outputs simultaneously.
 * throttlePerMille: 0-1000 maps to 0-100% duty (0 to PWM_TOP)
 * All three outputs are updated atomically.
 */
void set_gate_duty(uint16_t throttlePerMille) {
    uint16_t duty = (uint16_t)((uint32_t)throttlePerMille * PWM_TOP / 1000);
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    OCR3A = duty; // Phase A
    OCR3B = duty; // Phase B
    OCR4A = duty; // Phase C
    if (interrupts) {
        sei();
    }
}

/* ========================================================================== */
/* Input Capture Getters                                                      */
/* ========================================================================== */

uint8_t pwmDataReady(void) {
    return newDataReady;
}

void clearPwmDataReady(void) {
    newDataReady = 0;
}

// Returns pulse width in microseconds
uint16_t getPulseWidth(void) {
    uint16_t pw;
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    pw = pulseWidth;
    if (interrupts) {
        sei();
    }

    return pw / TICKS_PER_US;
}

// Returns period in microseconds
uint16_t getPeriod(void) {
    uint16_t p;
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    p = period;
    if (interrupts) {
        sei();
    }

    return p / TICKS_PER_US;
}

// Returns frequency in Hz
uint16_t getFrequency(void) {
    uint16_t p;
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    p = period;
    if (interrupts) {
        sei();
    }

    return (p == 0) ? 0 : (F_CPU / TIMER1_PRESCALER) / p;
}

/* Returns raw duty cycle of RC signal as 0-1000 per mille
 * Note: RC signal at 50Hz will read ~50 to 100, NOT throttle position
 * Use getThrottlePerMille() for throttle
 */
uint16_t getRawDutyPerMille(void) {
    uint16_t pw, p;
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    pw = pulseWidth;
    p = period;
    if (interrupts) {
        sei();
    }

    if (p == 0) {
        return 0;
    }
    return (p == 0) ? 0 : (uint16_t)(((uint32_t)pw * 1000) / p);
}

// Returns throttle as 0-1000 per mille (maps 1ms-2ms pulse to 0-100.0%)
uint16_t getThrottlePerMille(void) {
    uint16_t pw = getPulseWidth();
    return (pw <= 1000) ? 0 : (pw >= 2000) ? 1000 : pw - 1000;
}

/* ========================================================================== */
/* System Tick Getters                                                        */
/* ========================================================================== */

// Returns milliseconds elapsed since initSystemTick() was called
uint32_t getSysTick(void) {
    uint32_t ticks;
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    ticks = systemTicks;
    if (interrupts) {
        sei();
    }

    return ticks;
}

/* ========================================================================== */
/* ISR Functions                                                              */
/* ========================================================================== */

ISR(TIMER0_COMPA_vect) {
    systemTicks++;
}

ISR(TIMER1_CAPT_vect) {
    uint16_t captured = ICR1;

    if (TCCR1B & (1 << ICES1)) {
        // Rising edge -- save timestamp, record period, switch to falling
        period = captured - risingEdgeTime;
        risingEdgeTime = captured;
        TCCR1B &= ~(1 << ICES1);
    } else {
        // Falling edge -- calculate pulse width, switch to rising
        pulseWidth = captured - risingEdgeTime;
        TCCR1B |= (1 << ICES1);
        newDataReady = 1;
    }
}
