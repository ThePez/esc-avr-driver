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
 * Sets up Timer0 in CTC mode to generate a 1ms system tick interrupt.
 * 16MHz / 64 / 250 = 1000Hz. Increments systemTicks in the ISR.
 */
void initSystemTick(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    TCCR0A = (1 << WGM01);              // CTC mode
    OCR0A = 249;                        // 0-indexed, 250 counts = 1ms
    TIMSK0 = (1 << OCIE0A);             // Enable compare match interrupt
    TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler /64 -- starts the timer

    if (interrupts) {
        sei();
    }
}

/* initTimer1_inputCapture()
 * -------------------------
 * Sets up Timer1 for RC PWM input capture on ICP1 (PB0 / D8).
 * Prescaler /8 gives a 2MHz tick rate and 0.5us resolution.
 * Noise canceller enabled to reject glitches shorter than 4 clock cycles.
 * Captures rising edges to measure period and falling edges to measure
 * pulse width. Results are available via the input capture getters below.
 */
void initTimer1_inputCapture(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    // PB0 (ICP1) as input with pullup
    DDRB &= ~(1 << PB0);
    PORTB |= (1 << PB0);

    TCNT1 = 0;
    TCCR1A = 0;

    TIMSK1 = (1 << ICIE1); // Enable input capture interrupt

    // Noise canceller on, rising edge first, prescaler /8 -- starts the timer
    TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);

    newDataReady = 0;

    if (interrupts) {
        sei();
    }
}

/* initTimer2_led_pwm()
 * --------------------
 * Sets up Timer2 in Fast PWM mode on OC2A (PB3 / D11) for LED brightness
 * testing. No prescaler gives 16MHz / 256 = 62.5kHz, well above the
 * flicker threshold so brightness appears smooth. Duty cycle is set via
 * set_led_duty(). Output starts at 0% (LED off).
 */
void initTimer2_led_pwm(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    DDRB |= (1 << PB3); // PB3 (OC2A / D11) as output

    // Fast PWM mode, non-inverting output on OC2A
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS20); // No prescaler -> 62.5kHz -- starts the timer

    OCR2A = 0; // Start at 0% duty

    if (interrupts) {
        sei();
    }
}

/* initTimer_gate_pwm()
 * --------------------
 * Sets up Timer3 and Timer4 in phase correct PWM mode for gate driver use.
 * Phase correct PWM produces a symmetrical waveform which reduces current
 * ripple in motor windings compared to fast PWM.
 *   Timer3 OC3A (PD0): Phase A high side
 *   Timer3 OC3B (PD2): Phase B high side
 *   Timer4 OC4A (PD1): Phase C high side
 * TOP = 400 gives 16MHz / (2 * 1 * 400) = 20kHz.
 * Both timers are started last and simultaneously to minimise phase offset.
 * Duty cycle is set via set_gate_duty().
 */
void initTimer_gate_pwm(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();

    // OC3A (PD0), OC3B (PD2), OC4A (PD1) as outputs
    DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD2);

    // Phase correct PWM mode 10 (TOP = ICR3/4), non-inverting outputs
    TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << WGM31);
    TCCR4A = (1 << COM4A1) | (1 << WGM41);

    ICR3 = PWM_TOP;
    ICR4 = PWM_TOP;
    OCR3A = 0;
    OCR3B = 0;
    OCR4A = 0;

    // Start both timers -- written last to minimise phase offset between them
    TCCR3B = (1 << WGM33) | (1 << CS30); // No prescaler
    TCCR4B = (1 << WGM43) | (1 << CS40); // No prescaler

    if (interrupts) {
        sei();
    }
}

/* ========================================================================== */
/* Disable Functions                                                          */
/* ========================================================================== */

/* disableTimer0()
 * ---------------
 * Stops Timer0 by clearing all prescaler bits and disabling its interrupts.
 * Also stops the system tick -- getSysTick() will return a frozen value.
 */
static void disableTimer0(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
    TIMSK0 = 0;
    if (interrupts) {
        sei();
    }
}

/* disableTimer1()
 * ---------------
 * Stops Timer1 by clearing all prescaler bits and disabling its interrupts.
 * Also stops input capture -- pwmDataReady() will not update after this.
 */
static void disableTimer1(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    TIMSK1 = 0;
    if (interrupts) {
        sei();
    }
}

/* disableTimer2()
 * ---------------
 * Stops Timer2 by clearing all prescaler bits and disabling its interrupts.
 * OC2A (PB3 / D11) output will hold its last state.
 */
static void disableTimer2(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
    TIMSK2 = 0;
    if (interrupts) {
        sei();
    }
}

/* disableTimer3()
 * ---------------
 * Stops Timer3 by clearing all prescaler bits and disabling its interrupts.
 * OC3A and OC3B gate drive outputs will hold their last state.
 */
static void disableTimer3(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));
    TIMSK3 = 0;
    if (interrupts) {
        sei();
    }
}

/* disableTimer4()
 * ---------------
 * Stops Timer4 by clearing all prescaler bits and disabling its interrupts.
 * OC4A gate drive output will hold its last state.
 */
static void disableTimer4(void) {
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    TCCR4B &= ~((1 << CS42) | (1 << CS41) | (1 << CS40));
    TIMSK4 = 0;
    if (interrupts) {
        sei();
    }
}

/* disableTimers()
 * ---------------
 * Convenience function that stops all five timers. Safe to call at any
 * time -- each timer is disabled atomically with interrupts guarded.
 */
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

/* set_led_duty()
 * --------------
 * Sets the Timer2 Fast PWM duty cycle for LED brightness testing.
 * throttlePerMille: 0 (off) to 1000 (full brightness).
 * Maps the 0-1000 range to the 0-255 OCR2A register range.
 */
void set_led_duty(uint16_t throttlePerMille) {
    OCR2A = (uint8_t)((uint32_t)throttlePerMille * 255 / 1000);
}

/* set_gate_duty()
 * ---------------
 * Sets the duty cycle on all three gate driver PWM outputs simultaneously.
 * throttlePerMille: 0 (0% duty) to 1000 (100% duty).
 * Maps the 0-1000 range to 0-PWM_TOP for the OCR registers.
 * All three compare registers are updated inside a cli/sei guard to ensure
 * they change atomically and no phase sees a mismatched duty mid-cycle.
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

/* pwmDataReady()
 * --------------
 * Returns non-zero if a complete pulse width measurement is available.
 * Set by the input capture ISR on each falling edge. Remains set until
 * cleared by clearPwmDataReady().
 */
uint8_t pwmDataReady(void) {
    return newDataReady;
}

/* clearPwmDataReady()
 * -------------------
 * Clears the data ready flag set by the input capture ISR. Call after
 * consuming a measurement to detect the next fresh pulse.
 */
void clearPwmDataReady(void) {
    newDataReady = 0;
}

/* getPulseWidth()
 * ---------------
 * Returns the most recent pulse width measurement in microseconds.
 * For a standard RC PWM signal this will be in the range 1000-2000us.
 * Read is performed inside a cli/sei guard as pulseWidth is a 16-bit
 * volatile updated by the ISR and not atomically readable on an 8-bit CPU.
 */
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

/* getPeriod()
 * -----------
 * Returns the most recent period measurement in microseconds.
 * For a 50Hz RC signal this will be approximately 20000us.
 * Read is guarded against ISR corruption as per getPulseWidth().
 */
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

/* getFrequency()
 * --------------
 * Returns the frequency of the incoming PWM signal in Hz derived from the
 * measured period. Returns 0 if no signal has been received yet.
 * For a standard RC signal this will be approximately 50Hz.
 */
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

/* getRawDuty()
 * ------------
 * Returns the raw duty cycle of the incoming RC signal as 0-1000 per mille.
 * A 50Hz signal with a 1ms pulse will read ~50, with a 2ms pulse ~100.
 * This is NOT throttle position -- use getThrottle() for that.
 * Useful for sanity checking the signal frequency and duty are plausible.
 */
uint16_t getRawDuty(void) {
    uint16_t pw, p;
    uint8_t interrupts = bit_is_set(SREG, SREG_I);
    cli();
    pw = pulseWidth;
    p = period;
    if (interrupts) {
        sei();
    }

    return (p == 0) ? 0 : (uint16_t)(((uint32_t)pw * 1000) / p);
}

/* getThrottle()
 * -------------
 * Returns the throttle demand as 0-1000 per mille by mapping the RC pulse
 * width range of 1000-2000us to 0-1000. Values at or below 1000us return 0,
 * values at or above 2000us return 1000. 1500us returns 500 (50.0%).
 */
uint16_t getThrottle(void) {
    uint16_t pw = getPulseWidth();
    return (pw <= 1000) ? 0 : (pw >= 2000) ? 1000 : pw - 1000;
}

/* ========================================================================== */
/* System Tick Getters                                                        */
/* ========================================================================== */

/* getSysTick()
 * ------------
 * Returns the number of milliseconds elapsed since initSystemTick() was
 * called. Wraps after approximately 49 days. Read is guarded against ISR
 * corruption as systemTicks is a 32-bit value updated by the Timer0 ISR
 * and not atomically readable on an 8-bit CPU.
 */
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

/* ISR(TIMER0_COMPA_vect)
 * ----------------------
 * Fired every 1ms by Timer0 CTC compare match. Increments the systemTicks
 * counter used by getSysTick() to provide a millisecond timebase.
 */
ISR(TIMER0_COMPA_vect) {
    systemTicks++;
}

/* ISR(TIMER1_CAPT_vect)
 * ---------------------
 * Fired on each input capture edge on ICP1 (PB0 / D8). On a rising edge
 * the timer count is saved as the start of the pulse and the edge detect
 * is switched to falling. On a falling edge the pulse width is calculated
 * from the difference, the edge detect switches back to rising, and the
 * data ready flag is set for the application to consume.
 */
ISR(TIMER1_CAPT_vect) {
    uint16_t captured = ICR1;
    if (TCCR1B & (1 << ICES1)) {
        // Rising edge -- record period since last rising edge, switch to
        // falling
        period = captured - risingEdgeTime;
        risingEdgeTime = captured;
        TCCR1B &= ~(1 << ICES1);
    } else {
        // Falling edge -- calculate pulse width, switch back to rising
        pulseWidth = captured - risingEdgeTime;
        TCCR1B |= (1 << ICES1);
        newDataReady = 1;
    }
}
