/*
 * Copyright (c) 2026 Jack Cairns
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define TESTING 1 // Remove for use with actual motors and not LEDs

#include "timers.h"
#ifdef TESTING
#include "serialio.h"
#include <stdio.h>
#endif
#include <stdint.h>

// AVR Headers
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>

/* ========================================================================== */
/* Pin Definitions                                                            */
/* ========================================================================== */

#define MOTOR_NEUTRAL PD6                   // Motor common / star point
#define A_EMF_EN()    (ADMUX = (0 << MUX0)) // 0x00, PC0 -- Phase A back-EMF
#define B_EMF_EN()    (ADMUX = (1 << MUX0)) // 0x01, PC1 -- Phase B back-EMF
#define C_EMF_EN()    (ADMUX = (1 << MUX1)) // 0x02, PC2 -- Phase C back-EMF
#define A_HIGH_PIN    PD0                   // A gate high side
#define B_HIGH_PIN    PD1                   // B gate high side
#define C_HIGH_PIN    PD2                   // C gate high side
#define A_LOW_PIN     PC3                   // A gate low side
#define B_LOW_PIN     PC4                   // B gate low side
#define C_LOW_PIN     PC5                   // C gate low side
#define SIGNAL_INPUT  PB0                   // Input speed PWM signal

/* ========================================================================== */
/* Macros                                                                     */
/* ========================================================================== */

#ifdef TESTING
// Set PC0 to PC5 as outputs for LED testing
#define SETUP_LEDS()                                                           \
    do {                                                                       \
        DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) |            \
                (1 << PC4) | (1 << PC5);                                       \
        PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) |         \
                   (1 << PC4) | (1 << PC5));                                   \
    } while (0)
#else
// Set low side GPIO pins as outputs, all initially off
#define SETUP_LOW_SIDE()                                                       \
    do {                                                                       \
        DDRC |= (1 << A_LOW_PIN) | (1 << B_LOW_PIN) | (1 << C_LOW_PIN);        \
        PORTC &= ~((1 << A_LOW_PIN) | (1 << B_LOW_PIN) | (1 << C_LOW_PIN));    \
    } while (0)

// Set / Reset the pin -- only one low side active at a time
#define SET_A_LOW_PHASE()                                                      \
    (PORTC =                                                                   \
         (PORTC & ~((1 << B_LOW_PIN) | (1 << C_LOW_PIN))) | (1 << A_LOW_PIN))
#define SET_B_LOW_PHASE()                                                      \
    (PORTC =                                                                   \
         (PORTC & ~((1 << A_LOW_PIN) | (1 << C_LOW_PIN))) | (1 << B_LOW_PIN))
#define SET_C_LOW_PHASE()                                                      \
    (PORTC =                                                                   \
         (PORTC & ~((1 << A_LOW_PIN) | (1 << B_LOW_PIN))) | (1 << C_LOW_PIN))
#define RESET_A_LOW_PHASE() (PORTC &= ~(1 << A_LOW_PIN))
#define RESET_B_LOW_PHASE() (PORTC &= ~(1 << B_LOW_PIN))
#define RESET_C_LOW_PHASE() (PORTC &= ~(1 << C_LOW_PIN))

// Connect / Disconnect the pin to PWM -- only one high side active at a time
#define SET_A_HIGH_PHASE()                                                     \
    (DDRD = (DDRD & ~((1 << B_HIGH_PIN) | (1 << C_HIGH_PIN))) |                \
            (1 << A_HIGH_PIN))
#define SET_B_HIGH_PHASE()                                                     \
    (DDRD = (DDRD & ~((1 << A_HIGH_PIN) | (1 << C_HIGH_PIN))) |                \
            (1 << B_HIGH_PIN))
#define SET_C_HIGH_PHASE()                                                     \
    (DDRD = (DDRD & ~((1 << A_HIGH_PIN) | (1 << B_HIGH_PIN))) |                \
            (1 << C_HIGH_PIN))
#define RESET_A_HIGH_PHASE() (DDRD &= ~(1 << A_HIGH_PIN))
#define RESET_B_HIGH_PHASE() (DDRD &= ~(1 << B_HIGH_PIN))
#define RESET_C_HIGH_PHASE() (DDRD &= ~(1 << C_HIGH_PIN))
#endif

/* ========================================================================== */
/* Function Prototypes                                                        */
/* ========================================================================== */

void shutdown(void);
void phase0(void);
void phase1(void);
void phase2(void);
void phase3(void);
void phase4(void);
void phase5(void);
void run_phase(void);
void toggle_phase(void);
void comparatorInit(void);

/* ========================================================================== */
/* State Variables                                                            */
/* ========================================================================== */

static uint8_t phase = 0;
static volatile uint8_t emf = 0;

/* ========================================================================== */
/* Main                                                                       */
/* ========================================================================== */

/* main()
 * ------
 * Entry point. Initialises the system tick, input capture, and peripheral
 * setup depending on build mode. In TESTING mode the serial port and LED
 * PWM are initialised and phases are stepped on a 1 second timer. In normal
 * mode the gate driver PWM, low side GPIO, and analog comparator are
 * initialised and the motor is driven from back-EMF zero crossings.
 */
int main(void) {
    sei();
    initSystemTick();
    initTimer1_inputCapture();

#ifdef TESTING
    initSerialComs();
    initTimer2_led_pwm();
    SETUP_LEDS();
    printf_P(PSTR("System ready!\n"));
    uint32_t prev = getSysTick();
#else
    SETUP_LOW_SIDE();
    initTimer_gate_pwm();
    comparatorInit();
    phase0();
#endif

    while (1) {
        if (pwmSignalLost()) {
            shutdown();
            continue;
        }

#ifdef TESTING
        if (getSysTick() - prev > 1000) {
            toggle_phase();
            printf_P(PSTR("PW: %uus Throttle: %u\n"), getPulseWidth(),
                     getThrottle());
            printf_P(PSTR("Phase: %d\n"), phase + 1);
            prev = getSysTick();
        }

        set_led_duty(getThrottle());
        run_phase();
#else
        // Update gate PWM if throttle input is available
        if (pwmDataReady()) {
            set_gate_duty(getThrottle());
        }

        // Has a zero crossing occurred from the back-EMF
        if (emf) {
            run_phase(); // Will progress to next phase
        }
#endif
    }

    return 0;
}

/* ========================================================================== */
/* Safety                                                                     */
/* ========================================================================== */

/* shutdown()
 * ----------
 * Safely stops all motor drive outputs and resets commutation state.
 * In TESTING mode all LED outputs are cleared. In normal mode all high
 * side PWM outputs are disconnected from the timer, all low side GPIOs
 * are driven low, and the gate duty is set to zero. The emf and phase
 * flags are reset so the motor can be restarted cleanly from phase 0
 * once the PWM signal is restored.
 */
void shutdown(void) {
#ifdef TESTING
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) |
               (1 << PC5));
#else
    RESET_A_HIGH_PHASE();
    RESET_B_HIGH_PHASE();
    RESET_C_HIGH_PHASE();
    RESET_A_LOW_PHASE();
    RESET_B_LOW_PHASE();
    RESET_C_LOW_PHASE();
    set_gate_duty(0);
    clearPwmDataReady();
#endif
    emf = 0;
    phase = 0;
}

/* ========================================================================== */
/* Commutation Phases                                                         */
/* ========================================================================== */

/* phase0()
 * --------
 * Commutation step 0: A High, B Low, C Floating.
 * In TESTING mode drives LEDs to visualise the step.
 * In normal mode connects A high side PWM, enables B low side GPIO,
 * and selects C phase back-EMF on the comparator mux.
 */
void phase0(void) {
#ifdef TESTING
    PORTC &= ~((1 << PC1) | (1 << PC2) | (1 << PC4) | (1 << PC5));
    PORTC |= (1 << PC0) | (1 << PC3);
#else
    SET_A_HIGH_PHASE();
    SET_B_LOW_PHASE();
    C_EMF_EN();
#endif
}

/* phase1()
 * --------
 * Commutation step 1: A High, C Low, B Floating.
 * In TESTING mode drives LEDs to visualise the step.
 * In normal mode connects A high side PWM, enables C low side GPIO,
 * and selects B phase back-EMF on the comparator mux.
 */
void phase1(void) {
#ifdef TESTING
    PORTC &= ~((1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4));
    PORTC |= (1 << PC0) | (1 << PC5);
#else
    SET_A_HIGH_PHASE();
    SET_C_LOW_PHASE();
    B_EMF_EN();
#endif
}

/* phase2()
 * --------
 * Commutation step 2: B High, C Low, A Floating.
 * In TESTING mode drives LEDs to visualise the step.
 * In normal mode connects B high side PWM, enables C low side GPIO,
 * and selects A phase back-EMF on the comparator mux.
 */
void phase2(void) {
#ifdef TESTING
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC3) | (1 << PC4));
    PORTC |= (1 << PC2) | (1 << PC5);
#else
    SET_B_HIGH_PHASE();
    SET_C_LOW_PHASE();
    A_EMF_EN();
#endif
}

/* phase3()
 * --------
 * Commutation step 3: B High, A Low, C Floating.
 * In TESTING mode drives LEDs to visualise the step.
 * In normal mode connects B high side PWM, enables A low side GPIO,
 * and selects C phase back-EMF on the comparator mux.
 */
void phase3(void) {
#ifdef TESTING
    PORTC &= ~((1 << PC0) | (1 << PC3) | (1 << PC4) | (1 << PC5));
    PORTC |= (1 << PC2) | (1 << PC1);
#else
    SET_B_HIGH_PHASE();
    SET_A_LOW_PHASE();
    C_EMF_EN();
#endif
}

/* phase4()
 * --------
 * Commutation step 4: C High, A Low, B Floating.
 * In TESTING mode drives LEDs to visualise the step.
 * In normal mode connects C high side PWM, enables A low side GPIO,
 * and selects B phase back-EMF on the comparator mux.
 */
void phase4(void) {
#ifdef TESTING
    PORTC &= ~((1 << PC0) | (1 << PC2) | (1 << PC3) | (1 << PC5));
    PORTC |= (1 << PC4) | (1 << PC1);
#else
    SET_C_HIGH_PHASE();
    SET_A_LOW_PHASE();
    B_EMF_EN();
#endif
}

/* phase5()
 * --------
 * Commutation step 5: C High, B Low, A Floating.
 * In TESTING mode drives LEDs to visualise the step.
 * In normal mode connects C high side PWM, enables B low side GPIO,
 * and selects A phase back-EMF on the comparator mux.
 */
void phase5(void) {
#ifdef TESTING
    PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC5));
    PORTC |= (1 << PC4) | (1 << PC3);
#else
    SET_C_HIGH_PHASE();
    SET_B_LOW_PHASE();
    A_EMF_EN();
#endif
}

/* ========================================================================== */
/* Commutation Control                                                        */
/* ========================================================================== */

/* toggle_phase()
 * --------------
 * Advances the commutation phase counter by one step wrapping
 * back to 0 after step 5.
 */
void toggle_phase(void) {
    phase = (phase + 1) % 6;
}

/* run_phase()
 * -----------
 * Advances to the next commutation step and drives the appropriate
 * outputs for that step. In normal mode toggle_phase() is called here
 * on each back-EMF zero crossing. Clears the emf flag after switching
 * so the next zero crossing can be detected.
 */
void run_phase(void) {
#ifndef TESTING
    toggle_phase();
#endif
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

    emf = 0;
}

/* ========================================================================== */
/* Comparator                                                                 */
/* ========================================================================== */

/* comparatorInit()
 * ----------------
 * Configures the analog comparator for back-EMF zero crossing detection.
 * AIN0 (PD6) is the positive input connected to the motor neutral point.
 * The ADC mux is connected to the comparator negative input via ACME,
 * allowing PC0/PC1/PC2 (ADC0/1/2) to be selected per commutation step.
 * Interrupt fires on output toggle to catch zero crossings in both
 * directions. ADC is disabled to allow the mux to be used by the
 * comparator. Starts monitoring phase C (floating in step 0).
 */
void comparatorInit(void) {
    ADCSRB = (1 << ACME);   // Connect ADC mux to comparator negative input
    ADCSRA &= ~(1 << ADEN); // Disable ADC so mux is available to comparator
    ACSR = (1 << ACIE);     // Enable comparator interrupt on output toggle
    // ACIS1:ACIS0 = 0:0 -- interrupt on both edges

    DDRD &= ~(1 << MOTOR_NEUTRAL); // PD6 (AIN0) as input -- motor neutral point
    // PC0-2 as inputs -- back-EMF
    DDRC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2));

    C_EMF_EN(); // Phase 0 has C floating so start monitoring C
}

/* ========================================================================== */
/* ISR Functions                                                              */
/* ========================================================================== */

/* ISR(ANALOG_COMP_vect)
 * ---------------------
 * Fired on each back-EMF zero crossing detected by the analog comparator.
 * Sets the emf flag for the main loop to advance the commutation step.
 * Kept minimal to avoid re-triggering from switching noise.
 */
ISR(ANALOG_COMP_vect) {
    emf = 1;
}
