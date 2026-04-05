#include "serialio.h"
#include "timers.h"

#include <stdint.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

int main(void) {
    // Start timers
    initSystemTick();
    initTimer1_inputCapture();
    initTimer2_pwm();

    // Start Serial
    initSerialComs();
    sei();
    printf_P(PSTR("System ready!\n"));
    uint32_t prev = getSysTicks();
    while (1) {
        // printf_P(PSTR("Counter: %u\n"), counter++);
        if (pwmDataReady() && getSysTicks() - prev > 1000) {
            printf_P(PSTR("PW: %uus  Period: %uus  Freq: %uHz  Duty: %u  "
                          "Throttle: %u\n"),
                     getPulseWidth(), getPeriod(), getFrequency(),
                     getDutyPerMille(), getThrottlePerMille());
            prev = getSysTicks();
        }

        // Update LED pwm to match throttle
        set_led_duty(getThrottlePerMille());
    }

    return 0;
}

// Need to setup 3 pins for PWM

// 3 pins will be straight on/off

// 3 pins for the resistor COM network

// 1 pin for the COM point

// 1 pin for speed input

// A B C

//  1     2     3     4     5     6
// AB -> AC -> BC -> BA -> CA -> CB


