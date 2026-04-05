/*
 * Copyright (c) 2026 Jack Cairns
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TIMERS_H
#define TIMERS_H

#include <stdint.h>

////////////////////////////// Init Functions /////////////////////////////////

void initSystemTick(void);
void initTimer1_inputCapture(void);
void initTimer2_led_pwm(void);
void initTimer3_get_pwm(void);

////////////////////////////// Disable Functions //////////////////////////////

void disableTimers(void);

////////////////////////////// Duty Setters ///////////////////////////////////

void set_led_duty(uint16_t throttlePerMille);
void set_gate_duty(uint16_t throttlePerMille);

////////////////////////////// Input Capture Getters //////////////////////////

uint8_t pwmDataReady(void);
void clearPwmDataReady(void);
uint16_t getPulseWidth(void);
uint16_t getPeriod(void);
uint16_t getFrequency(void);
uint16_t getRawDutyPerMille(void);
uint16_t getThrottlePerMille(void);

////////////////////////////// System Tick Getters ////////////////////////////

uint32_t getSysTick(void);

#endif
