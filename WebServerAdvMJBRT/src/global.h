#pragma once
#include <Arduino.h>

// PINS SETUP
const int32_t PIN_PID_LOOP = 17;
const int32_t PIN_ENC_A = 26;
const int32_t PIN_ENC_B = 27;

const int32_t PIN_MOTOR_INA = 18;
const int32_t PIN_MOTOR_INB = 19;
const int32_t PIN_MOTOR_CTRL = 23;

const int32_t PIN_LIMIT_SW = 25;


// PWM SETUP
const int32_t PWM_CH = 0;                             // 0 - 7(15?)  = 8(16?) channels
const int32_t PWM_FREQ_HZ = 19500;                    // 1Hz to 40MHz
const int32_t PWM_RES_BITS = 12;                      // 1 to 16(20) bits
const int32_t PWM_MIN = 0;                            // minimum pwm value
const int32_t PWM_MAX = pow(2, PWM_RES_BITS) - 1;     // maximum pwm value

// PID SETUP
const double DT_S = 0.001;
const double MIN_CTRL_VALUE = -4000;
const double MAX_CTRL_VALUE = 4000;
