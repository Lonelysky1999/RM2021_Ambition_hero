#ifndef SNAIL_MOTOR_H
#define SNAIL_MOTOR_H

#define SNAIL_START_PWM 1090
#define SNAIL_OFF_PWM   1090
#define SNAIL_MIN_PWM   1200
#define SNAIL_MAX_PWM   1400

#define SNAIL_MIN_SET_PWM   1090
#define SNAIL_MAX_SET_PWM   1900

#include "main.h"

void snail_init(void);
void snail_1_set(uint16_t cmd);
void snail_2_set(uint16_t cmd);
void snail_off(void);

#endif
