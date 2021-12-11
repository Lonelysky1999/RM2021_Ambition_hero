#include "snail_motor.h"
#include "timer.h"
#include "main.h"
#include "sys.h"
#include "user_lib.h"

void snail_init(void)
{
	TIM1_Init(2000,180);
	TIM_SetCompare1(TIM1,1090);
	TIM_SetCompare2(TIM1,1090);
	TIM_SetCompare3(TIM1,1090);
	TIM_SetCompare4(TIM1,1090);
}

void snail_1_set(uint16_t cmd)
{
	uint16_t pwm;
	pwm = int16_constrain(cmd,SNAIL_MIN_SET_PWM,SNAIL_MAX_SET_PWM);
	TIM_SetCompare1(TIM1,pwm);
}

void snail_2_set(uint16_t cmd)
{
	uint16_t pwm;
	pwm = int16_constrain(cmd,SNAIL_MIN_SET_PWM,SNAIL_MAX_SET_PWM);
	TIM_SetCompare4(TIM1,pwm);
}

void snail_off(void)
{
	TIM_SetCompare1(TIM8,SNAIL_OFF_PWM);
	TIM_SetCompare4(TIM8,SNAIL_OFF_PWM);
}

