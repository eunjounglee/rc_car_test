/*
 * speed.c
 *
 *  Created on: Oct 29, 2025
 *      Author: user13
 */


#include "speed.h"

void motor_speedInit()
{
	TIM3 -> CCR1 = 700; // 70% duty
}

void motor_speedUp()
{
	TIM3 -> CCR1 = 900; // 90% duty
}

void motor_speedDown()
{
	TIM3 -> CCR1 = 500; // 50% duty
}

void pwm_sweep_test()
{
   printf("CCR1=%lu\r\n", TIM3 -> CCR1);
   HAL_Delay(500);
}
