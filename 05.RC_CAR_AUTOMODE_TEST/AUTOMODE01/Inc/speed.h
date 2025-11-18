/*
 * speed.h
 *
 *  Created on: Oct 29, 2025
 *      Author: user13
 */

#ifndef INC_SPEED_H_
#define INC_SPEED_H_

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "stdio.h"

#define PWM_TIM        htim3
#define PWM_CH_RIGHT   TIM_CHANNEL_1
#define PWM_CH_LEFT    TIM_CHANNEL_2

void motor_speedInit();
void motor_setDuty(float dutyR, float dutyL);
void motor_speedUp();
void motor_speedDown();
void motor_right_speedUp();  // 좌회전(오 0.8, 왼 0.2)
void motor_left_speedUp();   // 우회전(오 0.2, 왼 0.8)
void motor_recover();
void pwm_sweep_test();

void automode_motor_speedInit();
void automode_motor_setDuty(float dutyR, float dutyL);

#endif /* INC_SPEED_H_ */
