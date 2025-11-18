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

void motor_speedInit();
void motor_speedUp();
void motor_speedDown();
void motor_left_speedUp();
void motor_right_speedUp();
void motor_recover();
void pwm_sweep_test();

void auto_motor_speedInit();
void auto_motor_speedUp();
void auto_motor_speedDown();
void auto_motor_left_speedUp();
void auto_motor_right_speedUp();
void auto_motor_left_speedDown();
void auto_motor_right_speedDown();
void auto_motor_slow();

#endif /* INC_SPEED_H_ */
