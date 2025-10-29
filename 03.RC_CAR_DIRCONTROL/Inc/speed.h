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
void motor_goRight();
void motor_goLeft();
void motor_recover();
void pwm_sweep_test();

#endif /* INC_SPEED_H_ */
