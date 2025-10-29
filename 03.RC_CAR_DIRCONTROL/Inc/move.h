/*
 * move.h
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include "main.h"

#define IN1_PIN  	GPIO_PIN_4
#define IN1_GPIO_PORT  	GPIOA
#define IN2_PIN  	GPIO_PIN_0
#define IN2_GPIO_PORT  	GPIOB
#define IN3_PIN  	GPIO_PIN_1
#define IN3_GPIO_PORT  	GPIOC
#define IN4_PIN  	GPIO_PIN_0
#define IN4_GPIO_PORT  	GPIOC

void motor_init();
void motor_go();
void motor_back();
void motor_right();
void motor_left();
void motor_stop();

#endif /* INC_MOVE_H_ */
