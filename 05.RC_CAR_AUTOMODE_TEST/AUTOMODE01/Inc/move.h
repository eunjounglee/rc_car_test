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

void right_dir_forward();
void right_dir_backward();
void left_dir_forward();
void left_dir_backward();

void motor_init();
void motor_forward();
void motor_backward();
void motor_rightRotaion();
void motor_leftRotaion();
void motor_stop();

#endif /* INC_MOVE_H_ */
