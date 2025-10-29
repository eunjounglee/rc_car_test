/*
 * move.c
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */

#include "move.h"

void motor_init()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 0);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 0);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 0);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 0);
}

void motor_go()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 1);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 0);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 1);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 0);
}

void motor_back()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 0);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 1);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 0);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 1);
}

void motor_right()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 0);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 0);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 1);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 0);
}

void motor_laft()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 1);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 0);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 0);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 0);
}

void motor_stop()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 0);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 0);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 0);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 0);
}
