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

void motor_forward()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 1);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 1);
}

void motor_backward()
{
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 1);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 1);
}

void motor_rightRotaion()
{
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 1);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 1);
}

void motor_leftRotaion()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 1);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 1);
}

void right_dir_forward()
{
    HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, GPIO_PIN_RESET);
}

void right_dir_backward()
{
    HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, GPIO_PIN_SET);
}

void left_dir_forward()
{
    HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, GPIO_PIN_RESET);
}

void left_dir_backward()
{
    HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, GPIO_PIN_SET);
}

void motor_stop()
{
	HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, 0);
	HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, 0);
	HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, 0);
	HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, 0);
}

void drive_forward()
{
    right_dir_forward();
    left_dir_forward();
}

void pivot_right()
{
    right_dir_backward();
    left_dir_forward();
}

void pivot_left()
{
    right_dir_forward();
    left_dir_backward();
}
