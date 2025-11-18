/*
 * move.c
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */

#include "move.h"

// 오른쪽: IN1/IN2, 왼쪽: IN3/IN4
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

void motor_init()
{
    // 초기에는 모두 Low (코스트 상태 권장)
    HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, GPIO_PIN_RESET);
}

void motor_forward()
{
    right_dir_forward();
    left_dir_forward();
}

void motor_backward()
{
    right_dir_backward();
    left_dir_backward();
}

// 제자리 우회전(오른쪽 뒤, 왼쪽 앞) / 좌회전(반대)
void motor_rightRotation()
{
    right_dir_backward();
    left_dir_forward();
}
void motor_leftRotation()
{
    right_dir_forward();
    left_dir_backward();
}

void motor_stop()
{
    // 코스트(EN=0에 해당)로 두려면 여기서는 방향핀 Low, PWM=0은 speed.c에서
    HAL_GPIO_WritePin(IN1_GPIO_PORT, IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_PORT, IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_PORT, IN3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_PORT, IN4_PIN, GPIO_PIN_RESET);
}
