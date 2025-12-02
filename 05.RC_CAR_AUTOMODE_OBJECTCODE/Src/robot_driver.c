/*
 * robot_driver.c
 *
 *  Created on: Nov 30, 2025
 *      Author: user13
 */

// robot_driver.c (새로 만들 파일)

#include "robot_driver.h" // RobotCar_t 정의가 있는 곳

// 이 함수는 이제 '어떤 차'인지(handle)만 알면,
// 그 차가 TIM3를 쓰든 TIM4를 쓰든 상관없이 운전할 수 있음.
void Robot_SetSpeed(RobotCar_t *car, int left_speed, int right_speed)
{
    // 1. 구조체에서 '타이머 정보'를 꺼내 쓴다 (-> 화살표 연산자 주목!)
    __HAL_TIM_SET_COMPARE(car->timer, car->ch_left,  left_speed);
    __HAL_TIM_SET_COMPARE(car->timer, car->ch_right, right_speed);
}

void Robot_MoveForward(RobotCar_t *car)
{
    // 2. 구조체에서 'GPIO 정보'를 꺼내 쓴다
    // 왼쪽 바퀴 전진 (IN1=H, IN2=L)
    HAL_GPIO_WritePin(car->port_left_1, car->pin_left_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(car->port_left_2, car->pin_left_2, GPIO_PIN_RESET);

    // 오른쪽 바퀴 전진 (IN3=H, IN4=L)
    HAL_GPIO_WritePin(car->port_right_3, car->pin_right_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(car->port_right_4, car->pin_right_4, GPIO_PIN_RESET);
}
