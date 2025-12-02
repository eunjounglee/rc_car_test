/*
 * robot_driver.h
 *
 *  Created on: Nov 30, 2025
 *      Author: user13
 */

#ifndef INC_ROBOT_DRIVER_H_
#define INC_ROBOT_DRIVER_H_

#include "main.h"

typedef struct
{
    // [공통 심장] PWM 타이머 핸들
    TIM_HandleTypeDef *timer;

    // 속도
    uint16_t speed_left;
    uint16_t speed_right;

    // [왼쪽휠]
    uint32_t      ch_left;      // PWM 채널
    GPIO_TypeDef *port_left_1;  // IN1 Port
    uint16_t      pin_left_1;   // IN1 Pin
    GPIO_TypeDef *port_left_2;  // IN2 Port
    uint16_t      pin_left_2;   // IN2 Pin

    // [오른쪽휠]
    uint32_t      ch_right;     // PWM 채널
    GPIO_TypeDef *port_right_3; // IN3 Port
    uint16_t      pin_right_3;  // IN3 Pin
    GPIO_TypeDef *port_right_4; // IN4 Port
    uint16_t      pin_right_4;  // IN4 Pin

} RobotCar_t;

void Robot_SetSpeed(RobotCar_t *car, int left_speed, int right_speed);
void Robot_MoveForward(RobotCar_t *car);

#endif /* INC_ROBOT_DRIVER_H_ */
