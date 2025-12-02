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
#include "robot_driver.h"


void motor_speedInit(RobotCar_t *car);
void motor_speedUp(RobotCar_t *car);
void motor_speedDown(RobotCar_t *car);
void motor_left_speedUp(RobotCar_t *car);
void motor_right_speedUp(RobotCar_t *car);
void motor_recover(RobotCar_t *car);
void pwm_sweep_test(RobotCar_t *car);

void auto_motor_speedInit(RobotCar_t *car);
void auto_motor_speedUp(RobotCar_t *car);
void auto_motor_speedDown(RobotCar_t *car);
void auto_motor_left_speedUp(RobotCar_t *car);
void auto_motor_right_speedUp(RobotCar_t *car);
void auto_motor_left_speedDown(RobotCar_t *car);
void auto_motor_right_speedDown(RobotCar_t *car);
void auto_motor_slow(RobotCar_t *car);

#endif /* INC_SPEED_H_ */
