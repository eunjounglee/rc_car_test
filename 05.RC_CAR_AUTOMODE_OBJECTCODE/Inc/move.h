/*
 * move.h
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */

#ifndef INC_MOVE_H_
#define INC_MOVE_H_

#include "main.h"
#include "robot_driver.h"

void motor_init(RobotCar_t *car);
void motor_forward(RobotCar_t *car);
void motor_backward(RobotCar_t *car);
void motor_rightRotaion(RobotCar_t *car);
void motor_leftRotaion(RobotCar_t *car);
void motor_stop(RobotCar_t *car);

void right_dir_forward(RobotCar_t *car);
void right_dir_backward(RobotCar_t *car);
void left_dir_forward(RobotCar_t *car);
void left_dir_backward(RobotCar_t *car);

void drive_forward(RobotCar_t *car);
void drive_backward(RobotCar_t *car);
void pivot_right(RobotCar_t *car);
void pivot_left(RobotCar_t *car);

#endif /* INC_MOVE_H_ */
