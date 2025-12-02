/*
 * move.c
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */

#include "move.h"

void motor_init(RobotCar_t *car)
{
  // 왼쪽 바퀴 (IN1=L, IN2=L)
  HAL_GPIO_WritePin(car->port_left_1, car->pin_left_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(car->port_left_2, car->pin_left_2, GPIO_PIN_RESET);

  // 오른쪽 바퀴 (IN3=L, IN4=L)
  HAL_GPIO_WritePin(car->port_right_3, car->pin_right_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(car->port_right_4, car->pin_right_4, GPIO_PIN_RESET);
}

void motor_forward(RobotCar_t *car)
{
  HAL_GPIO_WritePin(car->port_left_1, car->pin_left_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(car->port_right_3, car->pin_right_3, GPIO_PIN_SET);
}

void motor_backward(RobotCar_t *car)
{
  HAL_GPIO_WritePin(car->port_left_2, car->pin_left_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(car->port_right_4, car->pin_right_4, GPIO_PIN_SET);
}

void motor_rightRotaion(RobotCar_t *car)
{
  HAL_GPIO_WritePin(car->port_left_2, car->pin_left_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(car->port_right_3, car->pin_right_3, GPIO_PIN_SET);
}

void motor_leftRotaion(RobotCar_t *car)
{
  HAL_GPIO_WritePin(car->port_left_1, car->pin_left_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(car->port_right_4, car->pin_right_4, GPIO_PIN_SET);
}

void right_dir_forward(RobotCar_t *car)
{
  HAL_GPIO_WritePin(car->port_left_1, car->pin_left_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(car->port_left_2, car->pin_left_2, GPIO_PIN_RESET);
}

void right_dir_backward(RobotCar_t *car)
{
  HAL_GPIO_WritePin(car->port_left_1, car->pin_left_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(car->port_left_2, car->pin_left_2, GPIO_PIN_SET);
}

void left_dir_forward(RobotCar_t *car)
{
  // 오른쪽 바퀴
  HAL_GPIO_WritePin(car->port_right_3, car->pin_right_3, GPIO_PIN_SET);
  HAL_GPIO_WritePin(car->port_right_4, car->pin_right_4, GPIO_PIN_RESET);
}

void left_dir_backward(RobotCar_t *car)
{
  HAL_GPIO_WritePin(car->port_right_3, car->pin_right_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(car->port_right_4, car->pin_right_4, GPIO_PIN_SET);
}

void motor_stop(RobotCar_t *car)
{
  // 왼쪽 바퀴 (IN1=L, IN2=L)
  HAL_GPIO_WritePin(car->port_left_1, car->pin_left_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(car->port_left_2, car->pin_left_2, GPIO_PIN_RESET);

  // 오른쪽 바퀴 (IN3=L, IN4=L)
  HAL_GPIO_WritePin(car->port_right_3, car->pin_right_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(car->port_right_4, car->pin_right_4, GPIO_PIN_RESET);
}

void drive_forward(RobotCar_t *car)
{
    right_dir_forward(car);
    left_dir_forward(car);
}

void drive_backward(RobotCar_t *car)
{
    right_dir_backward(car);
    left_dir_backward(car);
}

void pivot_right(RobotCar_t *car)
{
    right_dir_backward(car);
    left_dir_forward(car);
}

void pivot_left(RobotCar_t *car)
{
    right_dir_forward(car);
    left_dir_backward(car);
}
