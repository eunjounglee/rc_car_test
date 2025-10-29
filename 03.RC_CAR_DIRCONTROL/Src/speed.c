/*
 * speed.c
 *
 *  Created on: Oct 29, 2025
 *      Author: user13
 */


#include "speed.h"

uint16_t rightMotorSpeed;
uint16_t leftMotorSpeed;

void motor_speedInit()
{
	rightMotorSpeed = TIM3->CCR1;
	leftMotorSpeed = TIM3->CCR2;
}

void motor_speedUp()
{
	if(rightMotorSpeed < 900 && leftMotorSpeed < 900)
	{
		rightMotorSpeed += 100;
		leftMotorSpeed += 100;
	}
	else
	{
		return;
	}
	TIM3 -> CCR1 = rightMotorSpeed;
	TIM3 -> CCR2 = leftMotorSpeed;
}

void motor_speedDown()
{
	if(rightMotorSpeed > 500 && leftMotorSpeed > 500)
	{
		rightMotorSpeed -= 100;
		leftMotorSpeed -= 100;
	}
	else
	{
		return;
	}
	TIM3 -> CCR1 = rightMotorSpeed;
	TIM3 -> CCR2 = leftMotorSpeed;
}

void motor_goRight()
{
	uint16_t motor_goRight_rightMotorSpeed;
	uint16_t motor_goRight_leftMotorSpeed;

	motor_goRight_rightMotorSpeed = rightMotorSpeed;
	motor_goRight_leftMotorSpeed = leftMotorSpeed;

	motor_goRight_rightMotorSpeed -= 200;
	motor_goRight_leftMotorSpeed += 200;

	TIM3 -> CCR1 = motor_goRight_rightMotorSpeed;
	TIM3 -> CCR2 = motor_goRight_leftMotorSpeed;
}

void motor_goLeft()
{
	uint16_t motor_goLeft_rightMotorSpeed;
	uint16_t motor_goLeft_leftMotorSpeed;

	motor_goLeft_rightMotorSpeed = rightMotorSpeed;
	motor_goLeft_leftMotorSpeed = leftMotorSpeed;

	motor_goLeft_rightMotorSpeed += 200;
	motor_goLeft_leftMotorSpeed -= 200;

	TIM3 -> CCR1 = motor_goLeft_rightMotorSpeed;
	TIM3 -> CCR2 = motor_goLeft_leftMotorSpeed;
}

void motor_recover()
{
	TIM3 -> CCR1 = rightMotorSpeed;
	TIM3 -> CCR2 = leftMotorSpeed;
}

void pwm_sweep_test()
{
   printf("CCR1=%lu  CCR2=%lu\r\n", TIM3 -> CCR1, TIM3 -> CCR2);
   HAL_Delay(500);
}
