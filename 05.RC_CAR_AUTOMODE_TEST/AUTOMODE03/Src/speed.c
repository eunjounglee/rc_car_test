/*
 * speed.c
 *
 *  Created on: Oct 29, 2025
 *      Author: user13
 */


#include "speed.h"

#define SPEED_MIN    400u
#define SPEED_BASE   400u
#define SPEED_MAX    800u
#define SPEED_STEP    50u

extern TIM_HandleTypeDef htim3; // HAL 사용 시

static uint16_t rightMotorSpeed = SPEED_BASE;
static uint16_t leftMotorSpeed  = SPEED_BASE;

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

void motor_left_speedUp()
{
	uint16_t motor_goRight_rightMotorSpeed;
	uint16_t motor_goRight_leftMotorSpeed;

	motor_goRight_rightMotorSpeed = rightMotorSpeed;
	motor_goRight_leftMotorSpeed = leftMotorSpeed;

	motor_goRight_rightMotorSpeed -= 300;
	motor_goRight_leftMotorSpeed += 300;

	TIM3 -> CCR1 = motor_goRight_rightMotorSpeed;
	TIM3 -> CCR2 = motor_goRight_leftMotorSpeed;
}

void motor_right_speedUp()
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


// automode 함수
static inline uint16_t clamp16(uint16_t v)
{
    if (v < SPEED_MIN) return SPEED_MIN;
    if (v > SPEED_MAX) return SPEED_MAX;
    return v;
}

static inline void apply_pwm(void)
{
    // HAL 사용 권장
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, rightMotorSpeed); // CCR1
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, leftMotorSpeed);  // CCR2
    // 직접 접근도 가능: TIM3->CCR1 = rightMotorSpeed; TIM3->CCR2 = leftMotorSpeed;
}

// === 외부 노출 함수들 ===
void auto_motor_speedInit(void)
{
    rightMotorSpeed = leftMotorSpeed = clamp16(SPEED_BASE);
    apply_pwm();
}

void auto_motor_speedUp(void)
{
    rightMotorSpeed = clamp16((uint16_t)(rightMotorSpeed + SPEED_STEP));
    leftMotorSpeed  = clamp16((uint16_t)(leftMotorSpeed  + SPEED_STEP));
    apply_pwm();
}

void auto_motor_speedDown(void)
{
    // 최소값 밑으로 내려가지 않게 개별 클램프
    rightMotorSpeed = (rightMotorSpeed > SPEED_MIN + SPEED_STEP)
                    ? (uint16_t)(rightMotorSpeed - SPEED_STEP)
                    : SPEED_MIN;
    leftMotorSpeed  = (leftMotorSpeed  > SPEED_MIN + SPEED_STEP)
                    ? (uint16_t)(leftMotorSpeed  - SPEED_STEP)
                    : SPEED_MIN;
    apply_pwm();
}

void auto_motor_left_speedUp(void)
{
    leftMotorSpeed = clamp16((uint16_t)(leftMotorSpeed + SPEED_STEP));
    apply_pwm();
}

void auto_motor_right_speedUp(void)
{
    rightMotorSpeed = clamp16((uint16_t)(rightMotorSpeed + SPEED_STEP));
    apply_pwm();
}

void auto_motor_left_speedDown(void)
{
    leftMotorSpeed = (leftMotorSpeed > SPEED_MIN + SPEED_STEP)
                   ? (uint16_t)(leftMotorSpeed - SPEED_STEP)
                   : SPEED_MIN;
    apply_pwm();
}

void auto_motor_right_speedDown(void)
{
    rightMotorSpeed = (rightMotorSpeed > SPEED_MIN + SPEED_STEP)
                    ? (uint16_t)(rightMotorSpeed - SPEED_STEP)
                    : SPEED_MIN;
    apply_pwm();
}
