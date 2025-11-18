/*
 * speed.c
 *
 *  Created on: Oct 29, 2025
 *      Author: user13
 */


#include "speed.h"
#include "math.h"
#include "speed.h"
#include "move.h"


static inline uint32_t period_of_local(TIM_HandleTypeDef *ht) {
    return __HAL_TIM_GET_AUTORELOAD(ht); // htim->Instance->ARR 읽는 매크로
}

// 실수를 특정 범위로 제한하는 함수
static inline float limitFloat(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

// 현재 기본 듀티(복구용)
static float baseDutyR = 0.6f; // 초기값: 적당히 60% 정도(네 환경에 맞게)
static float baseDutyL = 0.6f;

// 내부: 듀티 0~1 → CCR로 기록
static void set_pwm(float dutyR, float dutyL)
{
    TIM_TypeDef* tim = TIM3;                  // ENA/ENB가 TIM3_CH1/CH2
    uint32_t arr = tim->ARR;                  // 자동재장전값
    uint32_t ccr1 = (uint32_t)llroundf(limitFloat(dutyR, 0.0f, 1.0f) * (arr + 1));
    uint32_t ccr2 = (uint32_t)llroundf(limitFloat(dutyL, 0.0f, 1.0f) * (arr + 1));
    tim->CCR1 = ccr1;
    tim->CCR2 = ccr2;
}

// 외부 API 1: 기준속도 저장/초기화
void motor_speedInit()
{
    // 타이머 현재 CCR을 읽어 초기 기준 듀티로 삼거나, 고정값으로 세팅
    uint32_t arr = TIM3->ARR;
    baseDutyR = (float)TIM3->CCR1 / (float)(arr + 1);
    baseDutyL = (float)TIM3->CCR2 / (float)(arr + 1);
    // 초기값이 0이면 기본 0.6으로
    if (baseDutyR <= 0.01f) baseDutyR = 0.6f;
    if (baseDutyL <= 0.01f) baseDutyL = 0.6f;
    set_pwm(baseDutyR, baseDutyL);
}

// 외부 API 2: 정규화 듀티 직접 설정(0~1). 방향은 move.c 쪽 상태를 그대로 사용.
void motor_setDuty(float dutyR, float dutyL)
{
    set_pwm(dutyR, dutyL);
}

// 외부 API 3: 동일 증가/감소 (정규화)
void motor_speedUp()
{
    baseDutyR = limitFloat(baseDutyR + 0.1f, 0.0f, 1.0f);
    baseDutyL = limitFloat(baseDutyL + 0.1f, 0.0f, 1.0f);
    set_pwm(baseDutyR, baseDutyL);
}
void motor_speedDown()
{
    baseDutyR = limitFloat(baseDutyR - 0.1f, 0.0f, 1.0f);
    baseDutyL = limitFloat(baseDutyL - 0.1f, 0.0f, 1.0f);
    set_pwm(baseDutyR, baseDutyL);
}

void motor_right_speedUp(void)
{
    set_pwm(0.8f, 0.2f);
}

void motor_left_speedUp(void)
{
    set_pwm(0.2f, 0.8f);
}

// 외부 API 5: 복구(기준 듀티로 되돌림)
void motor_recover()
{
    set_pwm(baseDutyR, baseDutyL);
}

// 디버깅
void pwm_sweep_test(void)
{
    printf("CCR1=%lu  CCR2=%lu\r\n", TIM3->CCR1, TIM3->CCR2);
    HAL_Delay(500);
}

void automode_motor_speedInit()
{
  // 초기 0%
  __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_RIGHT, 0);
  __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_LEFT,  0);
}

void automode_motor_setDuty(float dutyR, float dutyL)
{
  float uR = limitFloat(dutyR, 0.0f, 1.0f);
  float uL = limitFloat(dutyL, 0.0f, 1.0f);
  uint32_t arr = period_of_local(&PWM_TIM);
  __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_RIGHT, (uint32_t)lroundf(uR * (float)arr));
  __HAL_TIM_SET_COMPARE(&PWM_TIM, PWM_CH_LEFT,  (uint32_t)lroundf(uL * (float)arr));
}
