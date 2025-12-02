/*
 * speed.c — safer tuning for wall avoidance
 * - Up/Down 비대칭 슬루 (UP 작게, DOWN 크게)
 * - 최저/기본/최대 속도 현실화
 * - 좌/우 보정이 내부 상태에 누적되도록 수정
 * - 모든 변경은 clamp 후 HAL로 적용
 */

/* speed.c - Refactored by The Code Craftsman */

#include "speed.h"
#include "robot_driver.h" // RobotCar_t 정의 포함
#include <stdio.h> // printf용

// ==== TUNING ====
// (매크로 상수는 그대로 유지)
#define SPEED_MIN       390u
#define SPEED_BASE      390u
#define SPEED_MAX       780u
#define STEP_UP          40u
#define STEP_DOWN       150u
#define STEP_DIFF        50u

// ==== 유틸 ====
static inline uint16_t clamp16(uint16_t v)
{
    if (v < SPEED_MIN) return SPEED_MIN;
    if (v > SPEED_MAX) return SPEED_MAX;
    return v;
}

static inline void apply_pwm(RobotCar_t *car)
{
    __HAL_TIM_SET_COMPARE(car->timer, car->ch_right, car->speed_right);
    __HAL_TIM_SET_COMPARE(car->timer, car->ch_left,  car->speed_left);
}

// ===== API 구현 =====

void motor_speedInit(RobotCar_t *car)
{
    uint16_t current_R = (uint16_t)__HAL_TIM_GET_COMPARE(car->timer, car->ch_right);
    uint16_t current_L = (uint16_t)__HAL_TIM_GET_COMPARE(car->timer, car->ch_left);

    // 구조체 변수 초기화
    car->speed_right = clamp16(current_R);
    car->speed_left  = clamp16(current_L);

    // 범위 벗어나면 리셋
    if (car->speed_right < SPEED_MIN || car->speed_right > SPEED_MAX) car->speed_right = SPEED_BASE;
    if (car->speed_left  < SPEED_MIN || car->speed_left  > SPEED_MAX) car->speed_left  = SPEED_BASE;

    apply_pwm(car);
}

void motor_speedUp(RobotCar_t *car)
{
    // 전역 변수 대신 car->speed_... 사용
    car->speed_right = clamp16((uint16_t)(car->speed_right + STEP_UP));
    car->speed_left  = clamp16((uint16_t)(car->speed_left  + STEP_UP));
    apply_pwm(car);
}

void motor_speedDown(RobotCar_t *car)
{
    car->speed_right = (car->speed_right > SPEED_MIN + STEP_DOWN)
                     ? (uint16_t)(car->speed_right - STEP_DOWN)
                     : SPEED_MIN;

    car->speed_left  = (car->speed_left  > SPEED_MIN + STEP_DOWN)
                     ? (uint16_t)(car->speed_left  - STEP_DOWN)
                     : SPEED_MIN;
    apply_pwm(car);
}

void motor_left_speedUp(RobotCar_t *car)
{
    car->speed_left  = clamp16((uint16_t)(car->speed_left  + STEP_DIFF));

    car->speed_right = (car->speed_right > SPEED_MIN + STEP_DIFF/2)
                     ? (uint16_t)(car->speed_right - STEP_DIFF/2)
                     : SPEED_MIN;
    apply_pwm(car);
}

void motor_right_speedUp(RobotCar_t *car)
{
    car->speed_right = clamp16((uint16_t)(car->speed_right + STEP_DIFF));

    car->speed_left  = (car->speed_left > SPEED_MIN + STEP_DIFF/2)
                     ? (uint16_t)(car->speed_left - STEP_DIFF/2)
                     : SPEED_MIN;
    apply_pwm(car);
}

void motor_recover(RobotCar_t *car)
{
    // 현재 구조체에 저장된 값을 다시 하드웨어에 씀
    apply_pwm(car);
}

void pwm_sweep_test(RobotCar_t *car)
{
    uint32_t val_R = __HAL_TIM_GET_COMPARE(car->timer, car->ch_right);
    uint32_t val_L = __HAL_TIM_GET_COMPARE(car->timer, car->ch_left);

    printf("CCR_R=%lu  CCR_L=%lu (Stored: R=%u L=%u)\r\n",
            (unsigned long)val_R, (unsigned long)val_L,
            car->speed_right, car->speed_left);

    HAL_Delay(500);
}

// ... (AutoMode 함수들도 위와 똑같이 car->speed_... 로 바꾸면 되네) ...
// ===== AutoMode에서 호출하는 표준 API =====

void auto_motor_speedInit(RobotCar_t *car)
{
  car->speed_right = clamp16(SPEED_BASE);
  car->speed_left  = clamp16(SPEED_BASE);

  apply_pwm(car);
}

void auto_motor_speedUp(RobotCar_t *car)
{
    // 평상시 가속은 항상 완만하게
	car->speed_right = clamp16((uint16_t)(car->speed_right + STEP_UP));
	car->speed_left  = clamp16((uint16_t)(car->speed_left  + STEP_UP));
  apply_pwm(car);
}

void auto_motor_speedDown(RobotCar_t *car)
{
    // 급접근 상황에서 반복 호출하면 짧은 시간에 강하게 느려짐
	car->speed_right = (car->speed_right > SPEED_MIN + STEP_DOWN)
                    ? (uint16_t)(car->speed_right - STEP_DOWN)
                    : SPEED_MIN;
	car->speed_left  = (car->speed_left  > SPEED_MIN + STEP_DOWN)
                    ? (uint16_t)(car->speed_left  - STEP_DOWN)
                    : SPEED_MIN;
  apply_pwm(car);
}

void auto_motor_left_speedUp(RobotCar_t *car)
{
	car->speed_left = clamp16((uint16_t)(car->speed_left + STEP_DIFF));
  apply_pwm(car);
}

void auto_motor_right_speedUp(RobotCar_t *car)
{
	car->speed_right = clamp16((uint16_t)(car->speed_right + STEP_DIFF));
  apply_pwm(car);
}

void auto_motor_left_speedDown(RobotCar_t *car)
{
	car->speed_left = (car->speed_left > SPEED_MIN + STEP_DIFF)
                   ? (uint16_t)(car->speed_left - STEP_DIFF)
                   : SPEED_MIN;
  apply_pwm(car);
}

void auto_motor_right_speedDown(RobotCar_t *car)
{
	car->speed_right = (car->speed_right > SPEED_MIN + STEP_DIFF)
                    ? (uint16_t)(car->speed_right - STEP_DIFF)
                    : SPEED_MIN;
  apply_pwm(car);
}

void auto_motor_slow(RobotCar_t *car)
{
    // 코너·충돌가드 직전 등 “확실히 느려야” 할 때
	car->speed_right = clamp16(300);
	car->speed_left  = clamp16(300);
  apply_pwm(car);
}
