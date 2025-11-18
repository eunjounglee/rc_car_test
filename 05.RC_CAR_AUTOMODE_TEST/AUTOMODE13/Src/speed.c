/*
 * speed.c — safer tuning for wall avoidance
 * - Up/Down 비대칭 슬루 (UP 작게, DOWN 크게)
 * - 최저/기본/최대 속도 현실화
 * - 좌/우 보정이 내부 상태에 누적되도록 수정
 * - 모든 변경은 clamp 후 HAL로 적용
 */

#include "speed.h"
#include "tim.h"       // __HAL_TIM_SET_COMPARE 사용 시
#include <stdio.h>

// ==== TUNING (필드에서 조정) ====
#define SPEED_MIN       390u   // 이전 400 → 저속 코너링 확보
#define SPEED_BASE      390u   // 이전 400
#define SPEED_CRUISE    520u   // 목표 순항 근처(옵션)
#define SPEED_MAX       780u   // 이전 800

#define STEP_UP          40u   // 가속은 작게 (관성 축적 억제)
#define STEP_DOWN       150u   // 감속은 크게 (벽 근접 시 민첩)
#define STEP_DIFF        50u   // 좌/우 미세 보정 1회량 (과하지 않게)

extern TIM_HandleTypeDef htim3; // TIM3 CH1=Right, CH2=Left (보드에 맞게)

// ==== 내부 상태 ====
static uint16_t rightMotorSpeed = SPEED_BASE;
static uint16_t leftMotorSpeed  = SPEED_BASE;

// ==== 유틸 ====
static inline uint16_t clamp16(uint16_t v)
{
    if (v < SPEED_MIN) return SPEED_MIN;
    if (v > SPEED_MAX) return SPEED_MAX;
    return v;
}

static inline void apply_pwm(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, rightMotorSpeed); // CCR1 (Right)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, leftMotorSpeed);  // CCR2 (Left)
}

// ===== (레거시) motor_* API =====
// 필요하면 유지하되 내부 상태를 일관되게 업데이트하도록 수정

void motor_speedInit(void)
{
    // 타이머 현재값을 베이스로 삼고, 비정상 범위면 안전값으로 보정
    rightMotorSpeed = clamp16((uint16_t)TIM3->CCR1);
    leftMotorSpeed  = clamp16((uint16_t)TIM3->CCR2);
    if (rightMotorSpeed < SPEED_MIN || rightMotorSpeed > SPEED_MAX) rightMotorSpeed = SPEED_BASE;
    if (leftMotorSpeed  < SPEED_MIN || leftMotorSpeed  > SPEED_MAX) leftMotorSpeed  = SPEED_BASE;
    apply_pwm();
}

void motor_speedUp(void)
{
    rightMotorSpeed = clamp16((uint16_t)(rightMotorSpeed + STEP_UP));
    leftMotorSpeed  = clamp16((uint16_t)(leftMotorSpeed  + STEP_UP));
    apply_pwm();
}

void motor_speedDown(void)
{
    // 감속은 강하게. 너무 낮아지지 않도록 클램프
    rightMotorSpeed = (rightMotorSpeed > SPEED_MIN + STEP_DOWN)
                    ? (uint16_t)(rightMotorSpeed - STEP_DOWN)
                    : SPEED_MIN;
    leftMotorSpeed  = (leftMotorSpeed  > SPEED_MIN + STEP_DOWN)
                    ? (uint16_t)(leftMotorSpeed  - STEP_DOWN)
                    : SPEED_MIN;
    apply_pwm();
}

void motor_left_speedUp(void)
{
    // 좌 바퀴 가속, 우 바퀴 약감속(코너 바깥바퀴/안쪽바퀴 느낌)
    leftMotorSpeed  = clamp16((uint16_t)(leftMotorSpeed  + STEP_DIFF));
    rightMotorSpeed = (rightMotorSpeed > SPEED_MIN + STEP_DIFF/2)
                    ? (uint16_t)(rightMotorSpeed - STEP_DIFF/2)
                    : SPEED_MIN;
    apply_pwm();
}

void motor_right_speedUp(void)
{
    rightMotorSpeed = clamp16((uint16_t)(rightMotorSpeed + STEP_DIFF));
    leftMotorSpeed  = (leftMotorSpeed > SPEED_MIN + STEP_DIFF/2)
                    ? (uint16_t)(leftMotorSpeed - STEP_DIFF/2)
                    : SPEED_MIN;
    apply_pwm();
}

void motor_recover(void)
{
    // 내부 상태를 그대로 재적용 (이 함수가 “원상복귀” 의미라면 BASE로 복귀하도록 바꿔도 됨)
    apply_pwm();
}

void pwm_sweep_test(void)
{
    printf("CCR1=%lu  CCR2=%lu\r\n", (unsigned long)TIM3->CCR1, (unsigned long)TIM3->CCR2);
    HAL_Delay(500);
}

// ===== AutoMode에서 호출하는 표준 API =====

void auto_motor_speedInit(void)
{
    rightMotorSpeed = leftMotorSpeed = clamp16(SPEED_BASE);
    apply_pwm();
}

void auto_motor_speedUp(void)
{
    // 평상시 가속은 항상 완만하게
    rightMotorSpeed = clamp16((uint16_t)(rightMotorSpeed + STEP_UP));
    leftMotorSpeed  = clamp16((uint16_t)(leftMotorSpeed  + STEP_UP));
    apply_pwm();
}

void auto_motor_speedDown(void)
{
    // 급접근 상황에서 반복 호출하면 짧은 시간에 강하게 느려짐
    rightMotorSpeed = (rightMotorSpeed > SPEED_MIN + STEP_DOWN)
                    ? (uint16_t)(rightMotorSpeed - STEP_DOWN)
                    : SPEED_MIN;
    leftMotorSpeed  = (leftMotorSpeed  > SPEED_MIN + STEP_DOWN)
                    ? (uint16_t)(leftMotorSpeed  - STEP_DOWN)
                    : SPEED_MIN;
    apply_pwm();
}

void auto_motor_left_speedUp(void)
{
    leftMotorSpeed = clamp16((uint16_t)(leftMotorSpeed + STEP_DIFF));
    apply_pwm();
}

void auto_motor_right_speedUp(void)
{
    rightMotorSpeed = clamp16((uint16_t)(rightMotorSpeed + STEP_DIFF));
    apply_pwm();
}

void auto_motor_left_speedDown(void)
{
    leftMotorSpeed = (leftMotorSpeed > SPEED_MIN + STEP_DIFF)
                   ? (uint16_t)(leftMotorSpeed - STEP_DIFF)
                   : SPEED_MIN;
    apply_pwm();
}

void auto_motor_right_speedDown(void)
{
    rightMotorSpeed = (rightMotorSpeed > SPEED_MIN + STEP_DIFF)
                    ? (uint16_t)(rightMotorSpeed - STEP_DIFF)
                    : SPEED_MIN;
    apply_pwm();
}

void auto_motor_slow(void)
{
    // 코너·충돌가드 직전 등 “확실히 느려야” 할 때
    rightMotorSpeed = clamp16(300);
    leftMotorSpeed  = clamp16(300);
    apply_pwm();
}
