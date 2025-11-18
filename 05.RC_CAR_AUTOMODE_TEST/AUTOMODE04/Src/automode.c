/*
 * automode.c — Front distance → 전진 + 정지회전(후진 없음)
 *  - FreeRTOS API 미사용 (HAL_GetTick만 사용)
 *  - 상태: ST_CLEAR(전진), ST_TURNING(정지 피벗 회전)
 *  - 히스테리시스 + 최소 회전 시간 + 재진입 홀드(no-turn hold)
 *
 *  필요 외부 심볼:
 *    - extern TIM_HandleTypeDef htim3;
 *    - motor_init(), motor_forward()
 *    - left_dir_forward(), left_dir_backward()
 *    - right_dir_forward(), right_dir_backward()
 *    - US_Left_cm(), US_Right_cm(), US_Center_cm()  // 미디안 필터된 값
 */

#include "automode.h"
#include "ultrasonic.h"
#include "tim.h"
#include "stdint.h"
#include "stdbool.h"

// ===== 튜닝 상수 =====
#define D_TURN_ENTER        40   // 이 값 이하 → 회전 진입
#define D_TURN_EXIT         46   // 이 값 이상 → 회전 종료(히스테리시스 상한)
#define TURN_MS            170   // 회전 최소 유지시간(ms)

// 사이드/정렬 튜닝
#define SIDE_CLEAR_CM       30   // 옆길이 "열림"으로 보이는 임계
#define SIDE_BAL_DB          8   // |L-R| ≤ DB → 좌우 균형(정렬)

// 회전 종료 추가 조건
#define FRONT_STRONG_EXIT   51   // 전방이 이보다 확실히 크면 최소시간 무시하고 즉시 종료
#define FRONT_EXIT_STREAK    2   // 전방 종료조건 연속 프레임 수
#define SIDE_BAL_STREAK      2   // 좌우 균형 종료조건 연속 프레임 수

// 전진 속도(느리게)
#define CRUISE_PWM         350   // 전진 기본 PWM

// 피벗 회전 PWM (바깥/안쪽 차등)
#define PIVOT_PWM_OUT      380   // 더멀어진 바퀴(바깥쪽)
#define PIVOT_PWM_IN       360   // 안쪽 바퀴

// 회전 종료 직후, 곧바로 재진입되는 것을 막는 홀드(ms)
#define NO_TURN_HOLD_MS    120

extern TIM_HandleTypeDef htim3;

// --- PWM 유틸 ---
static inline uint16_t get_pwm_max(void)
{
    return __HAL_TIM_GET_AUTORELOAD(&htim3);  // 예: 999 또는 1000
}

static inline uint16_t clamp_pwm(int32_t pwm_value)
{
    if (pwm_value < 0) return 0;
    uint16_t max = get_pwm_max();
    if (pwm_value > max) return max;
    return (uint16_t)pwm_value;
}

// 안전한 PWM 설정 (R,L)
static inline void set_speed(int32_t right_pwm, int32_t left_pwm)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, clamp_pwm(right_pwm)); // 오른쪽
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, clamp_pwm(left_pwm));  // 왼쪽
}

// --- 상태 ---
typedef enum { ST_CLEAR = 0, ST_TURNING } auto_state_t;
static auto_state_t s_state = ST_CLEAR;

// --- 회전 방향 보존 ---
typedef enum { TURN_LEFT = -1, TURN_NONE = 0, TURN_RIGHT = 1 } turn_dir_t;
static turn_dir_t s_turn_dir = TURN_NONE;

// --- 타이밍 ---
static uint32_t s_deadline_ms   = 0;  // 회전 최소 유지 종료 시각
static uint32_t s_no_turn_until = 0;  // 회전 종료 후 재진입 홀드 종료 시각

// --- 종료 조건 카운터 ---
static uint8_t s_front_clear_cnt = 0;
static uint8_t s_side_bal_cnt    = 0;

// 남은 시간(언더플로 안전)
static inline bool time_left(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(deadline_ms - now_ms) > 0;
}

void AutoMode_Init(void)
{
    motor_init();             // 방향핀/EN/PWM start 포함 가정
    s_state           = ST_CLEAR;
    s_turn_dir        = TURN_NONE;
    s_deadline_ms     = 0;
    s_no_turn_until   = 0;
    s_front_clear_cnt = 0;
    s_side_bal_cnt    = 0;
}

void AutoMode_Update(void)
{
    const uint32_t now = HAL_GetTick();

    // 필터된 거리 값
    const uint16_t left_cm  = US_Left_cm();
    const uint16_t right_cm = US_Right_cm();
    const uint16_t front_cm = US_Center_cm();

    switch (s_state)
    {
    // =========================
    // ST_TURNING — 정지 피벗
    // =========================
    case ST_TURNING:
    {
        // ---- 회전 종료 후보 판정 ----
        // 1) 전방이 충분히 열림(히스테리시스 상한 충족 프레임 카운트)
        if (front_cm >= D_TURN_EXIT) s_front_clear_cnt++; else s_front_clear_cnt = 0;

        // 2) 좌우가 균형 상태(정렬) + 전방도 어느정도는 열림
        int16_t diff  = (int16_t)left_cm - (int16_t)right_cm;
        int16_t adiff = (diff >= 0) ? diff : -diff;
        if (adiff <= SIDE_BAL_DB && front_cm >= D_TURN_ENTER) s_side_bal_cnt++; else s_side_bal_cnt = 0;

        // ---- 즉시 종료 조건 ----
        const bool min_time_ok = !time_left(now, s_deadline_ms);  // 최소 회전시간 충족
        const bool front_ok    = (s_front_clear_cnt >= FRONT_EXIT_STREAK);
        const bool side_ok     = (s_side_bal_cnt   >= SIDE_BAL_STREAK);
        const bool strong_exit = (front_cm >= FRONT_STRONG_EXIT); // 아주 넉넉히 열림

        if (strong_exit || side_ok || (front_ok && min_time_ok)) {
            // [STATE TRANSITION] ST_TURNING -> ST_CLEAR
            s_state           = ST_CLEAR;
            s_turn_dir        = TURN_NONE;
            s_no_turn_until   = now + NO_TURN_HOLD_MS;  // 재진입 홀드
            s_front_clear_cnt = 0;
            s_side_bal_cnt    = 0;
            break;
        }

        // ---- 피벗 회전 (방향 고정) ----
        // 회전에 들어올 때 s_turn_dir가 정해져 있어야 함
        if (s_turn_dir == TURN_LEFT) {
            // 반시계(좌 회전): 좌-뒤 / 우-앞
            left_dir_backward();
            right_dir_forward();
            set_speed(PIVOT_PWM_OUT, PIVOT_PWM_IN);   // (R,L)
        } else { // TURN_RIGHT 또는 TURN_NONE
            // 시계(우 회전): 좌-앞 / 우-뒤
            left_dir_forward();
            right_dir_backward();
            set_speed(PIVOT_PWM_IN, PIVOT_PWM_OUT);
        }
    } break;

    // =========================
    // ST_CLEAR — 전진
    // =========================
    case ST_CLEAR:
    default:
    {
        // [STATE TRANSITION CANDIDATE] ST_CLEAR -> ST_TURNING
        // 재진입 홀드가 끝났는지 확인한 뒤, 전방 부족 시 회전 진입
        if (!time_left(now, s_no_turn_until)) {
            if (front_cm <= D_TURN_ENTER) {
                // 코너 선택: 한쪽만 “열림”이면 그쪽으로 회전 방향 고정
                const bool left_open  = (left_cm  >= SIDE_CLEAR_CM);
                const bool right_open = (right_cm >= SIDE_CLEAR_CM);

                if (left_open ^ right_open) {
                    s_turn_dir = left_open ? TURN_LEFT : TURN_RIGHT;
                } else {
                    // 둘 다 열렸거나 둘 다 막혔으면 더 먼 쪽 선택
                    s_turn_dir = (left_cm >= right_cm) ? TURN_LEFT : TURN_RIGHT;
                }

                // [STATE TRANSITION] ST_CLEAR -> ST_TURNING
                s_state           = ST_TURNING;
                s_deadline_ms     = now + TURN_MS;  // 최소 회전 시간 세팅
                s_front_clear_cnt = 0;
                s_side_bal_cnt    = 0;
                break;
            }
        }

        // 전진(느리게)
        motor_forward();
        set_speed(CRUISE_PWM, CRUISE_PWM);
    } break;
    }
}
