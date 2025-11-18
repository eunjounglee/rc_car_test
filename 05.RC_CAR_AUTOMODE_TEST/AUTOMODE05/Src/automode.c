// ======= SIMPLE: 앞막힘 → 90도 회전 → 직진 (노이즈 완화 패치: 컴파일/경고 수정) =======

#include "tim.h"
#include "ultrasonic.h"
#include "stdint.h"
#include "stdbool.h"
#include "move.h"
#include "speed.h"

extern TIM_HandleTypeDef htim3;

// (필요 시) 아래 extern은 move.h에 프로토타입이 없다면 임시로 사용하세요.
// extern void motor_forward(void);
// extern void left_dir_forward(void);
// extern void left_dir_backward(void);
// extern void right_dir_forward(void);
// extern void right_dir_backward(void);

// --- 간단 튜닝값 ---
#define CRUISE_PWM_SIMPLE     360
#define PIVOT_PWM_SIMPLE      360
#define TURN_90_MS_SIMPLE     300

// [PATCH] 노이즈/안전 보강
#define NO_TURN_HOLD_MS        200
#define EMA_SHIFT                3
#define PWM_RAMP_STEP           37

#define MARGIN_CM                4
#define SAFE_FRONT_CM           40
#define CHOICE_STREAK            2

// --- 유틸 ---
static inline uint16_t get_pwm_max_simple(void) {
    return __HAL_TIM_GET_AUTORELOAD(&htim3);
}
static inline uint16_t clamp_pwm_simple(int32_t v) {
    if (v < 0) return 0;
    uint16_t m = get_pwm_max_simple();
    if (v > m) return m;
    return (uint16_t)v;
}

// 램프 적용 세터
static inline void set_speed_ramped_simple(int32_t right_pwm, int32_t left_pwm) {
    static int32_t s_pr = 0, s_pl = 0;
    int32_t tr = clamp_pwm_simple(right_pwm);
    int32_t tl = clamp_pwm_simple(left_pwm);

    int32_t dr = tr - s_pr;
    int32_t dl = tl - s_pl;
    if (dr >  PWM_RAMP_STEP) dr =  PWM_RAMP_STEP;
    if (dr < -PWM_RAMP_STEP) dr = -PWM_RAMP_STEP;
    if (dl >  PWM_RAMP_STEP) dl =  PWM_RAMP_STEP;
    if (dl < -PWM_RAMP_STEP) dl = -PWM_RAMP_STEP;

    s_pr += dr;
    s_pl += dl;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint16_t)s_pr); // R
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint16_t)s_pl); // L
}

static inline bool time_left_simple(uint32_t now_ms, uint32_t deadline_ms) {
    return (int32_t)(deadline_ms - now_ms) > 0;
}

// --- 상태 ---
typedef enum { ST_DRIVE = 0, ST_TURN } simple_state_t;
typedef enum { TURN_LEFT = -1, TURN_RIGHT = 1 } simple_turn_dir_t;
typedef enum { CH_FWD = 0, CH_LEFT, CH_RIGHT } choice_t;

static simple_state_t    s_state_simple    = ST_DRIVE;
static simple_turn_dir_t s_turn_dir_simple = TURN_RIGHT;
static uint32_t          s_deadline_simple = 0;

static choice_t  s_last_choice = CH_FWD;
static uint8_t   s_choice_streak = 0;

// 노이즈/안전 상태 변수
static uint16_t s_front_ema = 0;          // 전방 EMA(cm)
static uint32_t s_no_turn_until = 0;      // 재진입 홀드 종료 시각(ms)

void AutoMode_Init(void) {
    s_state_simple     = ST_DRIVE;
    s_turn_dir_simple  = TURN_RIGHT;
    s_deadline_simple  = 0;

    s_front_ema        = 0;
    s_no_turn_until    = 0;

    motor_forward();
    set_speed_ramped_simple(CRUISE_PWM_SIMPLE, CRUISE_PWM_SIMPLE);
}

void AutoMode_Update(void)
{
    const uint32_t now = HAL_GetTick();

    // 1) 센서 원시값
    const uint16_t left_cm_raw  = US_Left_cm();
    const uint16_t right_cm_raw = US_Right_cm();
    const uint16_t front_cm_raw = US_Center_cm();

    // 2) 전방 EMA 업데이트
    if (s_front_ema == 0) {
        s_front_ema = front_cm_raw;  // 초기 시드
    } else {
        s_front_ema = (uint16_t)(((uint32_t)s_front_ema * ((1u << EMA_SHIFT) - 1) + front_cm_raw) >> EMA_SHIFT);
    }

    // 3) 이 프레임에서 실제로 사용할 값(컴파일 에러 원인 제거)
    const uint16_t left_cm  = left_cm_raw;
    const uint16_t right_cm = right_cm_raw;
    const uint16_t front_cm = s_front_ema;    // 전방은 EMA 적용

    switch (s_state_simple)
    {
    case ST_TURN:
    {
        // 90도 회전 유지
        if (!time_left_simple(now, s_deadline_simple)) {
            s_state_simple   = ST_DRIVE;
            s_no_turn_until  = now + NO_TURN_HOLD_MS;   // 재진입 홀드
            motor_forward();
            set_speed_ramped_simple(CRUISE_PWM_SIMPLE, CRUISE_PWM_SIMPLE);
            break;
        }

        // 정지 피벗
        if (s_turn_dir_simple == TURN_LEFT) {
            left_dir_backward();
            right_dir_forward();
        } else {
            left_dir_forward();
            right_dir_backward();
        }
        set_speed_ramped_simple(PIVOT_PWM_SIMPLE, PIVOT_PWM_SIMPLE);
    } break;

    case ST_DRIVE:
    default:
    {
        // 재진입 홀드 동안은 회전 판단 금지(경고 억제 + 안정성)
        if ((int32_t)(s_no_turn_until - now) > 0) {
            motor_forward();
            set_speed_ramped_simple(CRUISE_PWM_SIMPLE, CRUISE_PWM_SIMPLE);
            break;
        }

        // 최대값 후보 계산 (+마진, +전방 안전)
        const bool front_ok = (front_cm >= SAFE_FRONT_CM);

        const bool front_is_max = front_ok &&
            (front_cm >= (uint16_t)(left_cm  + MARGIN_CM)) &&
            (front_cm >= (uint16_t)(right_cm + MARGIN_CM));

        const bool left_is_max =
            (left_cm  >= (uint16_t)(right_cm + MARGIN_CM)) &&
            (left_cm  >= (uint16_t)(front_cm + MARGIN_CM));

        const bool right_is_max =
            (right_cm >= (uint16_t)(left_cm  + MARGIN_CM)) &&
            (right_cm >= (uint16_t)(front_cm + MARGIN_CM));

        // 애매하면 직전 선택 유지(전방이 너무 가깝다면 더 먼 쪽)
        choice_t choice = s_last_choice;
        if (front_is_max)      choice = CH_FWD;
        else if (left_is_max)  choice = CH_LEFT;
        else if (right_is_max) choice = CH_RIGHT;
        else {
            if (front_ok && (front_cm + 1 >= left_cm) && (front_cm + 1 >= right_cm))
            {
                choice = CH_FWD;
            } else
            {
                choice = (left_cm >= right_cm) ? CH_LEFT : CH_RIGHT;
            }
        }

        // 선택 연속성
        if (choice == s_last_choice) s_choice_streak++;
        else { s_last_choice = choice; s_choice_streak = 1; }

        if (s_choice_streak < CHOICE_STREAK) {
            if (front_ok) {
                motor_forward();
                set_speed_ramped_simple(CRUISE_PWM_SIMPLE, CRUISE_PWM_SIMPLE);
            } else {
                set_speed_ramped_simple(0, 0);
            }
            break;
        }

        // 동작
        if (choice == CH_FWD) {
            motor_forward();
            set_speed_ramped_simple(CRUISE_PWM_SIMPLE, CRUISE_PWM_SIMPLE);
        } else {
            s_turn_dir_simple = (choice == CH_LEFT) ? TURN_LEFT : TURN_RIGHT;
            set_speed_ramped_simple(0, 0);
            s_deadline_simple = now + TURN_90_MS_SIMPLE;
            s_state_simple    = ST_TURN;
            s_choice_streak   = 0;
        }
    } break;
    }
}
// ======= /SIMPLE (patched) =======
