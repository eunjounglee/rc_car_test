#include <stdint.h>
#include <stdbool.h>
#include "move.h"
#include "ultrasonic.h"
#include "speed.h"

// ===== 튜닝 상수 (필요시 조정) =====
enum {
  WALL_NEAR_CM          = 40,   // 40cm 이내 감속
  FRONT_STOP_CM         = 33,   // 30cm 이내 회전 절차
  RIGHT_REF_CM          = 15,   // 우측 10cm 기준으로 보정

  // --- 충돌 가드 관련 ---
  CRASH_CM              = 15,   // 전방이 12cm 이하면 "코박음"으로 간주
  CRASH_STREAK_N        = 2,    // 연속 N회 이하일 때만 발동 (노이즈 억제)
  CRASH_COOLDOWN_TICKS  = 20,   // 가드 발동 후 쿨다운

  DECEL_TICKS_FOR_TURN  = 7,    // 회전 전 감속 틱
  BACKOFF_TICKS         = 10,   // 후진 유지 틱
  PIVOT_TICKS           = 12    // 피벗 유지 틱
};

// ===== 상태 정의 =====
typedef enum {
    ST_STRAIGHT = 0,
    ST_DECEL_FOR_RIGHT,
    ST_DECEL_FOR_LEFT,
    ST_TURN_RIGHT,
    ST_TURN_LEFT,

    // --- 충돌 가드 전용 상태 ---
    ST_EMERG_BRAKE,       // 급감속(사실상 정지에 준함)
    ST_BACKOFF,           // 짧게 후진
    ST_PIVOT_AWAY_R,      // 우측으로 피벗(좌전/우후)
    ST_PIVOT_AWAY_L       // 좌측으로 피벗(좌후/우전)
} am_state_t;

static am_state_t s_state = ST_STRAIGHT;
static int s_ticks = 0;                 // 다목적 타이머 틱
static int s_crash_streak = 0;          // 연속 “매우 근접” 샘플 수
static int s_crash_cooldown = 0;        // 가드 재발동 쿨다운
static uint16_t s_last_front = 1000;    // 전 프레임 전방값(디버깅/확장용)

// ===== 초기화 =====
void AutoMode_Init(void)
{
    auto_motor_speedInit();
    s_state = ST_STRAIGHT;
    s_ticks = 0;
    s_crash_streak = 0;
    s_crash_cooldown = 0;
    s_last_front = 1000;

    left_dir_forward();
    right_dir_forward();
}

// ===== 충돌 가드: 조건 검사 헬퍼 =====
static inline bool crash_detected(uint16_t front_cm)
{
    // 전방이 아주 가까움(<= CRASH_CM) 상태가 연속 CRASH_STREAK_N회 유지되면 충돌로 간주
    if (front_cm <= CRASH_CM) {
        s_crash_streak++;
    } else {
        s_crash_streak = 0;
    }
    return (s_crash_streak >= CRASH_STREAK_N);
}

// ===== 메인 업데이트(주기 10~20ms 가정) =====
void AutoMode_Update(void)
{
    const uint16_t left_cm   = US_Left_cm();
    const uint16_t right_cm  = US_Right_cm();
    const uint16_t front_cm  = US_Center_cm();

    // 쿨다운 감소
    if (s_crash_cooldown > 0) s_crash_cooldown--;

    // 어느 상태든 “진짜 코앞”이면 가드 우선(단, 쿨다운 중엔 재발동 금지)
    if (s_crash_cooldown == 0 &&
        (s_state == ST_STRAIGHT || s_state == ST_TURN_LEFT || s_state == ST_TURN_RIGHT) &&
        crash_detected(front_cm))
    {
        // ① 급감속부터 시작
        s_state = ST_EMERG_BRAKE;
        s_ticks = 0;
    }

    switch (s_state)
    {
    case ST_STRAIGHT:
    default:
    {
        // 기본 직진
        left_dir_forward();
        right_dir_forward();

        // 40cm 이내 감속, 아니면 서서히 가속
        if (front_cm <= WALL_NEAR_CM || left_cm <= WALL_NEAR_CM || right_cm <= WALL_NEAR_CM) {
            auto_motor_speedDown();
        } else {
            auto_motor_speedUp();
        }

        // 우측 기준 보정
        if (right_cm > RIGHT_REF_CM) auto_motor_left_speedUp();
        else                         auto_motor_left_speedDown();

        // 전방 30cm 이내면 좌/우 회전 절차 진입
        if (front_cm <= FRONT_STOP_CM) {
            s_ticks = 0;
            if (right_cm > left_cm) s_state = ST_DECEL_FOR_RIGHT;
            else if (right_cm < left_cm) s_state = ST_DECEL_FOR_LEFT;
            else s_state = ST_DECEL_FOR_RIGHT;
        }
    } break;

    case ST_DECEL_FOR_RIGHT:
    {
        auto_motor_speedDown();
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            left_dir_forward();
            right_dir_backward();
            s_state = ST_TURN_RIGHT;
        }
    } break;

    case ST_DECEL_FOR_LEFT:
    {
        auto_motor_speedDown();
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            left_dir_backward();
            right_dir_forward();
            s_state = ST_TURN_LEFT;
        }
    } break;

    case ST_TURN_RIGHT:
    {
        // 안전 감속 룰 유지
        if (front_cm <= WALL_NEAR_CM) auto_motor_speedDown();
        else                          auto_motor_speedUp();

        // 종료 조건: front > left → 직진 복귀
        if (front_cm > left_cm) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
        }
    } break;

    case ST_TURN_LEFT:
    {
        if (front_cm <= WALL_NEAR_CM) auto_motor_speedDown();
        else                          auto_motor_speedUp();

        // 종료 조건: front > right → 직진 복귀
        if (front_cm > right_cm) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
        }
    } break;

    // ====== 충돌 가드 시퀀스 ======
    case ST_EMERG_BRAKE:
    {
        // ① 급감속(사실상 정지에 가깝게 몇 틱 유지)
        auto_motor_speedDown();
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            // ② 후진 시작(양쪽 후진)
            left_dir_backward();
            right_dir_backward();
            s_state = ST_BACKOFF;
            s_ticks = 0;
        }
    } break;

    case ST_BACKOFF:
    {
        // 후진 유지
        auto_motor_speedDown(); // 뒤로 빠지면서 속도 낮춤(안전)
        if (++s_ticks >= BACKOFF_TICKS) {
            // ③ 빈쪽으로 피벗: 더 먼 쪽으로 회전
            if (right_cm >= left_cm) {
                // 우측 여유가 크면 우측 피벗(좌전/우후)
                left_dir_forward();
                right_dir_backward();
                s_state = ST_PIVOT_AWAY_R;
            } else {
                // 좌측 여유가 크면 좌측 피벗(좌후/우전)
                left_dir_backward();
                right_dir_forward();
                s_state = ST_PIVOT_AWAY_L;
            }
            s_ticks = 0;
        }
    } break;

    case ST_PIVOT_AWAY_R:
    {
        // 피벗 유지
        auto_motor_speedUp(); // 너무 꺾이지 않게 필요한 만큼만 가속(환경에 따라 Down도 고려)
        // ④ 종료 조건: front가 좌측보다 커지거나, 충분히 돌았으면 복귀
        if (front_cm > left_cm || ++s_ticks >= PIVOT_TICKS) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
            s_crash_cooldown = CRASH_COOLDOWN_TICKS; // 재발동 쿨다운
            s_crash_streak = 0;
        }
    } break;

    case ST_PIVOT_AWAY_L:
    {
        auto_motor_speedUp();
        if (front_cm > right_cm || ++s_ticks >= PIVOT_TICKS) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
            s_crash_cooldown = CRASH_COOLDOWN_TICKS;
            s_crash_streak = 0;
        }
    } break;
    }

    s_last_front = front_cm;
}
