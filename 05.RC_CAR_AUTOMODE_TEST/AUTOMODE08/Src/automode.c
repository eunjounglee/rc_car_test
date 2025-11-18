#include <stdint.h>
#include <stdbool.h>
#include "move.h"
#include "ultrasonic.h"
#include "speed.h"

/* ---------------------------------------------------------------------------
 * 튜닝 상수 (충돌가드 관련 상수 제거됨)
 *  - WALL_NEAR_CM    : 전/좌/우 중 하나라도 이 값 이하이면 감속
 *  - FRONT_STOP_CM   : 전방이 이 값 이하이면 회전 준비(DECEL_FOR_*)
 *  - DECEL_TICKS_FOR_TURN : 회전 직전 감속 유지 시간(틱)
 *  - PIVOT_TICKS          : 회전 유지 시간 상한(세이프가드)
 * ---------------------------------------------------------------------------*/

enum {
  WALL_NEAR_CM          = 65,
  FRONT_STOP_CM         = 50,

  DECEL_TICKS_FOR_TURN  = 12,
  PIVOT_TICKS           = 29
};
// --- 중앙 정렬 종료용 상수 추가 ---
enum {
  FRONT_CLEAR_CM   = 50,  // 전방이 이 정도는 열려 있어야 안전
  CENTER_TOL_CM    = 10,   // L≈R 허용 밴드 (처음 6~10 cm 권장)
  CENTER_STREAK_N  = 3,   // 연속 판정 회수 (노이즈 억제)
  CENTER_BIAS_CM   = 0    // 좌우 센서 바이어스 보정값(현장 캘리브레이션)
};

/* ---------------------------------------------------------------------------
 * 상태 정의 (충돌가드 상태 전부 삭제됨)
 * ---------------------------------------------------------------------------*/
typedef enum {
    ST_STRAIGHT = 0,
    ST_DECEL_FOR_RIGHT,
    ST_DECEL_FOR_LEFT,
    ST_TURN_RIGHT,
    ST_TURN_LEFT
} am_state_t;

/* ------------------------------
 * 내부 상태 변수
 * ------------------------------*/
static am_state_t s_state = ST_STRAIGHT;
static int s_ticks = 0;                  // 상태별로 재사용되는 틱 카운터
static uint16_t s_last_front = 1000;     // 디버깅용(로직 영향 없음)

// 직진 P-조향용(현재 D는 미사용)
static int16_t s_err_prev = 0;

static int s_center_streak = 0;

// --- 좌/우 센터 판정 공통 함수 ---
static inline int16_t center_diff_l_minus_r(uint16_t left_cm, uint16_t right_cm)
{
    // L - R - BIAS  (BIAS는 "좌 센서가 항상 +BIAS만큼 크게 나온다"일 때 양수로 세팅)
    return (int16_t)left_cm - (int16_t)right_cm - (int16_t)CENTER_BIAS_CM;
}

static inline bool is_centered(uint16_t left_cm, uint16_t right_cm)
{
    int16_t d = center_diff_l_minus_r(left_cm, right_cm);
    return (d >= -CENTER_TOL_CM) && (d <= CENTER_TOL_CM);
}

/* ---------------------------------------------------------------------------
 * 초기화
 * ---------------------------------------------------------------------------*/
void AutoMode_Init(void)
{
    auto_motor_speedInit();
    s_state = ST_STRAIGHT;
    s_ticks = 0;
    s_last_front = 1000;

    left_dir_forward();
    right_dir_forward();
}

/* ---------------------------------------------------------------------------
 * 메인 업데이트 루프 (주기 10~20ms 가정)
 * 1) 센서 샘플링
 * 2) 상태머신 (직진 / 회전 준비 / 회전)
 *  - 충돌가드(급정지/후진/피벗) 관련 분기는 전부 제거됨
 * ---------------------------------------------------------------------------*/
void AutoMode_Update(void)
{
    // 1) 초음파 거리(cm) 샘플
    const uint16_t left_cm   = US_Left_cm();
    const uint16_t right_cm  = US_Right_cm();
    const uint16_t front_cm  = US_Center_cm();

    // 2) 상태머신
    switch (s_state)
    {
    case ST_STRAIGHT:
    default:
    {
        // 진행방향 고정(혹시 이전 상태가 후진/역방향이었다면 바로 교정)
        left_dir_forward();
        right_dir_forward();

        // (a) 전/좌/우 중 하나라도 가까우면 감속, 아니면 가속
        if (front_cm <= WALL_NEAR_CM || left_cm <= WALL_NEAR_CM || right_cm <= WALL_NEAR_CM)
            auto_motor_speedDown();
        else
            auto_motor_speedUp();

        // (b) P-조향: err = right - left
        const int16_t DEAD  = 2;
        const int16_t CLAMP = 10;
        int16_t err = (int16_t)right_cm - (int16_t)left_cm;
        int16_t u = 0;

        if (err > DEAD)       u = +(err - DEAD);
        else if (err < -DEAD) u = -( -err - DEAD );
        if (u >  CLAMP) u =  CLAMP;
        if (u < -CLAMP) u = -CLAMP;

        if (u > 0) {  // 오른쪽이 더 멂 → 오른쪽으로 틀기(왼↑/오↓)
            auto_motor_left_speedUp();
            auto_motor_right_speedDown();
        } else if (u < 0) { // 왼쪽이 더 멂 → 왼쪽으로 틀기(오↑/왼↓)
            auto_motor_right_speedUp();
            auto_motor_left_speedDown();
        }
        s_err_prev = err;

        // (c) 전방 임계 진입 시 회전 준비
        if (front_cm <= FRONT_STOP_CM) {
            s_ticks = 0;
            if      (right_cm > left_cm) s_state = ST_DECEL_FOR_RIGHT;
            else if (right_cm < left_cm) s_state = ST_DECEL_FOR_LEFT;
            else                         s_state = ST_DECEL_FOR_RIGHT;
        }
    } break;

    case ST_DECEL_FOR_RIGHT:
    {
        // 회전 전 감속
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        // 감속 유지 뒤 우회전(좌전/우후 피벗) 진입
        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            left_dir_forward();
            right_dir_backward();
            s_state = ST_TURN_RIGHT;
            s_ticks = 0; // 회전 상한 카운트 분리
        }
    } break;

    case ST_DECEL_FOR_LEFT:
    {
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        // 감속 유지 뒤 좌회전(좌후/우전 피벗) 진입
        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            left_dir_backward();
            right_dir_forward();
            s_state = ST_TURN_LEFT;
            s_ticks = 0;
        }
    } break;

    case ST_TURN_RIGHT:
    {
        if (front_cm <= WALL_NEAR_CM) auto_motor_speedDown();
        else                          auto_motor_speedUp();

        // ▶ 통일된 중심 판정
        bool centered    = is_centered(left_cm, right_cm);
        bool front_clear = (front_cm >= FRONT_CLEAR_CM);

        if (centered && front_clear) s_center_streak++; else s_center_streak = 0;

        if (s_center_streak >= CENTER_STREAK_N || ++s_ticks >= PIVOT_TICKS) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
            s_ticks = 0;
            s_center_streak = 0;
        }
    } break;


    case ST_TURN_LEFT:
    {
        if (front_cm <= WALL_NEAR_CM) auto_motor_speedDown();
        else                          auto_motor_speedUp();

        // ▶ 통일된 중심 판정
        bool centered    = is_centered(left_cm, right_cm);
        bool front_clear = (front_cm >= FRONT_CLEAR_CM);

        if (centered && front_clear) s_center_streak++; else s_center_streak = 0;

        if (s_center_streak >= CENTER_STREAK_N || ++s_ticks >= PIVOT_TICKS) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
            s_ticks = 0;
            s_center_streak = 0;
        }
    } break;


    }

    // 디버깅 보조
    s_last_front = front_cm;
}
