/*
 * automode.c — Minimal autonomous drive (No-Stop)
 *  - 후진 없음 / 정지 없음
 *  - Pivot(제자리 90°) + Arc(부드러운 커브)
 *  - 결정 주기 제한 / 상태 최소 유지 / 방향 스트릭(±2)
 *  - Arc 유지 중 바이어스 재적용 / 거리 기반 속도 거버너
 */

#include "automode.h"
#include "ultrasonic.h"
#include "speed.h"
#include "move.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

// ====== 튜닝 상수 ======
enum {
  // 코너 판단 임계 (히스테리시스)
  FRONT_PIVOT_CM   = 66,   // 이하면 Pivot
  FRONT_ARC_CM     = 81,   // 이하면 Arc
  FRONT_CLEAR_CM   = 67,   // 이 이상이면 다시 직진

  // 회전/커브 유지 시간
  TURN_MS          = 350,  // Pivot 유지 시간(약간 늘림: 과소회전 방지)
  ARC_MIN_MS       = 310,  // Arc 최소 유지(소폭 ↑)
  ARC_MAX_MS       = 900,

  // 안정화
  DECIDE_EVERY_MS  = 30,
  HOLD_DRIVE_MS    = 115,
  HOLD_TURN_MS     = 180,

  // 너무 가까움 임계 (정지 금지 정책: 즉시 Pivot)
  FRONT_TOO_CLOSE  = 12
};

// ====== 상태 정의 ======
typedef enum { ST_DRIVE, ST_TURN } state_t;
typedef enum { TURN_PIVOT, TURN_ARC } mode_t;
typedef enum { TURN_LEFT=-1, TURN_RIGHT=+1 } dir_t;

static state_t  s_state = ST_DRIVE;
static mode_t   s_mode;
static dir_t    s_dir;
static uint32_t s_deadline_ms = 0;

// 안정화용 보조 상태
static int8_t   dir_vote = 0;
static uint32_t s_next_decide_ms = 0;
static uint32_t s_hold_until_ms   = 0;

// === startup turn-ban (전역) ===
static uint32_t s_no_turn_until = 0;   // [STARTUP TURN BAN] 회전 금지 기한

void AutoMode_Start(void)
{
  const uint32_t now = HAL_GetTick();
  s_no_turn_until = now + 1000;        // [STARTUP TURN BAN] 시작 1.2s 턴 완전 금지
  dir_vote = 0;
  s_state = ST_DRIVE;
}

static inline bool valid_cm(uint16_t x){
  return (x >= 2 && x <= 300);               // 이상치 무시
}

// ====== 메인 업데이트 루프 ======
void AutoMode_Update(void)
{
  const uint32_t now = HAL_GetTick();

  // 1) 센서 읽기 (필요 시 사용)
  const uint16_t L = US_Left_cm();
  const uint16_t C = US_Center_cm();
  const uint16_t R = US_Right_cm();
  (void)L; (void)R; // 경고 억제 (아래에서 사용됨)

  // [STARTUP TURN BAN] 시작 1.2초 동안: 어떤 턴도 금지(비상 포함) → 저속 직진만
  if ((int32_t)(now - s_no_turn_until) < 0) {
    drive_forward();
    auto_motor_slow();   // 필요하면 주행 시작 램프업은 여기서 1~2회 더 호출
    return;              // 턴/피벗/아크 판단 자체를 스킵
  }

  // === 정지 금지 정책 ===
  // 너무 가까워도 멈추지 않음: 즉시 Pivot으로 회피
  if (C <= FRONT_TOO_CLOSE) {
    // 방향 투표 최신화(한 틱이라도 반영)
    int d = (R > L) ? +1 : -1;
    dir_vote += d;
    if (dir_vote > +2) dir_vote = +2;
    if (dir_vote < -2) dir_vote = -2;
    s_dir = (dir_vote >= 0) ? TURN_RIGHT : TURN_LEFT;

    // 즉시 Pivot 진입(강제)
    s_state = ST_TURN; s_mode = TURN_PIVOT;
    s_deadline_ms   = now + TURN_MS;
    s_hold_until_ms = now + HOLD_TURN_MS;

    // 정지 대신 저속 전진과 함께 반대휠 회전
    auto_motor_slow();
    if (s_dir == TURN_RIGHT) pivot_right(); else pivot_left();
    return;
  }

  // A) 의사결정 주기 제한
  const bool can_decide = ((int32_t)(now - s_next_decide_ms) >= 0);
  if (can_decide) s_next_decide_ms = now + DECIDE_EVERY_MS;

  switch (s_state)
  {
    // ---------------------------
    // DRIVING
    // ---------------------------
    case ST_DRIVE:
    {
      // E) 거리 기반 속도 거버너
      drive_forward();
      uint16_t near = (L < R) ? L : R;
      uint16_t min_all = (near < C) ? near : C;

      if      (min_all < 55) { auto_motor_slow(); }
      else if (min_all >= 70) { auto_motor_speedUp(); }
      // 중간 구간은 유지

      // C) 방향 스트릭(±2)
      if (can_decide) {
        int d = (R > L) ? +1 : -1;
        dir_vote += d;
        if (dir_vote > +2) dir_vote = +2;
        if (dir_vote < -2) dir_vote = -2;
      }
      dir_t dir = (dir_vote >= 0) ? TURN_RIGHT : TURN_LEFT;

      // B) 최소 상태 유지시간
      const bool can_switch = can_decide && ((int32_t)(now - s_hold_until_ms) >= 0);

      if (can_switch) {
        if (C <= FRONT_PIVOT_CM) {
          s_state = ST_TURN; s_mode = TURN_PIVOT; s_dir = dir;
          s_deadline_ms   = now + TURN_MS;
          s_hold_until_ms = now + HOLD_TURN_MS;
          auto_motor_slow();
          if (s_dir == TURN_RIGHT) pivot_right(); else pivot_left();
          break;
        } else if (C <= FRONT_ARC_CM) {
          s_state = ST_TURN; s_mode = TURN_ARC; s_dir = dir;
          s_deadline_ms   = now + ARC_MIN_MS;
          s_hold_until_ms = now + HOLD_TURN_MS;

          // Arc: 선감속 → 전진 + 바이어스 강하게 시작
          auto_motor_slow();
          drive_forward();
          if (s_dir == TURN_RIGHT) { auto_motor_right_speedDown(); auto_motor_right_speedDown();}
          else                      { auto_motor_left_speedDown(); auto_motor_left_speedDown();}
          break;
        }
      }
    } break;

    // ---------------------------
    // TURNING (PIVOT / ARC)
    // ---------------------------
    case ST_TURN:
    {
      if (s_mode == TURN_PIVOT) {
        // 시간 우선 종료(정지 없음 → 곧바로 DRV 저속 전진으로 복귀)
        if ((int32_t)(s_deadline_ms - now) <= 0) {
          s_state = ST_DRIVE;
          s_hold_until_ms = now + HOLD_DRIVE_MS;
          // 정지 대신 저속 전진
          drive_forward();
          auto_motor_slow();
          break;
        }
        // 유지만
      }
      else { // TURN_ARC
        const bool arc_min_elapsed = ((int32_t)(now - s_deadline_ms) >= 0);
        const bool arc_too_long    = ((int32_t)(now - s_deadline_ms) >= (int32_t)(ARC_MAX_MS - ARC_MIN_MS));

        // 정지 없이 복귀 조건
        if ((C >= FRONT_CLEAR_CM && arc_min_elapsed) || arc_too_long) {
          s_state = ST_DRIVE;
          s_hold_until_ms = now + HOLD_DRIVE_MS;
          // 정지 대신 저속 전진
          drive_forward();
          auto_motor_slow();
          break;
        }

        // D) 유지: 전진 + 바이어스 재적용(곡률 유지), 멈추지 않음
        drive_forward();
        if (s_dir == TURN_RIGHT) { auto_motor_right_speedDown(); }
        else                      { auto_motor_left_speedDown();  }
      }
    } break;
  }
}
