/*
 * automode.c — Minimal autonomous drive (No-Stop)
 *  - 후진 없음 / 정지 없음
 *  - Pivot(제자리 90°) + Arc(부드러운 커브)
 *  - 결정 주기 제한 / 상태 최소 유지 / 방향 스트릭(±2)
 *  - Arc 유지 중 바이어스 재적용 / 거리 기반 속도 거버너
 *  - [ΔC] 전방 거리 변화율 기반 가변 임계 (반커브/출구 대응)
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
  DECIDE_EVERY_MS  = 36,
  HOLD_DRIVE_MS    = 150,
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

// ====== [ΔC] 전방 거리 변화량 추적(SMA3) + 지속성(연속틱) ======
static int16_t  s_prevC = 0;           // 직전 C
static int16_t  s_dC_buf[3] = {0,0,0}; // 최근 3틱 ΔC 저장
static uint8_t  s_dC_idx = 0;
static uint8_t  s_dC_warm = 0;         // ΔC 워밍업(0~3샘플)

/* ΔC 임계(틱당 cm; DECIDE_EVERY_MS≈30~34ms 가정)
   dC_avg <= -3 → 급폐쇄(U자/급커브),  dC_avg >= +3 → 급개방(출구/가속구간) */
#define DC_FAST_CLOSE_CM   (-3)
#define DC_FAST_OPEN_CM    (+3)

/* ΔC 연속성(스파이크 무시) */
static uint8_t s_fast_close_streak = 0; // 연속 급폐쇄 틱 수
static uint8_t s_fast_open_streak  = 0; // 연속 급개방  틱 수
#define DC_CLOSE_STREAK_N  2            // 2~3 권장
#define DC_OPEN_STREAK_N   2

// ====== 유틸 ======
static inline bool valid_cm(uint16_t x){
  return (x >= 2 && x <= 300);
}
static inline uint16_t clamp16(uint16_t v, uint16_t lo, uint16_t hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

void AutoMode_Start(void)
{
  const uint32_t now = HAL_GetTick();
  s_no_turn_until = now + 1300;  // 시작 1.0s 턴 완전 금지
  dir_vote = 0;
  s_state = ST_DRIVE;

  // [ΔC] 초기화
  s_prevC = US_Center_cm();
  s_dC_buf[0] = s_dC_buf[1] = s_dC_buf[2] = 0;
  s_dC_idx = 0;
  s_dC_warm = 0;
  s_fast_close_streak = 0;
  s_fast_open_streak  = 0;
}

// ====== 메인 업데이트 루프 ======
void AutoMode_Update(void)
{
  const uint32_t now = HAL_GetTick();

  // 1) 센서 읽기
  const uint16_t L = US_Left_cm();
  const uint16_t C = US_Center_cm();
  const uint16_t R = US_Right_cm();

  // [ΔC] 변화량 업데이트 (SMA3)
  int16_t dC_now = (int16_t)C - (int16_t)s_prevC;   // 이번 틱 거리변화
  s_prevC = C;
  s_dC_buf[s_dC_idx] = dC_now;
  s_dC_idx = (uint8_t)((s_dC_idx + 1) % 3);
  if (s_dC_warm < 3) s_dC_warm++;

  int16_t dC_avg = (int16_t)((int32_t)s_dC_buf[0] + s_dC_buf[1] + s_dC_buf[2]) / 3;
  const bool dC_ready = (s_dC_warm >= 3);

  // [ΔC] 연속성 카운트(스파이크 무시)
  if (dC_ready && dC_avg <= DC_FAST_CLOSE_CM) s_fast_close_streak++;
  else                                         s_fast_close_streak = 0;

  if (dC_ready && dC_avg >= DC_FAST_OPEN_CM)  s_fast_open_streak++;
  else                                         s_fast_open_streak  = 0;

  // [STARTUP TURN BAN] 시작 1.0초: 어떤 턴도 금지(비상 포함) → 저속 직진만
  if ((int32_t)(now - s_no_turn_until) < 0) {
    drive_forward();
    auto_motor_slow();
    return;
  }

  // === 정지 금지 정책 ===
  if (C <= FRONT_TOO_CLOSE) {
    int d = (R > L) ? +1 : -1;
    dir_vote += d;
    if (dir_vote > +2) dir_vote = +2;
    if (dir_vote < -2) dir_vote = -2;
    s_dir = (dir_vote >= 0) ? TURN_RIGHT : TURN_LEFT;

    s_state = ST_TURN; s_mode = TURN_PIVOT;
    s_deadline_ms   = now + TURN_MS;
    s_hold_until_ms = now + HOLD_TURN_MS;

    auto_motor_slow();
    if (s_dir == TURN_RIGHT) pivot_right(); else pivot_left();
    return;
  }

  // A) 의사결정 주기 제한
  const bool can_decide = ((int32_t)(now - s_next_decide_ms) >= 0);
  if (can_decide) s_next_decide_ms = now + DECIDE_EVERY_MS;

  switch (s_state)
  {
    case ST_DRIVE:
    {
      // E) 거리 기반 속도 거버너 (+ 출구 가속 튐 억제)
      drive_forward();
      uint16_t near = (L < R) ? L : R;
      uint16_t min_all = (near < C) ? near : C;

      const bool fast_open_now = (dC_ready && dC_avg >= DC_FAST_OPEN_CM && s_fast_open_streak >= DC_OPEN_STREAK_N);
      if      (min_all < 52)                { auto_motor_slow(); }
      else if (!fast_open_now && min_all >= 65) { auto_motor_speedUp(); }
      // fast_open_now면 당장 가속은 보류(출구 흔들림 감소)

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
        // [ΔC] 변화율 기반 동적 임계 적용 (+클램프)
        uint16_t PIVOT_TH = FRONT_PIVOT_CM;
        uint16_t ARC_TH   = FRONT_ARC_CM;

        if (dC_ready) {
          if (s_fast_close_streak >= DC_CLOSE_STREAK_N) { // 빠르게 닫힘 "지속"
            PIVOT_TH = (uint16_t)(PIVOT_TH + 4);
            ARC_TH   = (uint16_t)(ARC_TH   + 3);
          } else if (s_fast_open_streak >= DC_OPEN_STREAK_N) { // 빠르게 열림 "지속"
            ARC_TH = (ARC_TH > 3) ? (uint16_t)(ARC_TH - 3) : ARC_TH; // Arc 재진입 억제
          }
        }
        PIVOT_TH = clamp16(PIVOT_TH, 40, 90);
        ARC_TH   = clamp16(ARC_TH,   55, 95);

        if (C <= PIVOT_TH) {
          s_state = ST_TURN; s_mode = TURN_PIVOT; s_dir = dir;
          s_deadline_ms   = now + TURN_MS;
          s_hold_until_ms = now + HOLD_TURN_MS;
          auto_motor_slow();
          if (s_dir == TURN_RIGHT) pivot_right(); else pivot_left();
          break;
        } else if (C <= ARC_TH) {
          s_state = ST_TURN; s_mode = TURN_ARC; s_dir = dir;
          s_deadline_ms   = now + ARC_MIN_MS;
          s_hold_until_ms = now + HOLD_TURN_MS;

          // Arc: 선감속 → 전진 + 바이어스 강하게 시작
          auto_motor_slow();
          drive_forward();
          if (s_dir == TURN_RIGHT) { auto_motor_right_speedDown(); auto_motor_right_speedDown(); }
          else                      { auto_motor_left_speedDown();  auto_motor_left_speedDown();  }
          break;
        }
      }
    } break;

    case ST_TURN:
    {
      if (s_mode == TURN_PIVOT) {
        // 시간 우선 종료(정지 없음 → 곧바로 DRV 저속 전진으로 복귀)
        if ((int32_t)(s_deadline_ms - now) <= 0) {
          s_state = ST_DRIVE;
          s_hold_until_ms = now + HOLD_DRIVE_MS;
          drive_forward();
          auto_motor_slow();
          break;
        }
        // 유지
      }
      else { // TURN_ARC
        const bool arc_min_elapsed = ((int32_t)(now - s_deadline_ms) >= 0);
        const bool arc_too_long    = ((int32_t)(now - s_deadline_ms) >= (int32_t)(ARC_MAX_MS - ARC_MIN_MS));

        // [ΔC] 급개방 "지속"이면 복귀를 약간 보수적으로 빠르게
        uint16_t CLEAR_TH = FRONT_CLEAR_CM;
        if (dC_ready && s_fast_open_streak >= DC_OPEN_STREAK_N && CLEAR_TH > 2) {
          CLEAR_TH = (uint16_t)(CLEAR_TH - 2); // 조금만 열려도 복귀
        }
        CLEAR_TH = clamp16(CLEAR_TH, 50, 90);

        if ((C >= CLEAR_TH && arc_min_elapsed) || arc_too_long) {
          s_state = ST_DRIVE;
          s_hold_until_ms = now + HOLD_DRIVE_MS;
          drive_forward();
          auto_motor_slow();
          break;
        }

        // 유지: 전진 + 바이어스 재적용(곡률 유지)
        drive_forward();
        if (s_dir == TURN_RIGHT) { auto_motor_right_speedDown(); }
        else                      { auto_motor_left_speedDown();  }
      }
    } break;
  }
}
