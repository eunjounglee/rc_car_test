/*
 * automode.c — Minimal autonomous drive (No-Stop)  [PATCHED v2 — Gate+ExitTune, pruned]
 */

#include "automode.h"
#include "ultrasonic.h"
#include "speed.h"
#include "move.h"
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// ====== 튜닝 상수 ======
enum {
  // 코너 판단 임계 (히스테리시스)
  FRONT_PIVOT_CM   = 64,
  FRONT_ARC_CM     = 79,
  FRONT_CLEAR_CM   = 75,

  // 회전/커브 유지 시간
  TURN_MS          = 350,
  ARC_MIN_MS       = 300,
  ARC_MAX_MS       = 900,

  // 안정화(기본값)
  DECIDE_EVERY_MS  = 30,
  HOLD_DRIVE_MS    = 150,
  HOLD_TURN_MS     = 130,

  // 너무 가까움 임계 (정지 금지 정책: 즉시 Pivot)
  FRONT_TOO_CLOSE  = 12
};

// ====== 상태/보조 ======
typedef enum { ST_DRIVE, ST_TURN } state_t;
typedef enum { TURN_PIVOT, TURN_ARC } mode_t;
typedef enum { TURN_LEFT=-1, TURN_RIGHT=+1 } dir_t;

static state_t  s_state = ST_DRIVE;
static mode_t   s_mode;
static dir_t    s_dir;
static uint32_t s_deadline_ms = 0;

static int8_t   dir_vote = 0;
static uint32_t s_next_decide_ms = 0;
static uint32_t s_hold_until_ms  = 0;

static uint32_t s_no_turn_until = 0;

// ΔC 추적
static int16_t  s_prevC = 0;
static int16_t  s_dC_buf[3] = {0,0,0};
static uint8_t  s_dC_idx = 0;
static uint8_t  s_dC_warm = 0;
#define DC_FAST_CLOSE_CM   (-3)
#define DC_FAST_OPEN_CM    (+3)
static uint8_t s_fast_close_streak = 0;
static uint8_t s_fast_open_streak  = 0;
#define DC_CLOSE_STREAK_N  2
#define DC_OPEN_STREAK_N   2

// 연속커브/방지턱/바이어스 보조
static uint32_t s_last_turn_end   = 0;
static uint32_t s_arc_chain_until = 0;
static bool     s_in_bump         = false;
static uint32_t s_bump_until      = 0;
static uint32_t s_speedup_cool_until = 0;

static uint32_t s_last_arc_bias_ms   = 0;
static uint8_t  s_arc_phase          = 0;

// ====== 유틸 ======
static inline bool     valid_cm(uint16_t x){ return (x >= 2 && x <= 300); }
static inline uint16_t clamp16(uint16_t v, uint16_t lo, uint16_t hi){ return (v<lo)?lo:(v>hi)?hi:v; }
static inline int16_t  i16_abs(int16_t v){ return (v>=0)?v:(int16_t)(-v); }

// ====== 시작 ======
void AutoMode_Start(void)
{
  const uint32_t now = HAL_GetTick();
  s_no_turn_until = now + 1500;  // 1.5s
  dir_vote = 0;
  s_state = ST_DRIVE;

  s_prevC = US_Center_cm();
  s_dC_buf[0] = s_dC_buf[1] = s_dC_buf[2] = 0;
  s_dC_idx = 0; s_dC_warm = 0;
  s_fast_close_streak = 0; s_fast_open_streak = 0;

  s_last_turn_end   = now;
  s_arc_chain_until = now;
  s_in_bump = false; s_bump_until = now;
  s_speedup_cool_until = now;
  s_last_arc_bias_ms = 0;
  s_arc_phase = 0;
}

// ====== 메인 ======
void AutoMode_Update(void)
{
  const uint32_t now = HAL_GetTick();

  // 센서
  const uint16_t L = US_Left_cm();
  const uint16_t C = US_Center_cm();
  const uint16_t R = US_Right_cm();

  // ΔC 업데이트
  int16_t dC_now = (int16_t)C - (int16_t)s_prevC;
  s_prevC = C;
  s_dC_buf[s_dC_idx] = dC_now;
  s_dC_idx = (uint8_t)((s_dC_idx + 1) % 3);
  if (s_dC_warm < 3) s_dC_warm++;

  int16_t dC_avg = (int16_t)((int32_t)s_dC_buf[0] + s_dC_buf[1] + s_dC_buf[2]) / 3;
  const bool dC_ready = (s_dC_warm >= 3);

  if (dC_ready && dC_avg <= DC_FAST_CLOSE_CM) s_fast_close_streak++; else s_fast_close_streak = 0;
  if (dC_ready && dC_avg >= DC_FAST_OPEN_CM)  s_fast_open_streak++;  else s_fast_open_streak  = 0;

  // STARTUP 금지
  if ((int32_t)(now - s_no_turn_until) < 0) {
    drive_forward(); auto_motor_slow(); return;
  }

  // 비상 Pivot
  if (C <= FRONT_TOO_CLOSE) {
    int d = (R > L) ? +1 : -1;
    dir_vote += d; if (dir_vote>+2) dir_vote=+2; if (dir_vote<-2) dir_vote=-2;
    s_dir = (dir_vote >= 0) ? TURN_RIGHT : TURN_LEFT;

    s_state = ST_TURN; s_mode = TURN_PIVOT;
    s_deadline_ms   = now + TURN_MS;
    s_hold_until_ms = now + HOLD_TURN_MS;

    s_in_bump = false;
    auto_motor_slow();
    if (s_dir == TURN_RIGHT) pivot_right(); else pivot_left();
    return;
  }

  // 연속커브 윈도우
  const bool in_chain = ((int32_t)(now - s_arc_chain_until) < 0);

  // 결정주기(체인 가속), DRV 홀드
  const uint32_t decide_ms = in_chain ? 16u : (uint32_t)DECIDE_EVERY_MS;
  const uint32_t hold_drv  = in_chain ? 70u : (uint32_t)HOLD_DRIVE_MS;

  // 의사결정 주기
  const bool can_decide = ((int32_t)(now - s_next_decide_ms) >= 0);
  if (can_decide) s_next_decide_ms = now + decide_ms;

  // 방지턱 감지
  if (!s_in_bump && (int32_t)(now - s_last_turn_end) <= 350 && dC_ready) {
    bool dc_wobble =
      (i16_abs(dC_avg) <= 2) &&
      ( (s_dC_buf[(s_dC_idx+2)%3] > 0 && s_dC_buf[(s_dC_idx+1)%3] < 0) ||
        (s_dC_buf[(s_dC_idx+2)%3] < 0 && s_dC_buf[(s_dC_idx+1)%3] > 0) );
    if (dc_wobble && C >= 45 && C <= 85) {
      s_in_bump = true;
      s_bump_until = now + (in_chain ? 180u : 240u);
    }
  }

  if (s_in_bump) {
    drive_forward();
    auto_motor_speedUp();
    if ((int32_t)(now - s_speedup_cool_until) >= 0) s_speedup_cool_until = now + 120;
    if ( (int32_t)(now - s_bump_until) >= 0 || (dC_ready && i16_abs(dC_avg) <= 1) ) s_in_bump = false;
    return;
  }

  switch (s_state)
  {
    case ST_DRIVE:
    {
      // 속도 거버너
      drive_forward();
      uint16_t near = (L < R) ? L : R;
      uint16_t min_all = (near < C) ? near : C;

      const bool fast_open_now = (dC_ready && dC_avg >= DC_FAST_OPEN_CM && s_fast_open_streak >= DC_OPEN_STREAK_N);

      if      (min_all < 62) {        // 코너 초입 과속 억제
        auto_motor_slow();
      } else {
        if ((int32_t)(now - s_speedup_cool_until) >= 0) {
          if (min_all >= 68) {
            auto_motor_speedUp();
            s_speedup_cool_until = now + (fast_open_now ? 140 : 100);
          }
        }
      }

      // 방향 스트릭
      if (can_decide) {
        int d = (R > L) ? +1 : -1;
        dir_vote += d; if (dir_vote>+2) dir_vote=+2; if (dir_vote<-2) dir_vote=-2;
      }
      dir_t dir = (dir_vote >= 0) ? TURN_RIGHT : TURN_LEFT;

      // 타입 분기용 좌우 차
      int16_t lr_diff = (int16_t)L - (int16_t)R;
      int16_t lr_abs  = (lr_diff >= 0) ? lr_diff : (int16_t)(-lr_diff);
      bool sharpU = dC_ready && (dC_avg <= DC_FAST_CLOSE_CM) && (s_fast_close_streak >= 2) && (lr_abs <= 10);
      bool softL  = (lr_abs >= 20) && !(dC_ready && dC_avg <= DC_FAST_CLOSE_CM);

      const bool can_switch = can_decide && ((int32_t)(now - s_hold_until_ms) >= 0);

      if (can_switch) {
        uint16_t PIVOT_TH = FRONT_PIVOT_CM;
        uint16_t ARC_TH   = FRONT_ARC_CM;

        // ΔC 기반 임계 보정 먼저
        if (dC_ready) {
          if (s_fast_close_streak >= DC_CLOSE_STREAK_N) { PIVOT_TH += 4; ARC_TH += 3; }
          else if (s_fast_open_streak >= DC_OPEN_STREAK_N) {
            ARC_TH = (ARC_TH > 3) ? (uint16_t)(ARC_TH - 3) : ARC_TH;
          }
        }

        // 체인에서 Pivot 완화
        if (in_chain && PIVOT_TH >= 2) PIVOT_TH = (uint16_t)(PIVOT_TH - 2);

        PIVOT_TH = clamp16(PIVOT_TH, 40, 90);
        ARC_TH   = clamp16(ARC_TH,   55, 95);

        // softL이라도 아주 가까우면 Pivot 허용(예외)
        bool allow_pivot = !softL;
        if (!allow_pivot && C <= (uint16_t)(PIVOT_TH - 4)) allow_pivot = true;

        if (C <= PIVOT_TH && allow_pivot) {
          s_state = ST_TURN; s_mode = TURN_PIVOT; s_dir = dir;
          s_deadline_ms   = now + TURN_MS;
          s_hold_until_ms = now + HOLD_TURN_MS;
          auto_motor_slow();
          if (s_dir == TURN_RIGHT) pivot_right(); else pivot_left();
          break;
        } else if (C <= ARC_TH) {
          s_state = ST_TURN; s_mode = TURN_ARC; s_dir = dir;
          uint32_t arc_min = ARC_MIN_MS;
          if (sharpU) arc_min = 240;       // U자면 약간 짧게
          s_deadline_ms   = now + arc_min;
          s_hold_until_ms = now + HOLD_TURN_MS;

          s_arc_phase  = softL ? 1 : 0;    // 완만 ㄱ자는 약하게 시작
          s_last_arc_bias_ms = 0;

          auto_motor_slow();
          drive_forward();
          break;
        }
      }
    } break;

    case ST_TURN:
    {
      if (s_mode == TURN_PIVOT) {
        if ((int32_t)(s_deadline_ms - now) <= 0) {
          s_state = ST_DRIVE;
          s_hold_until_ms = now + hold_drv;
          drive_forward(); auto_motor_slow();

          s_last_turn_end   = now;
          s_arc_chain_until = now + 350;

          s_arc_phase = 0;
          break;
        }
      }
      else { // TURN_ARC
        const bool arc_min_elapsed = ((int32_t)(now - s_deadline_ms) >= 0);
        const bool arc_too_long    = ((int32_t)(now - s_deadline_ms) >= (int32_t)(ARC_MAX_MS - ARC_MIN_MS));

        // 출구 조기복귀(보수화)
        bool center_open = dC_ready && (dC_avg >= +4) && (s_fast_open_streak >= 3);
        uint16_t side_far = (s_dir == TURN_RIGHT) ? L : R;   // 바깥쪽
        bool outer_open = valid_cm(side_far) && (side_far >= 82);

        uint16_t CLEAR_TH = FRONT_CLEAR_CM + 2;   // 기본 +2
        if (dC_ready && s_fast_open_streak >= DC_OPEN_STREAK_N && CLEAR_TH > 2) {
          CLEAR_TH = (uint16_t)(CLEAR_TH - 2);    // 급개방 지속 시 원래 수준으로
        }
        CLEAR_TH = clamp16(CLEAR_TH, 50, 90);

        if ( (arc_min_elapsed && center_open && outer_open) ||
             (C >= CLEAR_TH && arc_min_elapsed) ||
             arc_too_long) {
          s_state = ST_DRIVE;
          s_hold_until_ms = now + hold_drv;
          drive_forward(); auto_motor_slow();

          s_last_turn_end   = now;
          s_arc_chain_until = now + 350;

          s_arc_phase = 0;
          break;
        }

        // 바이어스 램프다운(프레임당 1회, 단계적 약화)
        drive_forward();
        if ((int32_t)(now - s_last_arc_bias_ms) >= 0) {
          if (s_arc_phase <= 2) {
            if (s_dir == TURN_RIGHT) { auto_motor_right_speedDown(); }
            else                      { auto_motor_left_speedDown();  }
          }
          s_last_arc_bias_ms = now + DECIDE_EVERY_MS;
          if (s_arc_phase < 3) s_arc_phase++;
        }
      }
    } break;
  }
}
