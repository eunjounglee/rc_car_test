#include <stdint.h>
#include <stdbool.h>
#include "move.h"
#include "ultrasonic.h"
#include "speed.h"

/* -------------------------------------------------------------
 * 튜닝 상수 — 비감속형 부드러운 벽추종
 * ------------------------------------------------------------- */
enum {
  // 전방 임계들
  FRONT_STOP_CM   = 65,   // 코너 진입 판단(전방 충분히 가까움)
  FRONT_CLEAR_CM  = 45,   // 코너 탈출(전방 다시 확보)
  FRONT_PREP_BAND = 70,   // 준비 밴드: 속도 동결(Up/Down 금지)

  // 측면 중심추종
  CENTER_TOL_CM   = 10,   // L≈R 허용 밴드(중심 판정)
  CENTER_BIAS_CM  = 0,    // 현장 보정(좌 센서가 크게 나오면 +)

  // 직선구간 P-유사 조향(미세 감속만)
  DEAD            = 3,    // 작은 오차 무시
  CLAMP           = 5,    // 한 틱에서 줄일 최대 보정량(개념상)

  // 코너 조향 강도(전진만, 역회전 없음)
  HARD_STEER_TICKS = 2,   // 코너 진입 시 안쪽 바퀴 연속 Down 횟수
  SOFT_STEER_TICKS = 1,   // 직선 미세 보정 시 Down 횟수
};

/* -------------------------------------------------------------
 * 상태 정의
 * ------------------------------------------------------------- */
typedef enum {
  ST_CRUISE = 0,      // 직선 중심 추종(전진)
  ST_TURN_RIGHT_FWD,  // 전진 유지 + 우측 차동조향(오른바퀴만 더 줄임)
  ST_TURN_LEFT_FWD    // 전진 유지 + 좌측 차동조향(왼바퀴만 더 줄임)
} am_state_t;

/* ------------------------------------------------------------- */
static am_state_t s_state = ST_CRUISE;
static uint16_t s_prev_front = 1000;

/* 편의 함수 */
static inline int16_t lr_err(uint16_t L, uint16_t R) {
  return (int16_t)R - (int16_t)L - (int16_t)CENTER_BIAS_CM; // R-L (양수면 우측이 멂)
}
static inline bool centered(uint16_t L, uint16_t R) {
  const int16_t d = (int16_t)L - (int16_t)R - (int16_t)CENTER_BIAS_CM;
  return (d >= -CENTER_TOL_CM) && (d <= CENTER_TOL_CM);
}

/* ------------------------------------------------------------- */
void AutoMode_Init(void)
{
  auto_motor_speedInit();        // 기본 속도 세팅(유지)
  s_state = ST_CRUISE;
  s_prev_front = 1000;

  left_dir_forward();
  right_dir_forward();
}

/* ------------------------------------------------------------- */
void AutoMode_Update(void)
{
  const uint16_t L = US_Left_cm();
  const uint16_t R = US_Right_cm();
  const uint16_t F = US_Center_cm();

  // 항상 전진 방향 고정
  left_dir_forward();
  right_dir_forward();

  // 준비 밴드에선 속도 동결: Up/Down 호출 자체를 금지
  const bool in_prep_band = (F <= FRONT_PREP_BAND);

  switch (s_state)
  {
  default:
  case ST_CRUISE:
  {
    // 1) 코너 진입 판단: 전방이 충분히 가까워지면 열린쪽으로 전진 조향
    if (F <= FRONT_STOP_CM) {
      if (R > L) {
        // 우측이 열림 → 우측으로 전진 조향(오른바퀴만 더 줄임)
        for (int i=0; i<HARD_STEER_TICKS; ++i) auto_motor_right_speedDown();
        s_state = ST_TURN_RIGHT_FWD;
      } else {
        // 좌측이 열림 → 좌측으로 전진 조향(왼바퀴만 더 줄임)
        for (int i=0; i<HARD_STEER_TICKS; ++i) auto_motor_left_speedDown();
        s_state = ST_TURN_LEFT_FWD;
      }
      break;
    }

    // 2) 직선 구간 중심 추종: 가까운 벽의 "반대 바퀴만" 살짝 줄인다
    if (!in_prep_band) {
      const int16_t e = lr_err(L, R); // R-L
      int16_t u = 0;
      if (e > DEAD)       u = (e - DEAD);
      else if (e < -DEAD) u = -( -e - DEAD );
      if (u >  CLAMP) u =  CLAMP;
      if (u < -CLAMP) u = -CLAMP;

      if (u > 0) {
        // 우측이 더 멂 = 차가 좌로 붙는 경향 → 오른바퀴만 살짝 줄여서 우로 보정
        for (int i=0; i<SOFT_STEER_TICKS; ++i) auto_motor_right_speedDown();
      } else if (u < 0) {
        // 좌측이 더 멂 = 차가 우로 붙는 경향 → 왼바퀴만 살짝 줄여서 좌로 보정
        for (int i=0; i<SOFT_STEER_TICKS; ++i) auto_motor_left_speedDown();
      } else {
        // 중앙에 가까움 → 양쪽 그대로(속도 동결 철학)
        // 필요하면 아주 가끔씩 좌우를 1틱씩 Up해서 장기 드리프트만 보정
      }
    }
  } break;

  case ST_TURN_RIGHT_FWD:
  {
    // 전진 유지 + 오른바퀴 Down 위주로 차동조향
    // 코너 탈출 판단: 전방 회복 + 중심 근접
    if (F >= FRONT_CLEAR_CM && centered(L, R)) {
      s_state = ST_CRUISE;
      break;
    }
    // 계속 우측으로 꺾는 미세 보정(속도 동결 원칙하에 오른쪽만 Down)
    if (!in_prep_band) auto_motor_right_speedDown();
  } break;

  case ST_TURN_LEFT_FWD:
  {
    if (F >= FRONT_CLEAR_CM && centered(L, R)) {
      s_state = ST_CRUISE;
      break;
    }
    if (!in_prep_band) auto_motor_left_speedDown();
  } break;
  }

  s_prev_front = F;
}
