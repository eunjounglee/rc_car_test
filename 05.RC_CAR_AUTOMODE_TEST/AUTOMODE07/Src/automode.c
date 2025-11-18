#include <stdint.h>
#include <stdbool.h>
#include "move.h"
#include "ultrasonic.h"
#include "speed.h"

/* ---------------------------------------------------------------------------
 * 튜닝 상수
 *  - WALL_NEAR_CM    : 전/좌/우 중 하나라도 이 값 이하이면 '가까움'으로 보고 감속
 *  - FRONT_STOP_CM   : 전방이 이 값 이하이면 코너 회전을 준비(DECEL_FOR_*)
 *  - RIGHT_REF_CM    : (현재 코드에서는 사용 안 함. 이전 절대값 비교 방식 잔재)
 *
 *  - CRASH_*         : '충돌 가드'용 임계값들 (전방이 매우 가까울 때 강제 회피 시퀀스)
 *
 *  - DECEL_TICKS_FOR_TURN : 회전 전에 감속을 유지하는 틱(주기 10~20ms 가정)
 *  - BACKOFF_TICKS        : 충돌 가드에서 후진을 유지하는 틱
 *  - PIVOT_TICKS          : 회전 상태(ST_TURN_*) 또는 가드 피벗 지속 시간의 세이프가드
 * ---------------------------------------------------------------------------*/
enum {
  WALL_NEAR_CM          = 65,  // 측면/전방 중 65cm 이내면 감속
  FRONT_STOP_CM         = 55,  // 55cm 이내면 회전 절차 진입(코너 판단)

  // === '충돌 가드' 파트 (매우 가까울 때 급히 회피하는 상태 전이) ===
  CRASH_CM              = 18,  // 전방 18cm 이하가 연속되면 충돌로 간주
  CRASH_STREAK_N        = 1,   // 연속 샘플 N회(=1이면 한 번만 잡혀도 즉시) 충돌 판단
  CRASH_COOLDOWN_TICKS  = 40,  // 가드 종료 후 재발동 대기

  // === 회전/가드 동작 시간 파라미터 ===
  DECEL_TICKS_FOR_TURN  = 10,  // 회전 직전 감속 유지 시간
  BACKOFF_TICKS         = 20,  // 가드: 후진 유지 시간
  PIVOT_TICKS           = 25   // 회전/피벗 유지 시간(상한 세이프가드)
};


/* ---------------------------------------------------------------------------
 * 상태 정의
 *  - ST_STRAIGHT       : 직진(측면-차이 기반 P조향 + 근접시 감속)
 *  - ST_DECEL_FOR_*    : 회전 전에 강제 감속으로 속도 낮춤(선회 안정)
 *  - ST_TURN_*         : 좌/우 회전(피벗처럼 한쪽 전/다른쪽 후진을 설정)
 *
 *  - 아래 4개는 '충돌 가드' 전용 상태
 *    ST_EMERG_BRAKE    : 급감속
 *    ST_BACKOFF        : 양쪽 후진
 *    ST_PIVOT_AWAY_*   : 빈쪽으로 피벗 회전
 * ---------------------------------------------------------------------------*/
typedef enum {
    ST_STRAIGHT = 0,
    ST_DECEL_FOR_RIGHT,
    ST_DECEL_FOR_LEFT,
    ST_TURN_RIGHT,
    ST_TURN_LEFT,

    // --- 충돌 가드 전용 상태 ---
    ST_EMERG_BRAKE,       // 급감속(정지에 가깝게 유지)
    ST_BACKOFF,           // 양 바퀴 후진
    ST_PIVOT_AWAY_R,      // 우측(빈쪽)으로 피벗(좌전/우후)
    ST_PIVOT_AWAY_L       // 좌측(빈쪽)으로 피벗(좌후/우전)
} am_state_t;

/* ------------------------------
 * 모드 내부 상태 변수들
 * ------------------------------*/
static am_state_t s_state = ST_STRAIGHT; // 현재 상태
static int s_ticks = 0;                  // 다목적 타이머(상태별로 재사용)
static int s_crash_streak = 0;           // 전방 매우 근접 연속 카운터
static int s_crash_cooldown = 0;         // 가드 재발동 쿨다운
static uint16_t s_last_front = 1000;     // 디버깅용: 이전 프레임 전방값(현재 로직엔 영향 X)

// 직진 P조향 관련(현재는 P만 사용, D는 미사용. err_prev 저장만 하고 있음)
static int16_t s_err_prev = 0;

/* ---------------------------------------------------------------------------
 * 초기화
 *  - 속도 모듈 초기화(내부 PWM 상태를 BASE로 세팅)
 *  - 상태/타이머/가드 카운터 리셋
 *  - 진행방향은 전진으로 정렬
 * ---------------------------------------------------------------------------*/
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

/* ---------------------------------------------------------------------------
 * '충돌 가드' 조건 판단
 *  - front_cm <= CRASH_CM 이 연속 CRASH_STREAK_N회(여기선 1회) 발생하면 true
 *  - 매우 근접(코앞)일 때 가드를 트리거하기 위한 헬퍼
 * ---------------------------------------------------------------------------*/
static inline bool crash_detected(uint16_t front_cm)
{
    if (front_cm <= CRASH_CM) {
        s_crash_streak++;    // 매우 근접 샘플 누적
    } else {
        s_crash_streak = 0;  // 한 번이라도 넉넉해지면 리셋
    }
    return (s_crash_streak >= CRASH_STREAK_N);
}

/* ---------------------------------------------------------------------------
 * 메인 업데이트 루프 (주기 10~20ms 가정)
 * 1) 센서 샘플링
 * 2) (가드) 코앞이면 가드 우선
 * 3) 상태머신 분기
 *    - ST_STRAIGHT: P조향 + 근접 감속 + 전방 55cm 이내면 회전 준비
 *    - ST_DECEL_FOR_*: 일정 틱 감속 후 피벗 회전 상태로 진입
 *    - ST_TURN_*: 피벗 유지 + '안쪽벽' 기준 종료 또는 PIVOT_TICKS 상한
 *    - 가드 상태군: 급정지 → 후진 → 빈쪽 피벗 → 조건 충족 시 복귀
 * ---------------------------------------------------------------------------*/
void AutoMode_Update(void)
{
    // 1) 초음파 거리(cm) 샘플
    const uint16_t left_cm   = US_Left_cm();
    const uint16_t right_cm  = US_Right_cm();
    const uint16_t front_cm  = US_Center_cm();

    // 2) 가드 쿨다운 감소(가드가 끝난 뒤엔 일정 시간 재발동 금지)
    if (s_crash_cooldown > 0) s_crash_cooldown--;

    // 3) 코앞이면(가드 조건) — ST_STRAIGHT/회전 중에도 가드가 우선
    if (s_crash_cooldown == 0 &&
        (s_state == ST_STRAIGHT || s_state == ST_TURN_LEFT || s_state == ST_TURN_RIGHT) &&
        crash_detected(front_cm))
    {
        s_state = ST_EMERG_BRAKE; // 급감속부터
        s_ticks = 0;
    }

    // 4) 상태머신
    switch (s_state)
    {
    case ST_STRAIGHT:
    default:
    {
        // 항상 전진 방향(좌/우 모터)으로 맞춰 둔다
        left_dir_forward();
        right_dir_forward();

        // (a) 속도 스케일: 전방/측면 중 하나라도 가까우면 감속, 아니면 완만 가속
        if (front_cm <= WALL_NEAR_CM || left_cm <= WALL_NEAR_CM || right_cm <= WALL_NEAR_CM)
            auto_motor_speedDown();
        else
            auto_motor_speedUp();

       // (b) P조향: 좌우 차이(err=right-left)만큼 한 번 보정
         //     +면 우측이 더 멈: 차는 오른쪽으로 틀어야 하므로 왼모터↑/오른모터↓
        const int16_t DEAD  = 2;   // 작은 오차는 무시(데드존)
        const int16_t CLAMP = 10;  // 보정 최대치(현재 구현은 '호출 1회'라 강도 제한 효과는 제한적)
        int16_t err = (int16_t)right_cm - (int16_t)left_cm;
        int16_t u = 0; // 보정 강도(단위 없는 내부량)

        if (err > DEAD)       u = +(err - DEAD);
        else if (err < -DEAD) u = -( -err - DEAD );
        if (u >  CLAMP) u =  CLAMP;
        if (u < -CLAMP) u = -CLAMP;

        // 현재 구현은 u의 크기를 '호출 횟수'에 반영하지 않고, Up/Down 각각 1회만 호출
        // → 큰 오차도 1스텝만 보정되어 코너 직후 중앙 회복이 느림(분석 섹션 참조)
        if (u > 0) {  // 오른쪽으로 조향(왼↑/오↓)
            auto_motor_left_speedUp();
            auto_motor_right_speedDown();
        } else if (u < 0) { // 왼쪽으로 조향(오↑/왼↓)
            auto_motor_right_speedUp();
            auto_motor_left_speedDown();
        }
        s_err_prev = err; // D성분을 나중에 쓰려면 이전 오차 저장

        // (c) 전방이 55cm 이내면 회전 준비 상태로 진입
        //     더 먼 측면 쪽으로 회전(우>좌 → 우회전), 동률이면 우회전으로
        if (front_cm <= FRONT_STOP_CM) {
            s_ticks = 0;
            if      (right_cm > left_cm) s_state = ST_DECEL_FOR_RIGHT;
            else if (right_cm < left_cm) s_state = ST_DECEL_FOR_LEFT;
            else                         s_state = ST_DECEL_FOR_RIGHT;
        }
    } break;

    case ST_DECEL_FOR_RIGHT:
    {
        // 회전 전에 좌/우 모두 속도를 더 낮춰 선회 안정
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        // 일정 틱 감속 후 → 우회전: 좌전/우후(피벗)
        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            left_dir_forward();
            right_dir_backward();
            s_state = ST_TURN_RIGHT;
        }
    } break;

    case ST_DECEL_FOR_LEFT:
    {
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        // 일정 틱 감속 후 → 좌회전: 좌후/우전(피벗)
        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            left_dir_backward();
            right_dir_forward();
            s_state = ST_TURN_LEFT;
        }
    } break;

    case ST_TURN_RIGHT:
    {
        // 회전 중에도 근접이면 감속, 아니면 가속(※ 피벗 중 양쪽 동일 가/감속은 요레이트 불안정 요인)
        if (front_cm <= WALL_NEAR_CM) auto_motor_speedDown();
        else                          auto_motor_speedUp();

        // 회전 종료 조건(안쪽벽=right 기준) + PIVOT_TICKS 상한
        //  - front가 충분히 열리면(or 시간이 넘어가면) 직진 상태로 복귀
        if (front_cm > right_cm || ++s_ticks >= PIVOT_TICKS) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT; // 복귀(※ s_ticks를 0으로 리셋하는 것도 권장)
            s_ticks = 0;
        }
    } break;

    case ST_TURN_LEFT:
    {
        if (front_cm <= WALL_NEAR_CM) auto_motor_speedDown();
        else                          auto_motor_speedUp();

        // 좌회전 종료: 안쪽벽=left 기준 + 시간 상한
        if (front_cm > left_cm || ++s_ticks >= PIVOT_TICKS) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
            s_ticks = 0;
        }
    } break;

    // ================== 여기서부터 '충돌 가드' 전용 상태군 ==================

    case ST_EMERG_BRAKE:
    {
        // 급감속 유지(사실상 정지에 수렴)
        auto_motor_speedDown();
        auto_motor_left_speedDown();
        auto_motor_right_speedDown();

        // 충분히 감속되면 → 양 바퀴 후진으로 전환
        if (++s_ticks >= DECEL_TICKS_FOR_TURN) {
            left_dir_backward();
            right_dir_backward();
            s_state = ST_BACKOFF;
            s_ticks = 0;
        }
    } break;

    case ST_BACKOFF:
    {
        // 후진 유지(속도는 더 내리며 안전 확보)
        auto_motor_speedDown();

        // 후진이 끝나면 빈쪽(더 먼 쪽)으로 피벗 회전 시작
        if (++s_ticks >= BACKOFF_TICKS) {
            if (right_cm >= left_cm) {
                // 우측 여유가 크면 우측으로 피벗(좌전/우후)
                left_dir_forward();
                right_dir_backward();
                s_state = ST_PIVOT_AWAY_R;
            } else {
                // 좌측 여유가 크면 좌측으로 피벗(좌후/우전)
                left_dir_backward();
                right_dir_forward();
                s_state = ST_PIVOT_AWAY_L;
            }
            s_ticks = 0;
        }
    } break;

    case ST_PIVOT_AWAY_R:
    {
        // 피벗 유지(현재는 양쪽 동일 가속 호출 — 요레이트 변동 요인)
        auto_motor_speedUp();

        // front가 바깥벽(left)보다 커지거나, 시간이 넘으면 복귀
        if (front_cm > left_cm || ++s_ticks >= PIVOT_TICKS) {
            left_dir_forward();
            right_dir_forward();
            s_state = ST_STRAIGHT;
            s_crash_cooldown = CRASH_COOLDOWN_TICKS; // 재발동 쿨다운 시작
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

    // 디버깅 보조(현재 로직 영향 X)
    s_last_front = front_cm;
}
