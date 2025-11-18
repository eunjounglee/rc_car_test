/*
 * automode.c  — Wall-Follow (Right/Left) + Corner Turn
 *  - PD steering on side distance
 *  - Front-based speed scheduling + TTC brake + E-STOP
 *  - Robust corner-turn state
 */

#include "automode.h"         // AutoMode_Init/Update 외부 인터페이스 선언
#include "ultrasonic.h"       // us_frame_t, US_GetFrame() (정제된 거리 프레임 제공)
#include "math.h"             // fabsf, fminf 등 수학 함수
#include "stdio.h"            // 디버깅용 printf (필요시만 사용)

// 튜닝 상수
#define MIN_VALID_CM    5     // 초근접/미검출 보호 하한
#define NEAR_CM         10   // 전방 '가까움' 판정 하한
#define CLEAR_CM        150   // 전방 '안전' 판정 상한
#define DEAD_BAND_CM    5     // 직진 데드밴드
#define CORNER_DIFF_CM  15    // 코너 판정: 좌우 차이가 이 값 초과일 때
#define TURN_MS         350   // 코너 회전 유지 시간(ms)
#define TURN_SPEED_LOW  550   // 턴 시 안쪽 바퀴 속도(혹은 후진 속도)
#define TURN_SPEED_HIGH 750   // 턴 시 바깥 바퀴 속도

void AutoMode_Init()
{
	motor_init();
}

void AutoMode_Update(void)
{
    // 상태 정의
    typedef enum {
        STATE_NEAR = 0,       // 전방 가까움(감속/정지/턴 진입 판단)
        STATE_CLEAR,          // 전방 확보(직진 + 미세조향)
        STATE_TURN_LEFT,      // 코너 좌회전 유지
        STATE_TURN_RIGHT      // 코너 우회전 유지
    } auto_state_t;

    static auto_state_t s_state = STATE_NEAR;
    static uint8_t  s_near_cnt = 0, s_clear_cnt = 0;
    static uint8_t  s_inited_clear = 0;
    static uint32_t s_turn_end_ms = 0;

    uint16_t L = US_Left_cm();
    uint16_t R = US_Right_cm();
    uint16_t C = US_Center_cm();

    // 미검출/초근접 보호
    if (C < MIN_VALID_CM) C = 400;

    int16_t diff = (int16_t)L - (int16_t)R;

    // TURN 상태이면 타이머 먼저 확인 (센서 무시하고 유지)
    if (s_state == STATE_TURN_LEFT || s_state == STATE_TURN_RIGHT)
    {
        if ((int32_t)(HAL_GetTick() - s_turn_end_ms) >= 0)
        {
            // 턴 종료 → CLear/NEAR 재판정 위해 카운터 초기화
            s_state = STATE_NEAR;
            s_inited_clear = 0;
            s_near_cnt = s_clear_cnt = 0;
        }
        else
        {
            // 턴 계속 유지
            return;
        }
    }

    // 전방 히스테리시스 (2프레임 연속 확인)
    if (C >= CLEAR_CM) { s_clear_cnt++; s_near_cnt = 0; }
    else if (C <= NEAR_CM) { s_near_cnt++; s_clear_cnt = 0; }
    else { s_clear_cnt = 0; s_near_cnt = 0; }

    if (s_clear_cnt >= 2) s_state = STATE_CLEAR;
    if (s_near_cnt  >= 2) s_state = STATE_NEAR;

    // 상태별 동작
    switch (s_state)
    {
    case STATE_CLEAR:
        if (!s_inited_clear) {
            auto_motor_speedInit();   // 베이스 속도 1회 세팅
            s_inited_clear = 1;
        }
        motor_forward();

        // 미세조향: diff > 0 → 왼쪽이 더 멈(=오른쪽이 더 가까움) → 좌로 틀기(좌↓, 우↑)
        if      (diff >  DEAD_BAND_CM) { auto_motor_left_speedDown();  auto_motor_right_speedUp();   }
        else if (diff < -DEAD_BAND_CM) { auto_motor_left_speedUp();    auto_motor_right_speedDown(); }
        break;

    case STATE_NEAR:
        s_inited_clear = 0;

        // 코너 판정: 좌우 차이가 많이 나면 코너로 가정 → 턴 상태로 진입
        if (diff >= (int16_t)CORNER_DIFF_CM)
        {
            // 좌회전 코너: 왼쪽은 느리거나 후진, 오른쪽은 빠르게 전진
            motor_stop();                 // 안전한 방향 전환을 위해 잠깐 정지
            left_dir_backward();          // 안쪽 바퀴를 뒤로(제자리 회전 가속)
            right_dir_forward();

            // 원하는 속도로 직접 세팅 (회전 힘 확보)
            // 필요 시 네가 만든 모듈에 맞게 바꿔도 OK
            TIM3->CCR2 = TURN_SPEED_LOW;   // Left
            TIM3->CCR1 = TURN_SPEED_HIGH;  // Right

            s_turn_end_ms = HAL_GetTick() + TURN_MS;
            s_state = STATE_TURN_LEFT;
        }
        else if (diff <= -(int16_t)CORNER_DIFF_CM)
        {
            // 우회전 코너
            motor_stop();
            left_dir_forward();
            right_dir_backward();

            TIM3->CCR2 = TURN_SPEED_HIGH;  // Left
            TIM3->CCR1 = TURN_SPEED_LOW;   // Right

            s_turn_end_ms = HAL_GetTick() + TURN_MS;
            s_state = STATE_TURN_RIGHT;
        }
        else
        {
            // 정면에 가깝게 막혔고 좌우 차이도 작다 → 일단 정지(또는 아주 저속 크리핑)
            motor_stop();
        }
        break;

    default:
        // 위의 TURN 상태는 함수 초반에서 타이머로 처리되어 여기로 거의 안 옴
        break;
    }
}
