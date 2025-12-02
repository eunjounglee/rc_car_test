/*
 * automode.c — Minimal autonomous drive (No-Stop)  [PATCHED v2 — Gate+ExitTune, pruned]
 */

#include "automode.h"
#include <stdbool.h>
#include <stdint.h>

// [내부 상태] 로봇의 기억 (전역변수지만 static으로 숨김)
typedef enum {
    STATE_DRIVE,       // 평상시 주행
    STATE_AVOID_BACK,  // 너무 가까워서 후진
    STATE_AVOID_TURN,  // 회피 회전
    STATE_RECOVERY     // 자세 잡기
} State_t;

static State_t s_state = STATE_DRIVE;
static uint32_t s_timer = 0;     // 타이머 (Deadline)
static int s_turn_dir = 1;       // 1: Right, -1: Left

// =====================================================
//  뇌 함수: 입력을 받아 출력을 결정한다 (Pure Logic)
// =====================================================
AutoOutput_t AutoMode_Run(AutoInput_t input)
{
    // 1. 기본 출력 (아무것도 안 하면 정지)
    AutoOutput_t output = {
        .Action = ACTION_STOP,
        .Speed_L = 0,
        .Speed_R = 0
    };

    // 2. 상태 머신 (Brain Logic)
    switch (s_state)
    {
        // -----------------------------------------------------
        case STATE_DRIVE:
        // -----------------------------------------------------
            output.Action = ACTION_FORWARD;
            output.Speed_L = 400; // 기본 속도
            output.Speed_R = 400;

            // [상태 전이 조건]
            // 1. 너무 가까우면(10cm) -> 후진
            if (input.Dist_C < 10) {
                s_state = STATE_AVOID_BACK;
                s_timer = input.Current_Time_ms + 500; // 0.5초간 후진 예약
            }
            // 2. 적당히 가까우면(40cm) -> 회전 (장애물 반대쪽으로)
            else if (input.Dist_C < 40) {
                s_state = STATE_AVOID_TURN;
                s_timer = input.Current_Time_ms + 400; // 0.4초간 회전 예약
                // 공간이 넓은 쪽으로 회전 방향 결정
                if (input.Dist_L > input.Dist_R) s_turn_dir = -1; // Left
                else                             s_turn_dir = 1;  // Right
            }
            break;

        // -----------------------------------------------------
        case STATE_AVOID_BACK:
        // -----------------------------------------------------
            output.Action = ACTION_BACKWARD;
            output.Speed_L = 350;
            output.Speed_R = 350;

            // [타이머 만료 체크]
            if (input.Current_Time_ms >= s_timer) {
                s_state = STATE_AVOID_TURN; // 후진 끝났으면 돌려서 나가자
                s_timer = input.Current_Time_ms + 400;
            }
            break;

        // -----------------------------------------------------
        case STATE_AVOID_TURN:
        // -----------------------------------------------------
            // 결정된 방향으로 회전
            if (s_turn_dir > 0) output.Action = ACTION_PIVOT_RIGHT;
            else                output.Action = ACTION_PIVOT_LEFT;

            output.Speed_L = 350;
            output.Speed_R = 350;

            // [조건] 시간이 됐거나, 앞이 뻥 뚫리면(100cm 이상) 탈출
            if (input.Current_Time_ms >= s_timer || input.Dist_C > 100) {
                s_state = STATE_DRIVE; // 다시 주행 복귀
            }
            break;

        default:
            s_state = STATE_DRIVE;
            break;
    }

    // 3. 결정된 생각(Output)을 반환
    return output;
}
