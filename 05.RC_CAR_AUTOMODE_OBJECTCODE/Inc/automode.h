/*
 * automode.h
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */

#ifndef INC_AUTOMODE_H_
#define INC_AUTOMODE_H_

#include "main.h"
#include "robot_driver.h"

// 1. 행동 정의 (로봇이 할 수 있는 동작들)
typedef enum {
    ACTION_STOP = 0,
    ACTION_FORWARD,
    ACTION_BACKWARD,
    ACTION_PIVOT_LEFT,
    ACTION_PIVOT_RIGHT,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT
} RobotAction_t;

// 2. 입력 구조체 (Sensing)
typedef struct {
    uint16_t Dist_L;
    uint16_t Dist_C;
    uint16_t Dist_R;
    uint32_t Current_Time_ms;
} AutoInput_t;

// 3. 출력 구조체 (Decision)
typedef struct {
    RobotAction_t Action;      // 무엇을 할지
    uint16_t      Speed_L;     // 왼쪽 속도
    uint16_t      Speed_R;     // 오른쪽 속도
} AutoOutput_t;

// 4. [핵심] 뇌 함수 선언
// 입력(Input)을 먹고, 출력(Output)을 뱉는다!
// "void"가 아니라 "AutoOutput_t"를 반환.
AutoOutput_t AutoMode_Run(AutoInput_t input);

// (옵션) 상태 초기화 함수 등
void AutoMode_Init(void);
void AutoMode_Act(RobotCar_t *car, AutoOutput_t cmd);

#endif /* INC_AUTOMODE_H_ */
