/*
 * automode.h
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */

#ifndef INC_AUTOMODE_H_
#define INC_AUTOMODE_H_

#include "main.h"
#include "stdint.h"
#include "ultrasonic.h"
#include "move.h"         // right_dir_forward/backward, left_dir_forward/backward
#include "speed.h"        // motor_setDuty(), motor_recover(), motor_speedInit()
#include "stdbool.h"

void AutoMode_Init(void);
void AutoMode_Update(void);   // 10~20 ms 주기 호출


#endif /* INC_AUTOMODE_H_ */
