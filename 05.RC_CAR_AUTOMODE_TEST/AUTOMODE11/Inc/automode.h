/*
 * automode.h
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */

#ifndef INC_AUTOMODE_H_
#define INC_AUTOMODE_H_

#include "main.h"

void AutoMode_Start();
void AutoMode_Update();
void get_speed(uint16_t *right_pwm, uint16_t *left_pwm);

#endif /* INC_AUTOMODE_H_ */
