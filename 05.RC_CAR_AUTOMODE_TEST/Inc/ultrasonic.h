/*
 * ultrasonic.h
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */

#ifndef INC_ULTRASONIC_H_
#define INC_ULTRASONIC_H_

#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "delay_us.h"
#include "stdio.h"

#define TRIG_PORT_LEFT	  GPIOC
#define TRIG_PIN_LEFT	    GPIO_PIN_8
#define TRIG_PORT_RIGHT	  GPIOC
#define TRIG_PIN_RIGHT	  GPIO_PIN_5
#define TRIG_PORT_CENTER	GPIOC
#define TRIG_PIN_CENTER	  GPIO_PIN_6

void HCSR04_TRIGGER_LEFT();
void HCSR04_TRIGGER_RIGHT();
void HCSR04_TRIGGER_CENTER();

void processUltrasonic_All();

void filter();

void US_Update();
void US_Init();
void US_FilterInit();

uint16_t US_Left_cm();
uint16_t US_Right_cm();
uint16_t US_Center_cm();


#endif /* INC_ULTRASONIC_H_ */
