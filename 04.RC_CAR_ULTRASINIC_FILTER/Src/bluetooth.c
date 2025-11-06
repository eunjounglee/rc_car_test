/*
 * bluetooth.c
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */


#include "bluetooth.h"


uint8_t serial_RxData;
uint8_t bluetooth_RxData;


volatile uint8_t forward=0, backward=0, right=0, left=0;
volatile uint8_t speedUp=0, speedDown=0;
volatile uint8_t noDir=0, stop=0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
	HAL_UART_Transmit (&huart2, &bluetooth_RxData, 1, 1000);
	HAL_UART_Transmit (&huart1, &serial_RxData, 1, 1000);

	if(serial_RxData == 'F'||
		 serial_RxData == 'B'||
		 serial_RxData == 'R'||
		 serial_RxData == 'L'||
		 serial_RxData == 'T'||
		 serial_RxData == 'X'||
		 serial_RxData == 'C'||
		 serial_RxData == 'S'||
		 serial_RxData == 'D'||
		 serial_RxData == '0')
	{
		switch(serial_RxData)
		{
			// F: 앞으로 이동  B: 뒤로 이동  R: 오른쪽으로 회전  L: 왼쪽으로 회전
			case 'F' :
				forward = 1;
				break;
			case 'B' :
				backward = 1;
				break;
			case 'R' :
				right = 1;
				break;
			case 'L' :
				left = 1;
				break;
			// T: 가속  X: 감속
			case 'T' :
				speedUp = 1;
				break;
			case 'X' :
				speedDown = 1;
				break;
			// C: 우회전  S: 좌회전
			case 'C' :
				right = 1;
				break;
			case 'S' :
				left = 1;
				break;
			case 'D' :
				noDir = 1;
				break;
			// 0: 패드에서 손 뗐을 때 정지
			case '0' :
				stop = 1;
				break;
			default:
			break;
		}
	}
	HAL_UART_Receive_IT(&huart1, &serial_RxData, 1);
	}
}

void bluetoothControl()
{
  motor_init();

  HAL_UART_Receive_IT(&huart1, &serial_RxData, 1);
  HAL_UART_Receive_IT(&huart2, &serial_RxData, 1);
  while (1)
  {
  	// pwm CCR monitoring moserial
  	pwm_sweep_test();

  	  // 좌측 패드 화살표 방향
  	if(forward) // 직진
  	{
  		motor_forward();
  	}
  	if(backward) // 후진
  	{
  		motor_backward();
  	}
  	if((forward || backward) && noDir)
  	{
  		motor_recover();
  		right = 0;
  		left = 0;
  		noDir = 0;
  	}
  	if(right)
  	{
  		motor_left_speedUp();
  	}
  	if(left)
  	{
  		motor_right_speedUp();
  	}
  	if(stop)
  	{
  		motor_stop();
  		forward = 0;
  		backward = 0;
  		stop = 0;
  	}
  }
}

