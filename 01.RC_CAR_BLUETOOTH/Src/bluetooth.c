/*
 * bluetooth.c
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */


#include "bluetooth.h"


uint8_t serial_RxData;
uint8_t go = 0;
uint8_t back = 0;
uint8_t right = 0;
uint8_t left = 0;
uint8_t stop = 0;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
	HAL_UART_Receive_IT(&huart1, &serial_RxData, 1);
	}

	HAL_UART_Transmit (&huart1, &serial_RxData, 1, 1000);

	if(serial_RxData == 'F'||
		 serial_RxData == 'B'||
		 serial_RxData == 'R'||
		 serial_RxData == 'L'||
		 serial_RxData == 'P')
	{
		switch(serial_RxData)
		{
			case 'F' :
				go = 1;
				back = 0;
				right = 0;
				left = 0;
				stop = 0;
				break;
			case 'B' :
				go = 0;
				back = 1;
				right = 0;
				left = 0;
				stop = 0;
				break;
			case 'R' :
				go = 0;
				back = 0;
				right = 1;
				left = 0;
				stop = 0;
				break;
			case 'L' :
				go = 0;
				back = 0;
				right = 0;
				left = 1;
				stop = 0;
				break;
			case 'P' :
				go = 0;
				back = 0;
				right = 0;
				left = 0;
				stop = 1;
				break;
			default:
			break;
		}
	}
}

void bluetoothControl()
{
  MX_USART1_UART_Init();

  HAL_UART_Receive_IT(&huart1, &serial_RxData, 1);

  while (1)
  {
  	if(go)
  	{
  		motor_go();
  	}
  	else if(back)
  	{
  		motor_back();
  	}
  	else if(right)
  	{
  		motor_right();
  	}
  	else if(left)
  	{
  		motor_laft();
  	}
  	else if(stop)
  	{
  		motor_stop();
  	}
  }
}

