/*
 * bluetooth.c
 *
 *  Created on: Oct 28, 2025
 *      Author: user13
 */


#include "bluetooth.h"


uint8_t serial_RxData;
volatile uint8_t go=0, back=0, right=0, left=0;
volatile uint8_t speedUp=0, speedDown=0;
volatile uint8_t goRight=0, goLeft=0, noDir=0, stop=0;

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
				go = 1;
				break;
			case 'B' :
				back = 1;
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
				goRight = 1;
				break;
			case 'S' :
				goLeft = 1;
				break;
			case 'D' :
				noDir = 1;
		  	goRight = 0;
		  	goLeft = 0;
				break;
			// 0: 패드에서 손 뗐을 때 정지
			case '0' :
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

  motor_init();

  HAL_UART_Receive_IT(&huart1, &serial_RxData, 1);

  while (1)
  {
  	// pwm CCR monitoring moserial
  	pwm_sweep_test();

  	  // 좌측 패드 화살표 방향
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
  		motor_left();
  	}

    	// 우측패드 세모: speed Up, 엑스: speed Down
  	if(speedUp)
  	{
  		motor_speedUp();
  	}
  	else if(speedDown)
  	{
  		motor_speedDown();
  	}
   	// 우측패드 원: 우회전, 네모: 좌회전
  	if((go||back) && goRight)
  	{
  	 motor_goRight();
  	}
  	else if((go||back) && goLeft)
  	{
  	 motor_goLeft();
  	}
  	else if(noDir)
  	{
     motor_recover();
  	 noDir = 0;
  	}

   	// 좌측패드 손 뗐을 때 정지 및 플래그 초기화
  	if(stop)
  	{
  	 motor_stop();
  	 go = 0;
  	 back = 0;
  	 right = 0;
  	 left = 0;
  	 speedUp = 0;
  	 speedDown = 0;
  	 stop = 0;
  	}
  }
}

