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
#include "stdbool.h"
#include "stdint.h"

// ==== 보드 핀====
#define TRIG_PORT_LEFT	  GPIOC
#define TRIG_PIN_LEFT	    GPIO_PIN_8
#define TRIG_PORT_RIGHT	  GPIOC
#define TRIG_PIN_RIGHT	  GPIO_PIN_5
#define TRIG_PORT_CENTER	GPIOC
#define TRIG_PIN_CENTER	  GPIO_PIN_6


typedef struct {
    uint16_t cmL, cmF, cmR; // 0이면 invalid
    uint8_t  validMask;     // bit0=L, bit1=F, bit2=R
    uint32_t stamp_ms;
} us_frame_t;

void US_Init(float side_angle_deg, float corridor_width_cm,
             uint8_t sma_win, uint16_t stale_ms);

/* 주기적으로 10~20ms마다 호출
 * 내부에서 순차 트리거 → 캡처 처리 → 30도 보정 게이트(<=58cm相当) → 이동평균(SMA)까지 수행
 */
void US_Update(void);

/* 최신 정제 프레임 복사.
 * 새 샘플이 있으면 true, 없으면 false (그래도 out엔 마지막 프레임을 채워줌)
 */
bool US_GetFrame(us_frame_t *out);

/* (선택) HC-SR04 트리거/IC 콜백을 이 모듈이 소유하도록 프로토타입만 공개 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);


void Ultrasonic_Step();

typedef enum { US_LEFT=0, US_RIGHT=1, US_CENTER=2, US_NUM=3 } us_idx_t;

bool US_TryGet_cm(us_idx_t idx, uint16_t *out_cm);
bool US_TryGetLeft_cm (uint16_t *cm);
bool US_TryGetRight_cm(uint16_t *cm);
bool US_TryGetFront_cm(uint16_t *cm);
bool US_ReadLatest_cm(us_idx_t idx, uint16_t *out_cm);   // ★ 추가


// ★ 캐시 읽기(항상 최신값 반환; 0이면 아직 초기화 전일 수 있음)
bool US_ReadLatest_cm(us_idx_t idx, uint16_t *out_cm);

// 편의 함수(옵션)
static inline void US_ReadAllLatest_cm(uint16_t *left, uint16_t *front, uint16_t *right)
{
    if (left)  (void)US_ReadLatest_cm(US_LEFT,   left);
    if (front) (void)US_ReadLatest_cm(US_CENTER, front);
    if (right) (void)US_ReadLatest_cm(US_RIGHT,  right);
}

#endif /* INC_ULTRASONIC_H_ */
