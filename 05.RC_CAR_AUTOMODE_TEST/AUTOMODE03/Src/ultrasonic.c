/*
 * ultrasonic.c
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */


#include "ultrasonic.h"

// ==== 공통 배열(volatile: ISR-메인 공유) ====
typedef enum { US_LEFT=0, US_RIGHT=1, US_CENTER=2, US_NUM=3 } us_idx_t;

static volatile uint8_t  captureFlag[US_NUM] = {0};    // 0:rising대기, 1:falling대기, 2:완료
static volatile uint16_t IC_Value_1[US_NUM]    = {0};
static volatile uint16_t IC_Value_2[US_NUM]    = {0};
static volatile uint16_t echoTime[US_NUM] = {0};
static volatile uint16_t distance_cm[US_NUM] = {0};  // ★ uint16_t로 확장
static volatile uint16_t filter_distance_cm[US_NUM] = {0};

// 채널/IT 매핑
static const uint32_t CHANNEL[US_NUM] = { TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3 };

static inline uint32_t IT_FROM_CHANNEL(uint32_t channel)
{
    switch (channel)
    {
        case TIM_CHANNEL_1: return TIM_IT_CC1;
        case TIM_CHANNEL_2: return TIM_IT_CC2;
        case TIM_CHANNEL_3: return TIM_IT_CC3;
        case TIM_CHANNEL_4: return TIM_IT_CC4;
        default: return 0;
    }
}

// 콜백의 active-channel -> 인덱스
static inline int idx_from_active(HAL_TIM_ActiveChannel active) {
    if (active == HAL_TIM_ACTIVE_CHANNEL_2) return US_LEFT;
    if (active == HAL_TIM_ACTIVE_CHANNEL_1) return US_RIGHT;
    if (active == HAL_TIM_ACTIVE_CHANNEL_3) return US_CENTER;
    return -1;
}

// TRIG 포트/핀 모음 (기존 define 재사용)
static GPIO_TypeDef* const TRIG_PORT[US_NUM] = { TRIG_PORT_LEFT, TRIG_PORT_RIGHT, TRIG_PORT_CENTER };
static const uint16_t      TRIG_PIN [US_NUM] = { TRIG_PIN_LEFT,  TRIG_PIN_RIGHT,  TRIG_PIN_CENTER  };

static void HCSR04_Trigger(us_idx_t i)
{
    uint32_t ch_it = IT_FROM_CHANNEL(CHANNEL[i]);

    // (1) CC 플래그/오버캡처 플래그 클리어
    __HAL_TIM_CLEAR_FLAG(&htim4, ch_it);                // CCxIF
    __HAL_TIM_CLEAR_FLAG(&htim4, (ch_it << 9));         // CCxOF (HAL 매크로가 다르면 TIM_FLAG_CC1OF 등으로)

    // (2) 다음 측정은 Rising부터
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, CHANNEL[i], TIM_INPUTCHANNELPOLARITY_RISING);
    captureFlag[i] = 0;

    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_RESET);

    // (4) 채널 인터럽트 Enable
    __HAL_TIM_ENABLE_IT(&htim4, ch_it);
}

// HCSR04_Trigger 랩핑
void HCSR04_TRIGGER_LEFT()   { HCSR04_Trigger(US_LEFT); }
void HCSR04_TRIGGER_RIGHT()  { HCSR04_Trigger(US_RIGHT); }
void HCSR04_TRIGGER_CENTER() { HCSR04_Trigger(US_CENTER); }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Instance != TIM4) return;

    int i = idx_from_active(htim->Channel);
    if (i < 0) return;

    if (captureFlag[i] == 0) {
    	IC_Value_1[i] = HAL_TIM_ReadCapturedValue(htim, CHANNEL[i]);
    	captureFlag[i] = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, CHANNEL[i], TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (captureFlag[i] == 1) {
    	IC_Value_2[i] = HAL_TIM_ReadCapturedValue(htim, CHANNEL[i]);
    	captureFlag[i] = 2; // 완료
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, CHANNEL[i], TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

static inline uint16_t getTimeDifference(uint16_t start, uint16_t end)
{
    return (uint16_t)(end - start);  // 오버플로 자동 처리됨
}

static void process_one(TIM_HandleTypeDef *htim, us_idx_t i)
{
    if (captureFlag[i] == 2) {
    	echoTime[i] = getTimeDifference(IC_Value_1[i], IC_Value_2[i]);   // 16-bit 래핑 보정
      distance_cm[i] = echoTime[i] / 58;

        captureFlag[i] = 0;
        __HAL_TIM_DISABLE_IT(htim, IT_FROM_CHANNEL(CHANNEL[i]));
    }
}

void processUltrasonic_All()
{
    process_one(&htim4, US_LEFT);
    process_one(&htim4, US_RIGHT);
    process_one(&htim4, US_CENTER);
}

// ultrasonic.c
void US_Init()
{
    // 부팅 직후 0(미검출)로 멈추지 않게 큰 값으로 초기화
    distance_cm[US_LEFT]   = 400;
    distance_cm[US_RIGHT]  = 400;
    distance_cm[US_CENTER] = 400;
}

void US_Update()
{
    static uint32_t last_ms = 0;
    static uint8_t  idx = 0;           // 0:L, 1:R, 2:C
    static uint8_t  frame_mask = 0;    // bit0=L, bit1=R, bit2=C 갱신표시

    uint32_t now = HAL_GetTick();

    // 1) 먼저 콜백이 쌓아둔 완료건 처리 → distance_cm[] 갱신
    processUltrasonic_All();

    // 2) 이번 사이클에 새로 완료된 채널 표시
    for (uint8_t i = 0; i < US_NUM; i++) {
        // process_one()이 끝나면 captureFlag[i]가 0으로 돌아감
        // echoTime[i]가 0이 아니면 최소 1번은 유효 측정이 있었던 것으로 간주
        if (captureFlag[i] == 0 && echoTime[i] != 0) {
            frame_mask |= (1u << i);
        }
    }

    // 3) 가드타임 지나면 다음 채널만 트리거 (동시 트리거 금지)
    if ((uint32_t)(now - last_ms) >= 60u) {
        if      (idx == US_LEFT)   HCSR04_TRIGGER_LEFT();
        else if (idx == US_RIGHT)  HCSR04_TRIGGER_RIGHT();
        else                       HCSR04_TRIGGER_CENTER();

        idx = (uint8_t)((idx + 1u) % US_NUM);
        last_ms = now;
    }

    // 4) 좌/우/중 모두 새 데이터가 모이면 필터 1회만 수행
    if ((frame_mask & 0x07u) == 0x07u) {
        filter();                  // distance_cm[0..2] 기반으로 필터링
        frame_mask = 0u;           // 다음 프레임 준비
    }
}

void filter()
{
	filter_distance_cm[US_LEFT] = distance_cm[US_LEFT];
	filter_distance_cm[US_RIGHT] = distance_cm[US_RIGHT];
	filter_distance_cm[US_CENTER] = distance_cm[US_CENTER];

	if (distance_cm[US_LEFT] > 58)
	{
		filter_distance_cm[US_LEFT] = 58;
	}
	if (distance_cm[US_RIGHT] > 58)
	{
		filter_distance_cm[US_RIGHT] = 58;
	}
}

// distance 반환
uint16_t US_Left_cm()   { return filter_distance_cm[US_LEFT]; }
uint16_t US_Right_cm()  { return filter_distance_cm[US_RIGHT]; }
uint16_t US_Center_cm() { return filter_distance_cm[US_CENTER]; }
