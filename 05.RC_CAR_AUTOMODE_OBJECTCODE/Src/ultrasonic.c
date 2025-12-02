/*
 * ultrasonic.c
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */

#include "ultrasonic.h"
#include "delay_us.h"
#include <string.h> // memset 사용을 위해
#include <stdint.h>
#include <stdbool.h>

// ===== 튜닝/가드 =====
#define MAX_VALID_CM       400u   // 상한(환경에 맞춰 조정)
#define MIN_VALID_CM         2u   // 너무 작은 쓰레기 펄스 제거
#define FRAME_TIMEOUT_MS    80u   // 2m 환경 기준, 프레임 타임아웃
#define TRIG_GAP_MS         40u   // 2m 환경: 충분한 센서 간격 (L-C-R-C)


// 1. 전역 객체 생성 (배열 대신 이것들을 씁니다!)
Sonar_t sonarLeft = {
    .timer = &htim4,
    .channel = TIM_CHANNEL_2,        // Left는 CH2
    .clear_flag_bit = TIM_FLAG_CC2OF,
    .trig_port = TRIG_PORT_LEFT,     // 기존 define 활용
    .trig_pin  = TRIG_PIN_LEFT
};

Sonar_t sonarRight = {
    .timer = &htim4,
    .channel = TIM_CHANNEL_1,        // Right는 CH1
    .clear_flag_bit = TIM_FLAG_CC1OF,
    .trig_port = TRIG_PORT_RIGHT,
    .trig_pin  = TRIG_PIN_RIGHT
};

Sonar_t sonarCenter = {
    .timer = &htim4,
    .channel = TIM_CHANNEL_3,        // Center는 CH3
    .clear_flag_bit = TIM_FLAG_CC3OF,
    .trig_port = TRIG_PORT_CENTER,
    .trig_pin  = TRIG_PIN_CENTER
};

// 2. 매니저 변수 (구조체에 포함 안 된 녀석들)
static volatile uint8_t frame_mask = 0;
// static uint32_t frame_start_ms = 0; // 필요하면 유지

// [트리거 함수] 특정 소나 객체 하나를 동작시킴
void Sonar_Trigger(Sonar_t *sonar)
{
    // 1. 기존 플래그/상태 클리어
    __HAL_TIM_CLEAR_FLAG(sonar->timer, sonar->clear_flag_bit);
    // 오버캡처 플래그도 안전하게 지워줌 (옵션)
    __HAL_TIM_CLEAR_FLAG(sonar->timer, TIM_FLAG_CC1OF | TIM_FLAG_CC2OF | TIM_FLAG_CC3OF | TIM_FLAG_CC4OF);

    // 2. 상태 초기화
    sonar->state = 0; // Rising 대기 상태로

    // 3. 캡처 극성을 Rising으로 설정
    __HAL_TIM_SET_CAPTUREPOLARITY(sonar->timer, sonar->channel, TIM_INPUTCHANNELPOLARITY_RISING);

    // 4. 인터럽트 활성화
    __HAL_TIM_ENABLE_IT(sonar->timer, sonar->channel);

    // 5. 트리거 펄스 발사 (10us)
    HAL_GPIO_WritePin(sonar->trig_port, sonar->trig_pin, GPIO_PIN_RESET);
    // delay_us(1); // 필요시
    HAL_GPIO_WritePin(sonar->trig_port, sonar->trig_pin, GPIO_PIN_SET);
    delay_us(10);   // 10us 유지
    HAL_GPIO_WritePin(sonar->trig_port, sonar->trig_pin, GPIO_PIN_RESET);
}

// [인터럽트 로직] 한 센서의 Rising/Falling 처리 및 거리 계산
void Sonar_ISR_Process(Sonar_t *sonar)
{
    if (sonar->state == 0) {
        // [Rising Edge] 시작 시간 기록
        sonar->ic_start = HAL_TIM_ReadCapturedValue(sonar->timer, sonar->channel);
        sonar->state = 1; // Falling 대기 상태로 변경
          // 극성 반전: Rising -> Falling 감지하도록 변경
        __HAL_TIM_SET_CAPTUREPOLARITY(sonar->timer, sonar->channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (sonar->state == 1) {
        // [Falling Edge] 종료 시간 기록 -> 거리 계산
        sonar->ic_end = HAL_TIM_ReadCapturedValue(sonar->timer, sonar->channel);

        // 시간 차이 계산 (Timer Overflow 고려)
        uint32_t diff;
        if (sonar->ic_end >= sonar->ic_start) {
            diff = sonar->ic_end - sonar->ic_start;
        } else {
            // 타이머가 한 바퀴 돌았을 경우 (16bit 가정: 0xFFFF)
            // 타이머가 16비트인지 32비트인지에 따라 0xFFFF 혹은 0xFFFFFFFF로 수정 필요
            diff = (0xFFFF - sonar->ic_start) + sonar->ic_end + 1;
        }

        // 거리 계산 (소리 속도 340m/s -> 1us당 0.034cm -> 왕복이니까 나누기 2 -> 0.017)
        // 1us tick 기준
        sonar->distance = (float)diff * 0.017f;

        // 측정 완료! 인터럽트 끄기 (다음 트리거 때 다시 켬)
        __HAL_TIM_DISABLE_IT(sonar->timer, sonar->channel);
        sonar->state = 2; // 완료 상태
    }
}

// [글로벌 콜백] 하드웨어 인터럽트가 발생하면 여기서 분배함
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    // TIM4에서 온 인터럽트인가?
    if (htim->Instance == TIM4)
    {
        // 채널별로 담당 객체를 호출
        if (htim->Channel == sonarLeft.channel) {
            Sonar_ISR_Process(&sonarLeft);
        }
        else if (htim->Channel == sonarRight.channel) {
            Sonar_ISR_Process(&sonarRight);
        }
        else if (htim->Channel == sonarCenter.channel) {
            Sonar_ISR_Process(&sonarCenter);
        }
    }
}

float Sonar_Get_Distance(Sonar_t *sonar)
{
	return (sonar -> distance);
}
