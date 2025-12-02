/* ultrasonic.h */
#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "main.h"

#define TRIG_PORT_LEFT	  GPIOC
#define TRIG_PIN_LEFT	    GPIO_PIN_8
#define TRIG_PORT_RIGHT	  GPIOC
#define TRIG_PIN_RIGHT	  GPIO_PIN_5
#define TRIG_PORT_CENTER	GPIOC
#define TRIG_PIN_CENTER	  GPIO_PIN_6

#define MEDIAN_WIN           3u   // 3 또는 5 권장

// [설계도] 소나(Sonar) 센서 객체 정의
typedef struct {
    // 1. 하드웨어 설정 (변하지 않음)
    TIM_HandleTypeDef *timer;        // 사용할 타이머 핸들 (&htim4)
    uint32_t          channel;       // 타이머 채널 (TIM_CHANNEL_1 등)
    uint32_t          clear_flag_bit;// 인터럽트 클리어 플래그 (TIM_FLAG_CC1OF 등)
    GPIO_TypeDef      *trig_port;    // Trig 포트 (GPIOA 등)
    uint16_t          trig_pin;      // Trig 핀 (GPIO_PIN_0 등)

    // 2. 측정 데이터 (계속 변함)
    volatile uint8_t  state;         // 0:대기, 1:Rising완료, 2:Falling완료
    volatile uint32_t ic_start;      // Rising 시점의 타이머 값
    volatile uint32_t ic_end;        // Falling 시점의 타이머 값
    volatile float    distance;      // 최종 계산된 거리 (cm)

    // 3. 필터용 버퍼 (개별 사물함)
    uint16_t med_buf[MEDIAN_WIN];    // 과거 값 저장용 배열
    uint8_t  med_head;               // 버퍼 인덱스
    uint8_t  med_count;              // 버퍼 채워진 개수

} Sonar_t;

// [함수 선언]
void Sonar_Init(void);
void Sonar_Trigger(Sonar_t *sonar);
void Sonar_ISR_Process(Sonar_t *sonar);
float Sonar_Get_Distance(Sonar_t *sonar);

// (전역 객체 접근용 - main이나 다른 곳에서 쓸 수 있게)
extern Sonar_t sonarLeft;
extern Sonar_t sonarRight;
extern Sonar_t sonarCenter;

#endif /* ULTRASONIC_H_ */
