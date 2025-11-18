/*
 * ultrasonic.c
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */

#include "ultrasonic.h"
#include <stdint.h>
#include <stdbool.h>

// ===== 튜닝/가드 =====
#define MAX_VALID_CM       400u   // 상한(환경에 맞춰 조정)
#define MIN_VALID_CM         2u   // 너무 작은 쓰레기 펄스 제거
#define MEDIAN_WIN           3u   // 3 또는 5 권장
#define FRAME_TIMEOUT_MS    80u   // 2m 환경 기준, 프레임 타임아웃
#define TRIG_GAP_MS         40u   // 2m 환경: 충분한 센서 간격 (L-C-R-C)

// ==== 공통 배열(volatile: ISR-메인 공유) ====
typedef enum { US_LEFT=0, US_RIGHT=1, US_CENTER=2, US_NUM=3 } us_idx_t;

static volatile uint8_t  captureFlag[US_NUM]   = {0};   // 0:rising대기, 1:falling대기, 2:완료
static volatile uint16_t IC_Value_1[US_NUM]    = {0};
static volatile uint16_t IC_Value_2[US_NUM]    = {0};
static volatile uint16_t echoTime[US_NUM]      = {0};   // 1 tick = 1us 전제
static volatile uint16_t distance_cm[US_NUM]   = {0};   // 원시 거리(필터 전)

// 미디언 필터 버퍼
static uint16_t med_buf[US_NUM][MEDIAN_WIN];
static uint8_t  med_wpos[US_NUM] = {0};
static uint8_t  med_filled[US_NUM] = {0};

// 필터 출력(최종 제공값)
static volatile uint16_t filter_distance_cm[US_NUM] = {400,400,400};

// 이번 라운드에서 새 샘플 완료 비트 (bit0=L, bit1=R, bit2=C)
static volatile uint8_t frame_mask = 0;

// 프레임 타이밍
static uint32_t frame_start_ms = 0;

// Center 신선도(옵션 개선 #5)
static uint8_t  c_valid_streak = 0;

// === TIM4 채널/IT 매핑 ===
static const uint32_t CHANNEL[US_NUM] = { TIM_CHANNEL_2, TIM_CHANNEL_1, TIM_CHANNEL_3 };

static inline uint32_t IT_FROM_CHANNEL(uint32_t channel)
{
    switch (channel) {
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

// TRIG 포트/핀 (기존 define 재사용)
static GPIO_TypeDef* const TRIG_PORT[US_NUM] = { TRIG_PORT_LEFT, TRIG_PORT_RIGHT, TRIG_PORT_CENTER };
static const uint16_t      TRIG_PIN [US_NUM] = { TRIG_PIN_LEFT,  TRIG_PIN_RIGHT,  TRIG_PIN_CENTER  };

static inline void clear_cc_flags_safely(uint32_t ch)
{
    // CCxIF
    __HAL_TIM_CLEAR_FLAG(&htim4, IT_FROM_CHANNEL(ch));
    // CCxOF (오버캡처) — 채널별 공식 플래그 사용
    if (ch == TIM_CHANNEL_1) __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC1OF);
    if (ch == TIM_CHANNEL_2) __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC2OF);
    if (ch == TIM_CHANNEL_3) __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC3OF);
    if (ch == TIM_CHANNEL_4) __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_CC4OF);
}

static void HCSR04_Trigger(us_idx_t i)
{
    uint32_t ch = CHANNEL[i];

    // (1) CC 플래그/오버캡처 플래그 안전 클리어
    clear_cc_flags_safely(ch);

    // (2) 다음 측정은 Rising부터
    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, ch, TIM_INPUTCHANNELPOLARITY_RISING);
    captureFlag[i] = 0;

    // (3) 10us TRIG 펄스
    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_RESET);
    delay_us(1);
    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_SET);
    delay_us(10);
    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_RESET);

    // (4) 채널 인터럽트 Enable
    __HAL_TIM_ENABLE_IT(&htim4, IT_FROM_CHANNEL(ch));
}

// HCSR04_Trigger 래퍼
void HCSR04_TRIGGER_LEFT(void)   { HCSR04_Trigger(US_LEFT); }
void HCSR04_TRIGGER_RIGHT(void)  { HCSR04_Trigger(US_RIGHT); }
void HCSR04_TRIGGER_CENTER(void) { HCSR04_Trigger(US_CENTER); }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) return;

    int i = idx_from_active(htim->Channel);
    if (i < 0) return;

    uint32_t ch = CHANNEL[i];

    if (captureFlag[i] == 0) {
        IC_Value_1[i] = HAL_TIM_ReadCapturedValue(htim, ch);
        captureFlag[i] = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else if (captureFlag[i] == 1) {
        IC_Value_2[i] = HAL_TIM_ReadCapturedValue(htim, ch);
        captureFlag[i] = 2; // 완료
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, ch, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

static inline uint16_t getTimeDifference(uint16_t start, uint16_t end)
{
    return (uint16_t)(end - start);  // 16-bit 래핑 보정
}

static void process_one(TIM_HandleTypeDef *htim, us_idx_t i)
{
    if (captureFlag[i] == 2) {
        echoTime[i] = getTimeDifference(IC_Value_1[i], IC_Value_2[i]); // tick=1us
        // 1tick=1us → 거리[cm] ≈ echo(us)/58
        uint16_t cm = (uint16_t)(echoTime[i] / 58u);

        // (4) 비정상 샘플 거르기 (0/과대값) — 이전값 유지
        if (cm >= MIN_VALID_CM && cm <= MAX_VALID_CM) {
            distance_cm[i] = cm;  // 정상값만 반영
        }

        captureFlag[i] = 0;
        __HAL_TIM_DISABLE_IT(htim, IT_FROM_CHANNEL(CHANNEL[i]));

        // 이번 라운드 갱신 완료 비트 설정
        frame_mask |= (1u << i);
    }
}

void processUltrasonic_All(void)
{
    process_one(&htim4, US_LEFT);
    process_one(&htim4, US_RIGHT);
    process_one(&htim4, US_CENTER);
}

// 초기화
void US_Init(void)
{
    // 부팅 직후 0(미검출)로 멈추지 않게 큰 값으로 초기화
    distance_cm[US_LEFT]   = 400u;
    distance_cm[US_RIGHT]  = 400u;
    distance_cm[US_CENTER] = 400u;
    filter_distance_cm[US_LEFT]   = 400u;
    filter_distance_cm[US_RIGHT]  = 400u;
    filter_distance_cm[US_CENTER] = 400u;
    frame_mask = 0u;
    frame_start_ms = 0u;
    c_valid_streak = 0u;
}

void US_FilterInit(void)
{
    for (int s = 0; s < US_NUM; ++s) {
        for (uint8_t k = 0; k < MEDIAN_WIN; ++k) med_buf[s][k] = 400u;
        med_wpos[s] = 0u;
        med_filled[s] = MEDIAN_WIN;
        filter_distance_cm[s] = 400u;
    }
}

// 아주 작은 N(<=5)에 적합한 삽입정렬 기반 미디안
static inline uint16_t median_of(uint16_t *arr, uint8_t n)
{
    for (uint8_t i = 1; i < n; ++i) {
        uint16_t key = arr[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            --j;
        }
        arr[j + 1] = key;
    }
    return arr[n / 2]; // n=3→idx1, n=5→idx2
}

static inline void median_push_sample(us_idx_t s, uint16_t sample)
{
    // 클램프 후 저장
    if (sample > MAX_VALID_CM) sample = MAX_VALID_CM;
    med_buf[s][med_wpos[s]] = sample;

    // 포인터 갱신
    med_wpos[s] = (uint8_t)((med_wpos[s] + 1u) % MEDIAN_WIN);
    if (med_filled[s] < MEDIAN_WIN) med_filled[s]++;

    // 중앙값 계산: 현재 버퍼 스냅샷 복사 후 median
    uint16_t tmp[MEDIAN_WIN];
    uint8_t n = med_filled[s];
    for (uint8_t i = 0; i < n; ++i) tmp[i] = med_buf[s][i];

    filter_distance_cm[s] = median_of(tmp, n);
}

static inline void filter_once(void)
{
    median_push_sample(US_LEFT,   distance_cm[US_LEFT]);
    median_push_sample(US_RIGHT,  distance_cm[US_RIGHT]);
    median_push_sample(US_CENTER, distance_cm[US_CENTER]);

    // Center 신선도 스트릭(옵션 개선 #5)
    uint16_t c = filter_distance_cm[US_CENTER];
    if (c >= MIN_VALID_CM && c <= MAX_VALID_CM) {
        if (c_valid_streak < 255) c_valid_streak++;
    } else {
        c_valid_streak = 0;
    }
}

void US_Update(void)
{
    static uint32_t last_ms = 0;

    uint32_t now = HAL_GetTick();

    // 1) 캡처 완료건 처리 → distance_cm[] 갱신
    processUltrasonic_All();

    // 2) 순차 트리거 (L-C-R-C)
    static const us_idx_t order[] = { US_LEFT, US_CENTER, US_RIGHT, US_CENTER };
    static uint8_t oidx = 0;

    if ((uint32_t)(now - last_ms) >= TRIG_GAP_MS) {
        HCSR04_Trigger(order[oidx]);
        oidx = (uint8_t)((oidx + 1u) % (sizeof(order)/sizeof(order[0])));
        last_ms = now;

        // 프레임 시작 타임스탬프 초기화(첫 트리거 시)
        if (frame_start_ms == 0u) frame_start_ms = now;
    }

    // 3) 프레임 완료 or 타임아웃 시 필터 수행
    bool frame_complete = ((frame_mask & 0x07u) == 0x07u);
    bool frame_timeout  = (frame_start_ms != 0u) && ((uint32_t)(now - frame_start_ms) >= FRAME_TIMEOUT_MS);

    if (frame_complete || frame_timeout) {
        filter_once();          // (2) 타임아웃이어도 사용 가능한 샘플로 필터 수행
        frame_mask = 0u;        // 다음 프레임 준비
        frame_start_ms = now;   // 새 프레임 시작
    }
}

// === distance 반환 API ===
uint16_t US_Left_cm(void)   { return filter_distance_cm[US_LEFT]; }
uint16_t US_Right_cm(void)  { return filter_distance_cm[US_RIGHT]; }
uint16_t US_Center_cm(void) { return filter_distance_cm[US_CENTER]; }

// (옵션) Center 신선도 — automode에서 급결정 시 사용 가능
bool US_Center_isFresh(void) { return c_valid_streak >= 2; }
