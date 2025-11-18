/*
 * ultrasonic.c
 *
 *  Created on: Nov 4, 2025
 *      Author: user13
 */
#include "ultrasonic.h"
#include "string.h"
#include "math.h"

// ==== 외부 심볼 ====
extern TIM_HandleTypeDef htim4;

// 채널 매핑 (TIM4)
static const uint32_t CHN[US_NUM] = { TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_1 };

static inline uint32_t IT_FROM_CH(uint32_t ch){
    switch(ch){
    case TIM_CHANNEL_1: return TIM_IT_CC1;
    case TIM_CHANNEL_2: return TIM_IT_CC2;
    case TIM_CHANNEL_3: return TIM_IT_CC3;
    case TIM_CHANNEL_4: return TIM_IT_CC4;
    default: return 0;
    }
}
static inline int idx_from_active(HAL_TIM_ActiveChannel a){
    if (a==HAL_TIM_ACTIVE_CHANNEL_2) return US_LEFT;
    if (a==HAL_TIM_ACTIVE_CHANNEL_3) return US_CENTER;
    if (a==HAL_TIM_ACTIVE_CHANNEL_1) return US_RIGHT;
    return -1;
}

static GPIO_TypeDef* const TRIG_PORT[US_NUM] = { TRIG_PORT_LEFT, TRIG_PORT_CENTER, TRIG_PORT_RIGHT };
static const uint16_t      TRIG_PIN [US_NUM] = { TRIG_PIN_LEFT,  TRIG_PIN_CENTER,  TRIG_PIN_RIGHT  };

// 캡처/결과 버퍼
static volatile uint8_t  capFlag[US_NUM]; // 0:RISING   1:FALLING   2:DONE
static volatile uint16_t ic1[US_NUM], ic2[US_NUM];
static volatile uint16_t echo_us[US_NUM];
static volatile uint16_t cm_raw[US_NUM];
static volatile uint8_t  new_raw[US_NUM];

// 순차 스케줄
static uint32_t last_ms;
static uint8_t  phase;
static uint32_t GUARD_MS = 10;

// 필터/게이트 파라미터
static float   side_angle_deg = 30.0f;
static float   side_cos       = 0.8660254f; // cos(30°)
static float   corridor_cm    = 50.0f;      // 수평 통로 폭
static float   side_raw_gate  = 58.0f;      // 50/cos(30°) ≈ 57.7

// 이동평균
#define MA_MAX 9
static uint8_t  sma_win = 5;  // 3~9
static uint16_t stale_ms = 100;

typedef struct {
    float buf[MA_MAX];
    uint8_t idx, cnt;
    uint32_t last_ms;
} sma_t;

static sma_t smaL, smaF, smaR;

// 출력 프레임
static us_frame_t last_fr = {0};
static uint8_t    new_fr  = 0;

// ==== 유틸 ====
static inline uint16_t dt16(uint16_t s, uint16_t e){ return (uint16_t)(e - s); }
static inline void trig_one(us_idx_t i){
    uint32_t it = IT_FROM_CH(CHN[i]);
    __HAL_TIM_CLEAR_FLAG(&htim4, it);
    __HAL_TIM_CLEAR_FLAG(&htim4, (it<<9)); // CCxOF

    __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, CHN[i], TIM_INPUTCHANNELPOLARITY_RISING);
    capFlag[i] = 0;

    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_RESET);
    for(volatile int d=0; d<100; ++d) __NOP(); // ~1us
    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_SET);
    // 10us
    for(volatile int d=0; d<1200; ++d) __NOP();
    HAL_GPIO_WritePin(TRIG_PORT[i], TRIG_PIN[i], GPIO_PIN_RESET);

    __HAL_TIM_ENABLE_IT(&htim4, it);
}

static inline void sma_push(sma_t *m, float v, uint8_t valid)
{
    if (!valid) return;
    if (m->cnt < sma_win) m->cnt++;
    m->buf[m->idx] = v;
    m->idx = (m->idx + 1) % sma_win;
    m->last_ms = HAL_GetTick();
}
static inline float sma_read(const sma_t *m)
{
    if (m->cnt == 0) return -1.0f;
    if ((int32_t)(HAL_GetTick() - m->last_ms) > (int32_t)stale_ms) return -1.0f;
    float s=0; for (uint8_t i=0;i<m->cnt;i++) s+=m->buf[i];
    return s / (float)m->cnt;
}

// ==== 콜백(IC) ====
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM4) return;
    int i = idx_from_active(htim->Channel);
    if (i<0) return;

    if (capFlag[i]==0){
        ic1[i] = HAL_TIM_ReadCapturedValue(htim, CHN[i]);
        capFlag[i]=1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, CHN[i], TIM_INPUTCHANNELPOLARITY_FALLING);
    }else if (capFlag[i]==1){
        ic2[i] = HAL_TIM_ReadCapturedValue(htim, CHN[i]);
        echo_us[i] = dt16(ic1[i], ic2[i]);
        cm_raw[i]  = (uint16_t)(echo_us[i] / 58u);
        new_raw[i] = 1;
        capFlag[i]=2;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, CHN[i], TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

// ==== 공개 ====
void US_Init(float angle_deg, float corridor_width, uint8_t sma_window, uint16_t stale_time_ms)
{
    side_angle_deg = angle_deg;
    side_cos       = cosf(angle_deg * (float)M_PI / 180.0f);
    corridor_cm    = corridor_width;
    side_raw_gate  = corridor_cm / side_cos; // raw <= 이 값이면 "측면 50cm 이하"로 간주

    sma_win = (sma_window==0)?5 : (sma_window>MA_MAX?MA_MAX:sma_window);
    stale_ms = stale_time_ms;

    memset((void*)capFlag, 0, sizeof(capFlag));
    memset((void*)new_raw, 0, sizeof(new_raw));
    phase=0; last_ms=HAL_GetTick();
    memset(&smaL,0,sizeof(smaL));
    memset(&smaF,0,sizeof(smaF));
    memset(&smaR,0,sizeof(smaR));
    memset(&last_fr,0,sizeof(last_fr));
    new_fr=0;
}

void US_Update(void)
{
    // 1) 순차 트리거 (10ms 가드)
    uint32_t now = HAL_GetTick();
    if (now - last_ms >= GUARD_MS) {
        switch(phase){
            case 0: trig_one(US_CENTER); break;
            case 1: trig_one(US_LEFT);   break;
            case 2: trig_one(US_CENTER); break;
            case 3: trig_one(US_RIGHT);  break;
        }
        phase = (phase+1) & 3;
        last_ms = now;
    }

    // 2) 캡처 완료 샘플 소비 → 게이트 → SMA
    uint16_t L = cm_raw[US_LEFT];
    uint16_t F = cm_raw[US_CENTER];
    uint16_t R = cm_raw[US_RIGHT];

    uint8_t vL=0, vF=0, vR=0;

    if (new_raw[US_LEFT])  { new_raw[US_LEFT]=0;
        vL = (L>0 && L <= (uint16_t)(side_raw_gate+0.5f)); // 58cm
        sma_push(&smaL, (float)L, vL);
    }
    if (new_raw[US_CENTER]){ new_raw[US_CENTER]=0;
        vF = (F>0); // 전방은 임계 생략(원하면 상한 추가)
        sma_push(&smaF, (float)F, vF);
    }
    if (new_raw[US_RIGHT]) { new_raw[US_RIGHT]=0;
        vR = (R>0 && R <= (uint16_t)(side_raw_gate+0.5f));
        sma_push(&smaR, (float)R, vR);
    }

    // 3) SMA 결과로 프레임 구성
    us_frame_t fr = {0};
    float fL = sma_read(&smaL);
    float fF = sma_read(&smaF);
    float fR = sma_read(&smaR);

    if (fL>0) { fr.cmL=(uint16_t)(fL+0.5f); fr.validMask|= (1<<0); }
    if (fF>0) { fr.cmF=(uint16_t)(fF+0.5f); fr.validMask|= (1<<1); }
    if (fR>0) { fr.cmR=(uint16_t)(fR+0.5f); fr.validMask|= (1<<2); }
    fr.stamp_ms = HAL_GetTick();

    // 새 샘플 판단: 이전과 시각/마스크/값이 다르면 new
    if (fr.stamp_ms != last_fr.stamp_ms ||
        fr.validMask != last_fr.validMask ||
        fr.cmL != last_fr.cmL || fr.cmF != last_fr.cmF || fr.cmR != last_fr.cmR)
    {
        last_fr = fr;
        new_fr  = 1;
    }
}

bool US_GetFrame(us_frame_t *out)
{
    if (!out) return false;
    *out = last_fr;
    uint8_t was_new = new_fr;
    new_fr = 0;
    return was_new != 0;
}
