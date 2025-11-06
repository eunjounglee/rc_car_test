/*
 * automode.c — Simplest rule: front-block turn, else creep-forward
 */

#include "automode.h"
#include "ultrasonic.h"
#include "tim.h"

// ----- Tuning (simple) -----
#define MIN_VALID_CM     5     // 초근접/미검출 보호
#define FRONT_BLOCK_CM  35     // 전방이 이 값 이하이면 '벽 있음'
#define SIDE_CLEAR_CM   30     // 옆이 이 값 이상이면 '비어있음'

// 저속 크리프 속도 (ARR가 충분히 커야 함)
#define CREEP_RIGHT    520
#define CREEP_LEFT     520
#define TURN_RIGHT_IN  500     // 우회전 시 안쪽(오른쪽) 바퀴
#define TURN_RIGHT_OUT 500     // 우회전 시 바깥(왼쪽) 바퀴
#define TURN_LEFT_IN   500     // 좌회전 시 안쪽(왼쪽)
#define TURN_LEFT_OUT  500     // 좌회전 시 바깥(오른쪽)

// HAL compare helper
extern TIM_HandleTypeDef htim3;
#define SET_RIGHT(val)  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (val))
#define SET_LEFT(val)   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,  (val))

void AutoMode_Init(void)
{
    motor_init(); // 방향핀/EN핀/PWM start는 이 안에서 해준다고 가정
}

static inline void set_speed(uint16_t right, uint16_t left)
{
    SET_RIGHT(right);
    SET_LEFT(left);
}

// 가장 단순한 룰 기반 제어
static void AutoMode_Update_Simple()
{
    uint16_t L = US_Left_cm();
    uint16_t R = US_Right_cm();
    uint16_t C = US_Center_cm();

    // 미검출 보호: 0~2cm 튀는 값은 큰 값으로 치환(정지 남발 방지)
    if (C < MIN_VALID_CM) C = 400;

    // 1) 앞에 벽 + 오른쪽이 비었으면 → 우회전(제자리 피벗)
    if (C <= FRONT_BLOCK_CM)
    	{
    	if(R >= SIDE_CLEAR_CM && L < SIDE_CLEAR_CM)
    	{
        left_dir_forward();
        right_dir_backward();
        set_speed(0, 400);
        return;
       }
    	}

    // 2) 앞에 벽 + 왼쪽이 비었으면 → 좌회전(제자리 피벗)
    if (C <= FRONT_BLOCK_CM)
    {
				if(L >= SIDE_CLEAR_CM && R < SIDE_CLEAR_CM)
			{
					left_dir_backward();
					right_dir_forward();
					set_speed(400, 0);
					return;
			}
    }

    // 3) 앞이 막혔는데 양옆도 막힘 → 정지
    if (C <= FRONT_BLOCK_CM)
    {
        motor_stop();
        return;
    }

    // 4) 앞이 비었으면 아주 느리게 직진
    motor_forward();
    set_speed(CREEP_RIGHT, CREEP_LEFT);
}

// FreeRTOS에서 기존대로 AutoMode_Update()를 부르면 동작하도록 래퍼 제공
void AutoMode_Update()
{
    AutoMode_Update_Simple();
}
