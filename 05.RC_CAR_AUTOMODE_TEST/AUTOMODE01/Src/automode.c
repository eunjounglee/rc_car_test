/*
 * automode.c  — Wall-Follow (Right/Left) + Corner Turn
 *  - PD steering on side distance
 *  - Front-based speed scheduling + TTC brake + E-STOP
 *  - Robust corner-turn state
 */

#include "automode.h"         // AutoMode_Init/Update 외부 인터페이스 선언
#include "ultrasonic.h"       // us_frame_t, US_GetFrame() (정제된 거리 프레임 제공)
#include "math.h"             // fabsf, fminf 등 수학 함수
#include "stdio.h"            // 디버깅용 printf (필요시만 사용)

// 벽 추종 기준 (오른쪽 벽)
#define FOLLOW_RIGHT  1       // 1이면 오른쪽 벽 추종, 0이면 왼쪽 벽 추종

// ── 튜닝 상수(단위 cm) ─────────────────────────────────────────────
static const float D_STOP = 25.0f;  // [전방] 이 거리부터 속도를 0→1로 선형 스케줄 (아래 D_SLOW 참조)
static const float D_SLOW = 80.0f;  // [전방] 이 거리 이상이면 최고 속도(1.0)로 제한 해제
static const float D_ESTOP = 5.0f; // [전방] 절대 정지 임계 (이하면 무조건 정지)

static const float KP = 0.050f;     // PD 제어의 P 게인 (측면 거리 오차 → 조향)
static const float KD = 0.120f;     // PD 제어의 D 게인 (오차 미분 → 조향 감쇠/선견)
static const float DS_MAX = 0.12f;  // 1주기당 S(조향) 변화량 제한 (램핑)
static const float DT_MAX = 0.06f;  // 1주기당 T(추력/속도) 변화량 제한 (가감속)

static const float SIDE_TARGET = 10.0f; // 목표 측면 거리 (벽과의 이상적 거리)
static const float SIDE_SAFE   = 22.0f; // 측면 안전 한계(이보다 가까우면 추가 페널티)
static const float SIDE_BIAS   = 0.8f;  // 가까울 때 한쪽으로 강하게 피하는 오프셋(조향 보정)
static const float SIDE_T_CAP  = 0.35f; // 가까울 때 전진 속도 상한 (T 최대값 제한)

static const float UMAX = 0.80f;        // PWM 듀티 상한(하드웨어 여유 확보용)

// ── 상태/변수 ─────────────────────────────────────────────────────────────
typedef enum { MODE_FOLLOW=0, MODE_TURN=1 } mode_t; // (현재 코드는 TURN 미사용, 확장 대비)
static mode_t mode;          // 현재 주행 모드
static float  S_now, T_now;  // 현재 조향/추력(램프 제한 적용된 내부 상태)
static float  e_prev;        // PD 제어용: 이전 오차(미분 계산에 사용)
static uint32_t last_ms;     // 이전 루프 시간(ms) → dt 계산에 사용

// clamp 유틸 (a~b로 값 제한) — 인라인이라 오버헤드 적음
static inline float clampf(float v,float a,float b){ return v<a?a:(v>b?b:v); }

// ── 모터 믹싱 (스키드-스티어) ─────────────────────────────────────────────
// 입력: T ∈ [-1,1] (후진..전진), S ∈ [-1,1] (좌..우)
// 출력: 각 바퀴의 목표 속도/방향(vR/vL) → 방향핀 전환 + 듀티 설정
static void motor_applyMix(float T, float S)
{
    T = clampf(T,-1,1);              // 안전: T를 범위 제한
    S = clampf(S,-1,1);              // 안전: S를 범위 제한
    float vR = clampf(T+S,-1,1),     // 우측 바퀴 목표 (전진+조향)
          vL = clampf(T-S,-1,1);     // 좌측 바퀴 목표 (전진-조향)
    const float dead=0.05f;          // 미세 떨림 방지 데드밴드

    if (fabsf(vR)<dead) vR=0;        // 데드밴드 내면 0으로 스냅
    if (fabsf(vL)<dead) vL=0;

    // 방향 전환 감지(부호 변화): 부호가 바뀔 때만 방향핀 갱신 (쇼트스루 예방)
    static int8_t lastR=0,lastL=0;   // 지난 루프에서의 부호(1,0,-1)
    int8_t sR=(vR>0)-(vR<0),         // C식 부호 계산: 양수→1, 0→0, 음수→-1
          sL=(vL>0)-(vL<0);

    if (sR!=lastR || sL!=lastL){     // 어느 쪽이든 부호가 바뀌면
        automode_motor_setDuty(0,0); // 1) 듀티 0으로 먼저 내리고
        if(vR>0) right_dir_forward();    // 2) 우측 방향핀 전진
        else if(vR<0) right_dir_backward(); //    또는 후진
        if(vL>0) left_dir_forward();     // 3) 좌측 방향핀 전진
        else if(vL<0) left_dir_backward();  //    또는 후진
        lastR=sR; lastL=sL;            // 4) 상태 갱신
    }

    // 듀티(절대값) 적용: 하드웨어 상한(UMAX) 내에서
    float uR=clampf(fabsf(vR),0,UMAX), // NOTE: 프로젝트에 clampf 로컬 정의 필요(링커 에러 방지)
          uL=clampf(fabsf(vL),0,UMAX);
    automode_motor_setDuty(uR,uL);     // 실제 PWM 듀티 출력 (의존: speed 모듈 래핑)
}

// ── 전방 거리 → 전진 추력 스케줄러 ─────────────────────────────────────────
//  D_ESTOP 이하면 무조건 정지, D_STOP~D_SLOW 사이를 0→1로 스케일(여기선 x^2로 부드럽게)
static float forward_from_front(float dF)
{
    if (dF<0) return 0.0f;                 // dF<0: 유효 데이터 없음 → 보수적으로 0
    if (dF<=D_ESTOP) return 0.0f;          // 절대 정지 구간

    float x=(dF-D_STOP)/(D_SLOW-D_STOP);   // 0~1로 정규화(선형)
    float T=(x<=0)?0: (x>=1?1:(x*x));      // 0~1 구간만 사용, 감도 완화 위해 제곱
    return T;
}

// ── 측면 거리 → 조향(PD) ─────────────────────────────────────────────────
// FOLLOW_RIGHT가 1이면 오른쪽 센서(dR)로 제어, 0이면 왼쪽(dL)으로 제어
static float steering_from_side(float dL, float dR, float dt)
{
#if FOLLOW_RIGHT
    float dS = dR; int sign = -1;          // 오른쪽이 멀수록 "오른쪽으로 붙게" 음의 S를 가산(차체 기준)
#else
    float dS = dL; int sign = +1;          // 왼쪽 추종 시 부호 반대
#endif
    if (dS<0) return 0.0f;                 // 유효 데이터 없으면 조향 0 (직진)

    float e  = (dS - SIDE_TARGET);         // 현재 측면 거리 - 목표 측면 거리 = 오차
    float de = (dt>1e-3f)? (e - e_prev)/dt // 오차의 시간 미분(속도): dt가 너무 작을 땐 0으로
                                            : 0.0f;
    e_prev = e;                             // 다음 루프를 위한 e_prev 갱신

    float S = sign * (KP*e + KD*de);       // PD 제어 (부호로 좌/우 체계 전환)
    return clampf(S,-1,1);                 // 안전 범위 제한
}

// ── 모드 초기화 ───────────────────────────────────────────────────────────
void AutoMode_Init(void)
{
    mode = MODE_FOLLOW;                     // 기본 모드: 벽 추종
    S_now=T_now=0.0f;                       // 조향/추력 상태 초기화
    e_prev=0.0f;                            // PD 미분 초기화
    last_ms = HAL_GetTick();                // 초기 시간 기준

    motor_init();                           // 의존: 방향핀 초기화(네 프로젝트의 move.c)
    automode_motor_speedInit();             // 의존: PWM 타이머 시작(네 프로젝트의 speed.c 래퍼)
    automode_motor_setDuty(0,0);            // 안전: 듀티 0으로 시작
}

// ── 주기 업데이트 (10~20 ms) ─────────────────────────────────────────────
void AutoMode_Update(void)
{
    // 1) 정제된 프레임 읽기 (ultrasonic 모듈이 내부에서 게이트/평활 완료)
    us_frame_t fr; (void)US_GetFrame(&fr);   // 새 여부 상관 없이 최신 프레임 복사
    float dL = (fr.validMask&(1<<0))? fr.cmL : -1.0f; // 유효면 값, 아니면 -1
    float dF = (fr.validMask&(1<<1))? fr.cmF : -1.0f;
    float dR = (fr.validMask&(1<<2))? fr.cmR : -1.0f;

    // 2) dt 계산(초 단위). 동일 tick이면 0.01s로 보호(미분 폭주 방지)
    uint32_t now=HAL_GetTick();
    float dt = (now==last_ms)? 0.01f : (now-last_ms)/1000.0f;
    last_ms = now;

    // 3) 속도/조향 참조값 계산
    float T_ref = forward_from_front(dF);    // 전방 기반 전진 추력 스케줄
    float S_ref = steering_from_side(dL,dR,dt); // 측면 기반 조향(PD)

#if FOLLOW_RIGHT
    // 오른쪽 벽에 너무 가까우면(안전 한계) 강하게 반발 + 속도 상한 적용
    if (dR>0 && dR<SIDE_SAFE) {
        S_ref -= SIDE_BIAS;                  // 오른쪽으로 치우치니 왼쪽으로 돌게(S 음수→왼쪽?)
        T_ref = fminf(T_ref, SIDE_T_CAP);    // 속도 캡으로 과도 접근 방지
    }
#else
    // 왼쪽 벽 추종일 때 대칭 처리
    if (dL>0 && dL<SIDE_SAFE) {
        S_ref += SIDE_BIAS;
        T_ref = fminf(T_ref, SIDE_T_CAP);
    }
#endif

    // 비상 정지(전방 절대 한계)
    if (dF>0 && dF<=D_ESTOP) T_ref=0.0f;

    // 4) 램핑(스무딩): 한 주기당 변화량 제한으로 진동/급가속 억제
    float dS=S_ref-S_now;
    if(dS>DS_MAX)dS=DS_MAX;
    if(dS<-DS_MAX)dS=-DS_MAX;
    S_now+=dS;

    float dT=T_ref-T_now;
    if(dT>DT_MAX)dT=DT_MAX;
    if(dT<-DT_MAX)dT=-DT_MAX;
    T_now+=dT;

    // 5) 모터 적용 (방향핀 전환 보호 포함)
    motor_applyMix(T_now, S_now);

     // 디버깅 원하면 활성화:
//     printf("[AM] L=%u F=%u R=%u | T=%.2f S=%.2f\n", fr.cmL, fr.cmF, fr.cmR, T_now, S_now);
}
