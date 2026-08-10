/*
 * BallSpeed.c — 球电机速度PID内环 + TIM8编码器测速
 *
 * 电机: JGB37-520, 11PPR霍尔编码器, 1:30减速比
 * 编码器: TIM8正交解码(TI12四倍频), PC6(TIM8_CH1=A相)+PC7(TIM8_CH2=B相)
 *         1320脉冲/输出轴圈 (11×30×4)
 * 采样:  TIM6 ISR 10ms (100Hz) 内调用 BallSpeed_Update()
 * 输出:  BallMoter_SetSpeed(-100~+100, 双向)
 *
 * 说明: 纯P控制（KI=0），保留静态误差，先验证编码器闭环再调参
 *       BallMoter 输出级带死区补偿，低速稳态附近可能出现小幅振荡
 */

#include "BallSpeed.h"
#include "PID.h"
#include "BallMoter.h"

/* 静态初始化：即使 TIM6 ISR 早于 BallSpeed_Init() 触发也安全 */
static PID_t s_BallPID = {
    .Kp = BALLSPEED_KP, .Ki = BALLSPEED_KI, .Kd = BALLSPEED_KD,
    .IntegralLimit = BALLSPEED_I_LIMIT, .OutputLimit = BALLSPEED_O_LIMIT,
    .Integral = 0, .PrevError = 0, .Out = 0
};

float g_BallTargetRPS = 0.0f;

/* ====== 采样状态 ====== */
static int16_t s_PrevBallCount;              /* 上次TIM8计数 */
static float   s_BallBuf[BALL_FILTER_SIZE];  /* 滑动平均缓存 */
static float   s_BallSum;
static uint8_t s_BallIdx;
static float   s_PrevRawBall;                /* 率限幅用 */

/* ====== 初始化：TIM8 编码器模式 ====== */

void BallSpeed_Init(void)
{
    /* 1. GPIO: PC6/PC7 → 浮空输入（TIM8复用） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef gi = {
        .GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7,
        .GPIO_Mode = GPIO_Mode_IN_FLOATING,
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    GPIO_Init(GPIOC, &gi);

    /* 2. TIM8 编码器模式（高级定时器仅用于计数，无需BDTR/MOE） */
    TIM_TimeBaseInitTypeDef ti = {
        .TIM_Prescaler = 0,
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 65535,
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = 0
    };
    TIM_TimeBaseInit(TIM8, &ti);

    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM8, ENABLE);

    /* 3. 变量清零 + 停球 */
    BallSpeed_Stop();
}

/* ====== 10ms 采样 + 速度闭环（TIM6 ISR内调用） ====== */

void BallSpeed_Update(void)
{
    /* 读TIM8计数器（16位原子读，int16_t自动补偿回绕） */
    int16_t cur = (int16_t)TIM_GetCounter(TIM8);
    int16_t d = cur - s_PrevBallCount;
    s_PrevBallCount = cur;

    /* 转速 r/s = 脉冲差 × 采样频率 / 每圈脉冲数 */
    float raw = (float)d * BALL_SAMPLE_HZ / BALL_ENC_COUNTS_PER_REV;

    /* ① 率限幅：单次跳变不超过RATE_LIMIT */
    if (raw > s_PrevRawBall + BALL_RATE_LIMIT) raw = s_PrevRawBall + BALL_RATE_LIMIT;
    if (raw < s_PrevRawBall - BALL_RATE_LIMIT) raw = s_PrevRawBall - BALL_RATE_LIMIT;
    s_PrevRawBall = raw;

    /* ② 绝对值饱和限幅 */
    if (raw >  BALL_MAX_RPS) raw =  BALL_MAX_RPS;
    if (raw < -BALL_MAX_RPS) raw = -BALL_MAX_RPS;

    /* ③ 滑动平均滤波 */
    s_BallSum -= s_BallBuf[s_BallIdx];
    s_BallBuf[s_BallIdx] = raw;
    s_BallSum += raw;
    s_BallIdx = (s_BallIdx + 1) % BALL_FILTER_SIZE;

    /* ④ PID（双向输出，球电机需正反转，不钳位≥0） */
    float fb = s_BallSum / BALL_FILTER_SIZE;
    float out = PID_Calc(&s_BallPID, g_BallTargetRPS, fb);
    BallMoter_SetSpeed((int8_t)out);
}

float BallSpeed_GetRPS(void)
{
    return s_BallSum / BALL_FILTER_SIZE;
}

void BallSpeed_Stop(void)
{
    PID_Reset(&s_BallPID);
    g_BallTargetRPS = 0.0f;
    BallMoter_SetSpeed(0);

    s_PrevBallCount = 0;
    s_PrevRawBall = 0;
    s_BallSum = 0;
    s_BallIdx = 0;
    for (uint8_t i = 0; i < BALL_FILTER_SIZE; i++)
        s_BallBuf[i] = 0;
}
