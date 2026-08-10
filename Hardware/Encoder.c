/*
 * Encoder.c — 编码器测速（TIM硬件正交解码 + TIM6采样 + 滑动平均滤波）
 *
 * 硬件：WHEELTEC 1:90F TT减速电机，12PPR霍尔编码器
 *       左轮：PA0(TIM2_CH1) + PA1(TIM2_CH2)
 *       右轮：PA6(TIM3_CH1) + PA7(TIM3_CH2)
 *
 * 修改历史：
 *   V2.0 改用TIM硬件编码器模式替代EXTI，消除高速丢脉冲隐患
 *        TIM2/TIM3编码器模式TI12四倍频 → 4320计数/车轮圈
 *        TIM6基本定时器10ms采样（释放TIM2给编码器）
 *        新增率限幅+绝对值饱和，抑制转速跳变
 *        单次NVIC分组保护
 *
 * 隐患修复对照：
 *   隐患1 → TIM硬件正交解码，零软件开销，不丢脉冲
 *   隐患2 → s_NvicGrouped标志位，仅首次调用设置分组
 *   隐患3 → TIM_GetCounter 16位原子读，ISR外无竞争
 *   隐患4 → 率限幅(RATE_LIMIT) + 绝对值饱和(MAX_RPS)双层保护
 *   隐患5 → 宏定义PSC/ARR，如需改时钟只需调TIM6_PSC
 *
 *   V2.1 PID集成：TIM6 ISR内调用PID_Calc + Motor_SetSpeed，10ms精确控速
 *
 * 注意：若SysClk非72MHz，调整TIM6_PSC使计时器时钟=10KHz
 */

#include "Encoder.h"
#include "PID.h"       /* PID速度闭环 */
#include "Motor.h"     /* 电机PWM输出 */
#include "BallSpeed.h" /* 球电机速度闭环（TIM6 ISR内调用） */

/* ====== 采样定时器分频， PID 调控周期严格固定 10ms（100Hz）====== */
/* APB1定时器时钟 = 72MHz (APB1=36MHz, 但APB1预分频≠1时定时器时钟×2) */
#define TIM6_PSC        (7200 - 1)      /* 72MHz/7200 = 10KHz */
#define TIM6_ARR        (100 - 1)       /* 10KHz/100 = 100Hz = 10ms */

/* ====== 静态变量 ====== */
/* 10ms脉冲增量（ISR写入，主循环读取）*/
static volatile int16_t s_LeftDelta;
static volatile int16_t s_RightDelta;

/* 累计脉冲（循迹距离计算用）*/
static volatile int32_t s_LeftTotalPulses = 0;
static volatile int32_t s_RightTotalPulses = 0;

/* 上次采样计数值（16位回绕由int16_t自动处理）*/
static int16_t s_PrevLeft;
static int16_t s_PrevRight;

/* 滑动平均滤波 */
static float s_LeftBuf[ENC_FILTER_SIZE];
static float s_RightBuf[ENC_FILTER_SIZE];
static float s_LeftSum;
static float s_RightSum;
static uint8_t s_BufIdx;

/* 前次原始转速（率限幅用）*/
static float s_PrevRawL;
static float s_PrevRawR;

/* NVIC分组保护（全工程仅设置一次）*/
static uint8_t s_NvicGrouped;

/* ====== TIM6 10ms采样中断 ====== */

void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update))
    {
        /* 读TIM2/TIM3计数器值（16位原子读）*/
        int16_t curL = (int16_t)TIM_GetCounter(TIM2);
        int16_t curR = (int16_t)TIM_GetCounter(TIM3);

        /* 计算增量（int16_t自动补偿65536回绕）*/
        int16_t dL = curL - s_PrevLeft;
        int16_t dR = curR - s_PrevRight;
        s_PrevLeft = curL;
        s_PrevRight = curR;

        /* 方向反转修正 */
#if ENC_LEFT_REVERSE
        dL = -dL;
#endif
#if ENC_RIGHT_REVERSE
        dR = -dR;
#endif

        /* 写脉冲增量（供外部调试读取）*/
        s_LeftDelta = dL;
        s_RightDelta = dR;

        /* 累计脉冲（行驶距离计算）*/
        s_LeftTotalPulses += dL;
        s_RightTotalPulses += dR;

        /* 转速 r/s = 脉冲差 × 采样频率 / 每圈脉冲数 */
        float rawL = (float)dL * ENC_SAMPLE_HZ / ENC_COUNTS_PER_REV;
        float rawR = (float)dR * ENC_SAMPLE_HZ / ENC_COUNTS_PER_REV;

        /* ① 率限幅：单次跳变不超过RATE_LIMIT */
        if (rawL > s_PrevRawL + ENC_RATE_LIMIT) rawL = s_PrevRawL + ENC_RATE_LIMIT;
        if (rawL < s_PrevRawL - ENC_RATE_LIMIT) rawL = s_PrevRawL - ENC_RATE_LIMIT;
        s_PrevRawL = rawL;

        if (rawR > s_PrevRawR + ENC_RATE_LIMIT) rawR = s_PrevRawR + ENC_RATE_LIMIT;
        if (rawR < s_PrevRawR - ENC_RATE_LIMIT) rawR = s_PrevRawR - ENC_RATE_LIMIT;
        s_PrevRawR = rawR;

        /* ② 绝对值饱和限幅 */
        if (rawL > ENC_MAX_RPS) rawL = ENC_MAX_RPS;
        if (rawL < -ENC_MAX_RPS) rawL = -ENC_MAX_RPS;
        if (rawR > ENC_MAX_RPS) rawR = ENC_MAX_RPS;
        if (rawR < -ENC_MAX_RPS) rawR = -ENC_MAX_RPS;

        /* ③ 滑动平均滤波 */
        s_LeftSum -= s_LeftBuf[s_BufIdx];
        s_RightSum -= s_RightBuf[s_BufIdx];
        s_LeftBuf[s_BufIdx] = rawL;
        s_RightBuf[s_BufIdx] = rawR;
        s_LeftSum += rawL;
        s_RightSum += rawR;
        s_BufIdx = (s_BufIdx + 1) % ENC_FILTER_SIZE;

        /* ④ PID速度闭环（用滤波后转速做反馈，精确10ms周期）*/
        {
            float fbL = s_LeftSum / ENC_FILTER_SIZE;
            float fbR = s_RightSum / ENC_FILTER_SIZE;
            float outL = PID_Calc(&g_PID_Left, g_PID_TargetL, fbL);
            float outR = PID_Calc(&g_PID_Right, g_PID_TargetR, fbR);

            /* 钳位≥0禁止反转 + 清积分防windup */
            if (outL < 0) { outL = 0; PID_ClearI(&g_PID_Left); }
            if (outR < 0) { outR = 0; PID_ClearI(&g_PID_Right); }

            /* ISR内环直接输出PWM，GrayTrack不再直控电机 */
            MotorLeft_SetSpeed((int8_t)outL);
            MotorRight_SetSpeed((int8_t)outR);
        }

        /* 球电机速度闭环（同10ms周期） */
        BallSpeed_Update();

        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
    }
}

/* ====== 初始化 ====== */

void Encoder_Init(void)
{
    uint8_t i;

    /* 1. GPIO: PA0/PA1/PA6/PA7 → 浮空输入（TIM复用） */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef gi = {
        .GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6 | GPIO_Pin_7,
        .GPIO_Mode = GPIO_Mode_IN_FLOATING,
        .GPIO_Speed = GPIO_Speed_50MHz
    };
    GPIO_Init(GPIOA, &gi);

    /* 2. TIM2 编码器模式 —— 左轮 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef ti = {
        .TIM_Prescaler = 0,
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 65535,
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = 0
    };
    TIM_TimeBaseInit(TIM2, &ti);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM2, ENABLE);

    /* 3. TIM3 编码器模式 —— 右轮 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    ti.TIM_Prescaler = 0;
    ti.TIM_Period = 65535;
    TIM_TimeBaseInit(TIM3, &ti);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_Cmd(TIM3, ENABLE);

    /* 4. TIM6 基本定时器 —— 10ms采样 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    ti.TIM_Prescaler = TIM6_PSC;
    ti.TIM_Period = TIM6_ARR;
    TIM_TimeBaseInit(TIM6, &ti);

    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);

    /* 5. NVIC（仅首次设置分组）*/
    if (!s_NvicGrouped)
    {
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
        s_NvicGrouped = 1;
    }

    NVIC_InitTypeDef ni = {
        .NVIC_IRQChannel = TIM6_IRQn,
        .NVIC_IRQChannelPreemptionPriority = 1,
        .NVIC_IRQChannelSubPriority = 0,
        .NVIC_IRQChannelCmd = ENABLE
    };
    NVIC_Init(&ni);

    /* 6. 变量清零 */
    s_LeftDelta = 0;
    s_RightDelta = 0;
    s_PrevLeft = 0;
    s_PrevRight = 0;
    s_PrevRawL = 0;
    s_PrevRawR = 0;
    s_LeftSum = 0;
    s_RightSum = 0;
    s_BufIdx = 0;
    for (i = 0; i < ENC_FILTER_SIZE; i++)
    {
        s_LeftBuf[i] = 0;
        s_RightBuf[i] = 0;
    }
}

/* ====== 对外API ====== */

float Encoder_GetLeftSpeed_RPS(void)
{
    return s_LeftSum / ENC_FILTER_SIZE;
}

float Encoder_GetRightSpeed_RPS(void)
{
    return s_RightSum / ENC_FILTER_SIZE;
}

void Encoder_ResetAllCount(void)
{
    uint8_t i;
    __disable_irq();
    TIM_SetCounter(TIM2, 0);
    TIM_SetCounter(TIM3, 0);
    s_PrevLeft = 0;
    s_PrevRight = 0;
    s_LeftDelta = 0;
    s_RightDelta = 0;
    s_PrevRawL = 0;
    s_PrevRawR = 0;
    s_LeftSum = 0;
    s_RightSum = 0;
    s_BufIdx = 0;
    for (i = 0; i < ENC_FILTER_SIZE; i++)
    {
        s_LeftBuf[i] = 0;
        s_RightBuf[i] = 0;
    }
    s_LeftTotalPulses = 0;
    s_RightTotalPulses = 0;
    __enable_irq();
}

/* ====== 脉冲计数器（循迹距离）====== */

int32_t Encoder_GetLeftPulses(void)
{
    return s_LeftTotalPulses;
}

int32_t Encoder_GetRightPulses(void)
{
    return s_RightTotalPulses;
}

void Encoder_ResetPulses(void)
{
    __disable_irq();
    s_LeftTotalPulses = 0;
    s_RightTotalPulses = 0;
    __enable_irq();
}
