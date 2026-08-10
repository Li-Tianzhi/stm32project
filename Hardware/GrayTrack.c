/*
 * GrayTrack.c — 8路数字灰度传感器循迹模块实现（Hardware版）
 *
 * 引脚：PB0~PB7 → S0~S7
 * 权重：S0(-7) S1(-5) S2(-3) S3(-1) S4(+1) S5(+3) S6(+5) S7(+7)
 *
 * 赛道：直线段(1.5m) + 半圆弧(半径0.5m)，无直角弯
 *   全白时直接停车，不执行 pivot 转弯恢复
 *
 * 定时器：使用TIM4提供1ms节拍
 */

#include "GrayTrack.h"
#include "PID.h"        /* 外环循迹PID + 内环目标转速 */
#include <stddef.h>

/* ====== 全局变量 ====== */
float g_BaseSpeedRPS = 1.0f;    // 基础行驶速度(rps)，按键可调
int16_t g_TrackError;           // 当前循迹误差
float g_TrackCorrection;        // 外环PID修正量

/* ========== 静态变量：基础循迹 ========== */
static uint8_t s_LastRawValue = 0;
static uint8_t s_StableCount = 0;
static const int8_t s_Weight[8] = {-7, -4, -2, -1, 1, 2, 4, 7};

/* ========== 静态变量：TIM4节拍（1ms递增） ========== */
static volatile uint32_t s_TickCount = 0;

/* ========== 静态变量：误差滑动平均滤波 ========== */
#define ERROR_MA_SIZE    5
static float s_ErrBuf[ERROR_MA_SIZE];
static float s_ErrSum;
static uint8_t s_ErrIdx;
static uint8_t s_ErrCount;

/* ========== 静态变量：全白丢失 + 停车线检测 ========== */
static uint8_t  s_LastStableValue = 0;  /* 最近一次稳态灰度值（用于A点停车线检测）*/

/* ========== 1ms节拍实现 ========== */

uint32_t GrayTrack_GetTick(void)
{
    uint32_t tick;
    __disable_irq();
    tick = s_TickCount;
    __enable_irq();
    return tick;
}

void GrayTrack_TickIncrement(void)
{
    s_TickCount++;
}

/* ========== 基础函数实现 ========== */

void GrayTrack_Init(void)
{
    /* 关闭JTAG，释放PB3/PB4为普通GPIO */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 开启GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 |
                                  GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 初始化状态变量 */
    s_LastRawValue = 0;
    s_StableCount = 0;
    s_LastStableValue = 0;

    /* 初始化误差滑动平均滤波 */
    s_ErrSum = 0;
    s_ErrIdx = 0;
    s_ErrCount = 0;
    for (uint8_t i = 0; i < ERROR_MA_SIZE; i++)
        s_ErrBuf[i] = 0;

    /* 初始化TIM4为1ms定时器 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;         /* 72MHz / 7200 = 10KHz */
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;               /* 10KHz / 10 = 1KHz = 1ms */
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM4, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint8_t GrayTrack_ReadRaw(void)
{
    uint8_t rawValue = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0 << i) == Bit_RESET)
        {
            rawValue |= (0x01 << i);
        }
    }

    return rawValue;
}

void GrayTrack_ReadFiltered(GrayTrack_DataType *pData)
{
    uint8_t currentRaw;

    if (pData == NULL) return;

    currentRaw = GrayTrack_ReadRaw();

    if (currentRaw == s_LastRawValue)
    {
        s_StableCount++;
        if (s_StableCount >= DEBOUNCE_COUNT)
        {
            pData->Value = currentRaw;
            pData->Error = GrayTrack_CalcError(currentRaw);
            pData->ValidFlag = 1;
            if (s_StableCount > 250) s_StableCount = DEBOUNCE_COUNT;
        }
        else
        {
            pData->ValidFlag = 0;
        }
    }
    else
    {
        s_StableCount = 0;
        s_LastRawValue = currentRaw;
        pData->ValidFlag = 0;
    }
}

int16_t GrayTrack_CalcError(uint8_t RawValue)
{
    int16_t weightedSum = 0;
    uint8_t validCount = 0;
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (RawValue & (0x01 << i))
        {
            weightedSum += s_Weight[i];
            validCount++;
        }
    }

    if (validCount == 0) return 0;

    return (weightedSum / validCount);
}

void GrayTrack_TrackRun(int16_t Error)
{
    float correction;

    /* 滑动平均滤波（浮点，消除整数截断，过渡更圆滑）*/
    s_ErrSum -= s_ErrBuf[s_ErrIdx];
    s_ErrBuf[s_ErrIdx] = (float)Error;
    s_ErrSum += (float)Error;
    if (s_ErrCount < ERROR_MA_SIZE) s_ErrCount++;
    s_ErrIdx = (s_ErrIdx + 1) % ERROR_MA_SIZE;
    float filteredError = s_ErrSum / (float)s_ErrCount;

    g_TrackError = (int16_t)filteredError;

    /* 死区 + 过渡带：|e|≤1静默，1<|e|≤2线性过渡，|e|>2全量PID */
    correction = PID_Calc(&g_PID_Track, filteredError, 0.0f);
    {
        float absErr = filteredError >= 0 ? filteredError : -filteredError;
        if (absErr <= 1.0f)
            correction = 0;
        else if (absErr <= 2.0f)
            correction *= (absErr - 1.0f);  /* 线性过渡: 1→0, 2→全量 */
    }

    g_TrackCorrection = correction;

    /* 差速分配：误差正=偏右→左快右慢→回中 */
    g_PID_TargetL = g_BaseSpeedRPS + correction;
    g_PID_TargetR = g_BaseSpeedRPS - correction;

    /* 钳位 ≥ 0，禁反转 */
    if (g_PID_TargetL < 0) g_PID_TargetL = 0;
    if (g_PID_TargetR < 0) g_PID_TargetR = 0;
}

/* ========== 核心循迹步进 ========== */

/**
 * @brief  单步循迹（读取→滤波→检测→执行）
 *
 * 逻辑简化说明：
 *   本赛道由直线段(1.5m) + 半圆弧(半径0.5m)组成，无直角弯。
 *   当全白时直接停车，不做 pivot 转弯恢复。
 *   半圆弧半径足够大(0.5m)，配合PID差速修正即可正常通过。
 *
 * 流程：
 *   1. 读取灰度数据（带防抖）
 *   2. 防抖未通过 → 保持当前状态，本次跳过
 *   3. 检测到黑线 → 执行 PID TrackRun
 *   4. 全白 → 停车
 */
void GrayTrack_Step(void)
{
    GrayTrack_DataType grayData;

    /* 第1步：读取灰度数据（带防抖） */
    GrayTrack_ReadFiltered(&grayData);

    if (grayData.ValidFlag == 0)
    {
        return;  // 数据仍在抖动，保持当前电机状态
    }

    /* 第2步：检测到黑线 → 正常循迹 */
    if (grayData.Value != 0x00)
    {
        s_LastStableValue = grayData.Value;     /* 保存稳态值供A点停车线检测 */

        GrayTrack_TrackRun(grayData.Error);
        return;
    }

    /* 第3步：全白 → 减速，但保留基础速度。线回来时自动恢复 */
    s_LastStableValue = 0;
    /* 仅目标归零让车减速，不碰 g_BaseSpeedRPS；
       下一帧线恢复后 TrackRun 用原速度重新计算目标 */
    g_PID_TargetL = 0;
    g_PID_TargetR = 0;
    PID_Reset(&g_PID_Left);
    PID_Reset(&g_PID_Right);
}

/* ========== A点停车线检测 ========== */

/**
 * @brief  获取最近一次稳态读取中检测到黑线的传感器数量
 * @retval uint8_t 0~8
 * @note   正常循迹时通常2~3个传感器检测到黑线，
 *         经过A点启停线时增加到4~6个。
 */
uint8_t GrayTrack_GetActiveCount(void)
{
    uint8_t count = 0;
    uint8_t val = s_LastStableValue;

    while (val)
    {
        count++;
        val &= (val - 1);
    }

    return count;
}
