/*
 * PID.c — 位置式PID控制器（左/右轮独立）
 *
 * 公式：Out = Kp·e + Ki·∑e + Kd·(e - e₋₁)
 *       积分限幅 + 输出限幅，防饱和
 *
 * 调用频率：与编码器采样同步（10ms），在主循环中调用
 */

#include "PID.h"

/* ====== 全局PID实例 + 目标转速 ====== */
PID_t g_PID_Left;
PID_t g_PID_Right;
PID_t g_PID_Track;          // 外环循迹PID
float g_PID_TargetL;    // 目标转速（主循环/遥控写入）
float g_PID_TargetR;

/* ====== 初始化 ====== */

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float ilimit, float olimit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->IntegralLimit = ilimit;
    pid->OutputLimit = olimit;

    PID_Reset(pid);
}

/* ====== 核心计算 ====== */

float PID_Calc(PID_t *pid, float target, float actual)
{
    float error = target - actual;

    /* 积分累计 + 限幅防饱和 */
    pid->Integral += error;
    if (pid->Integral > pid->IntegralLimit)
        pid->Integral = pid->IntegralLimit;
    if (pid->Integral < -pid->IntegralLimit)
        pid->Integral = -pid->IntegralLimit;

    /* 位置式PID */
    float out = pid->Kp * error
              + pid->Ki * pid->Integral
              + pid->Kd * (error - pid->PrevError);

    pid->PrevError = error;

    /* 输出限幅 */
    if (out > pid->OutputLimit) out = pid->OutputLimit;
    if (out < -pid->OutputLimit) out = -pid->OutputLimit;

    pid->Out = out;
    return out;
}

/* ====== 辅助函数 ====== */

void PID_Reset(PID_t *pid)
{
    pid->Integral = 0;
    pid->PrevError = 0;
    pid->Out = 0;
}

void PID_ClearI(PID_t *pid)
{
    pid->Integral = 0;
}
