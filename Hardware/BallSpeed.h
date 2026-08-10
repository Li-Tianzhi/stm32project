#ifndef __BALLSPEED_H
#define __BALLSPEED_H

#include "stm32f10x.h"

/* ====== 球电机编码器参数 ====== */
#define BALL_ENC_PPR            11      /* 编码器线数（11线霍尔） */
#define BALL_ENC_GEAR           30      /* 减速比 1:30 */
#define BALL_ENC_COUNTS_PER_REV (BALL_ENC_PPR * BALL_ENC_GEAR * 4) /* 11×30×4=1320 (TIM8 TI12四倍频) */

/* ====== 采样 ====== */
#define BALL_SAMPLE_HZ          100     /* 100Hz = 10ms（与TIM6采样同步） */

/* ====== 滤波 ====== */
#define BALL_FILTER_SIZE        5

/* ====== 限幅保护 ====== */
#define BALL_RATE_LIMIT         2.0f    /* 单次采样转速变化率限幅 (r/s) */
#define BALL_MAX_RPS            10.0f   /* 转速绝对值饱和限幅 (r/s) */

/* ====== 球电机速度PID参数 ====== */
#define BALLSPEED_KP            80.0f
#define BALLSPEED_KI            0.0f
#define BALLSPEED_KD            0.0f
#define BALLSPEED_I_LIMIT       100.0f
#define BALLSPEED_O_LIMIT       100.0f

/* 目标转速（主循环写，ISR读） */
extern float g_BallTargetRPS;

void BallSpeed_Init(void);
void BallSpeed_Update(void);    /* 10ms采样+闭环：TIM6 ISR内调用 */
float BallSpeed_GetRPS(void);   /* 实际转速（滤波后, r/s） */
void BallSpeed_Stop(void);

#endif
