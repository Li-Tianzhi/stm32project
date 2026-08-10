#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* ====== 电机参数 ====== */
#define ENC_PPR             13      // 编码器线数（13线，26极正交AB相）
#define ENC_GEAR            90      // 减速比
#define ENC_COUNTS_PER_REV  (ENC_PPR * ENC_GEAR * 4)  // 13×90×4 = 4680 (TIM TI12四倍频)

/* ====== 采样 ====== */
#define ENC_SAMPLE_HZ       100     // 100Hz = 10ms

/* ====== 滤波 ====== */
#define ENC_FILTER_SIZE     5

/* ====== 限幅保护 ====== */
#define ENC_RATE_LIMIT      2.0f    // 单次采样转速变化率限幅 (r/s)
#define ENC_MAX_RPS         10.0f   // 转速绝对值饱和限幅 (r/s)

/* ====== 方向反转（前进为负时设1）====== */
#define ENC_LEFT_REVERSE    0
#define ENC_RIGHT_REVERSE   0

void Encoder_Init(void);
float Encoder_GetLeftSpeed_RPS(void);
float Encoder_GetRightSpeed_RPS(void);
void Encoder_ResetAllCount(void);
int32_t Encoder_GetLeftPulses(void);
int32_t Encoder_GetRightPulses(void);
void Encoder_ResetPulses(void);

#endif
