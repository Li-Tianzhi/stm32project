/*
 * GrayTrack.h — 8路数字灰度传感器循迹模块（Hardware版）
 *
 * 硬件：STM32F103RCT6
 * 引脚：PB0~PB7 → S0~S7
 * 灰度逻辑：黑→低电平(0)，白→高电平(1)
 *
 * 电机复用：MotorLeft_SetSpeed() / MotorRight_SetSpeed()
 * PWM：TIM5_CH3(PA2) + TIM5_CH4(PA3)，方向IO：PA4/PA5/PA11/PA12
 *
 * 注意：本题目赛道由直线段+半圆弧组成，不含直角弯，
 *       全白时不再执行 pivot 转弯恢复，直接停车等待。
 */

#ifndef __GRAYTRACK_H
#define __GRAYTRACK_H

#include "stm32f10x.h"
#include "Motor.h"

/* ========== 基础参数 ========== */

#define DEBOUNCE_COUNT  2       // 防抖采样次数

/* ========== 数据类型 ========== */

/** 8路灰度传感器状态 */
typedef struct {
    uint8_t Value;              // 位映射，bit0=S0 ... bit7=S7
    int16_t Error;              // 偏移误差
    uint8_t ValidFlag;          // 1=数据稳定有效
} GrayTrack_DataType;

/* ========== 全局变量 ========== */
extern float g_BaseSpeedRPS;            // 基础行驶速度(rps)，按键可调
extern int16_t g_TrackError;            // 当前循迹误差
extern float g_TrackCorrection;         // 外环PID修正量

/* ========== 函数声明 ========== */

void GrayTrack_Init(void);
uint8_t GrayTrack_ReadRaw(void);
void GrayTrack_ReadFiltered(GrayTrack_DataType *pData);
int16_t GrayTrack_CalcError(uint8_t RawValue);
void GrayTrack_TrackRun(int16_t Error);
void GrayTrack_Step(void);

/** 获取系统运行毫秒数（由TIM4中断递增） */
uint32_t GrayTrack_GetTick(void);

/** TIM4中断服务调用（供stm32f10x_it.c使用）*/
void GrayTrack_TickIncrement(void);

/** 获取最近一次稳态读取中检测到黑线的传感器数量（A点停车线检测用）*/
uint8_t GrayTrack_GetActiveCount(void);

#endif /* __GRAYTRACK_H */
