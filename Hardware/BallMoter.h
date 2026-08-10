#ifndef __BALLMOTER_H
#define __BALLMOTER_H

#include "stm32f10x.h"
#include "BallPWM.h"

/* ====== 死区补偿 ====== */
#define BALLMOTER_DEAD_ZONE  20       /* 死区阈值（0~BALLPWM_MAX），低于此值补偿至此 */

/* ====== Speed 范围与 BALLPWM_MAX 一致 ====== */
/* BALLMOTER_MAX_SPEED = BALLPWM_MAX = 100 */

void BallMoter_Init(void);
void BallMoter_SetSpeed(int8_t Speed);   /* Speed ∈ [-BALLPWM_MAX, +BALLPWM_MAX] */
void BallMoter_Stop(void);

#endif
