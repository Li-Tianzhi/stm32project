#ifndef __BALLPWM_H
#define __BALLPWM_H

#include "stm32f10x.h"

/* ====== PWM 参数 ====== */
#define BALLPWM_PSC         36          /* 预分频: 72MHz/36 = 2MHz */
#define BALLPWM_ARR         100         /* 自动重装: 2MHz/100 = 20KHz */
#define BALLPWM_MAX         100         /* 最大占空比(ARR=100, 0~100) */

void BallPWM_Init(void);
void BallPWM_SetCompare(uint16_t Compare);

#endif
