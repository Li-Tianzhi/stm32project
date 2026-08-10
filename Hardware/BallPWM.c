/*
 * BallPWM.c — 球电机 PWM 驱动
 *
 * 硬件：TIM1_CH1(PA8)
 * 频率：72MHz/36/100 = 20KHz
 * 注意：TIM1 是高级定时器，必须配置 BDTR + 使能 MOE
 */

#include "stm32f10x.h"
#include "BallPWM.h"

void BallPWM_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |
                           RCC_APB2Periph_GPIOA, ENABLE);

    /* PA8 = TIM1_CH1 复用推挽 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM1);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period            = BALLPWM_ARR - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler         = BALLPWM_PSC - 1;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;     /* CCR */
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);

    /* 高级定时器 BDTR 配置 + 使能主输出 */
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    TIM_BDTRStructInit(&TIM_BDTRInitStructure);
    TIM_BDTRInitStructure.TIM_OSSRState  = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState  = TIM_OSSIState_Enable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
}

void BallPWM_SetCompare(uint16_t Compare)
{
    TIM_SetCompare1(TIM1, Compare);
}
