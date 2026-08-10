/*
 * BallMoter.c — 小球直流减速电机驱动 (JGB37-520)
 *
 * 硬件映射：
 *   IN1=PB14, IN2=PB15, PWM=PA8(TIM1_CH1) → BallPWM_SetCompare
 *
 * PWM 定时器：TIM1
 */

#include "stm32f10x.h"
#include "BallPWM.h"
#include "BallMoter.h"

/* 死区补偿：非零但低于死区阈值的输出，提升至最小有效值 */
static int8_t BallMoter_DeadZoneComp(int8_t speed)
{
    if (speed > 0 && speed < BALLMOTER_DEAD_ZONE)
        return BALLMOTER_DEAD_ZONE;
    if (speed < 0 && speed > -(int8_t)BALLMOTER_DEAD_ZONE)
        return -(int8_t)BALLMOTER_DEAD_ZONE;
    return speed;
}

void BallMoter_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* IN1=PB14, IN2=PB15 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 初始停止 */
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);

    BallPWM_Init();
}

/**
 * 球电机调速
 *   Speed > 0: 正转, IN1=1 IN2=0
 *   Speed < 0: 反转, IN1=0 IN2=1
 *   Speed = 0: 惰行, IN1=0 IN2=0
 */
void BallMoter_SetSpeed(int8_t Speed)
{
    Speed = BallMoter_DeadZoneComp(Speed);
    if (Speed > 0)
    {
        GPIO_SetBits(GPIOB, GPIO_Pin_14);       /* IN1=1 */
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);     /* IN2=0 → 正转 */
        BallPWM_SetCompare((uint16_t)Speed);
    }
    else if (Speed < 0)
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);     /* IN1=0 */
        GPIO_SetBits(GPIOB, GPIO_Pin_15);       /* IN2=1 → 反转 */
        BallPWM_SetCompare((uint16_t)(-Speed));
    }
    else
    {
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);     /* IN1=0 */
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);     /* IN2=0 → 惰行 */
        BallPWM_SetCompare(0);
    }
}

void BallMoter_Stop(void)
{
    GPIO_ResetBits(GPIOB, GPIO_Pin_14);
    GPIO_ResetBits(GPIOB, GPIO_Pin_15);
    BallPWM_SetCompare(0);
}
