/*
 * Motor.c — 直流电机驱动模块
 *
 * 硬件映射：
 *   左电机：AIN1=PB8, AIN2=PB9, PWMA=PA2(TIM5_CH3)   → PWM_SetCompare3
 *   右电机：DIN1=PA11, DIN2=PA12, PWMD=PA3(TIM5_CH4)  → PWM_SetCompare4
 *
 * PWM定时器：TIM5
 */

#include "stm32f10x.h"                  // Device header
#include "PWM.h"
#include "Motor.h"

/* 死区补偿：非零但低于死区阈值的输出，提升至最小有效值 */
static int8_t Motor_DeadZoneComp(int8_t speed)
{
    if (speed > 0 && speed < MOTOR_DEAD_ZONE)
        return MOTOR_DEAD_ZONE;
    if (speed < 0 && speed > -(int8_t)MOTOR_DEAD_ZONE)
        return -(int8_t)MOTOR_DEAD_ZONE;
    return speed;
}

void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* 左电机方向：PB8(AIN1), PB9(AIN2) */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 右电机方向：PA11(DIN1), PA12(DIN2) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	PWM_Init();   // 初始化TIM5 PWM
}

/**
 * 左电机：AIN1=PB8, AIN2=PB9, PWMA=PA2(TIM5_CH3)
 * PWM_SetCompare3 对应 TIM5_CH3
 */
void MotorLeft_SetSpeed(int8_t Speed)
{
	Speed = Motor_DeadZoneComp(Speed);
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_8);     // AIN1=1
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);   // AIN2=0  → 正转
		PWM_SetCompare3(Speed);              // TIM5_CH3 PWMA (PA2)
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);   // AIN1=0
		GPIO_SetBits(GPIOB, GPIO_Pin_9);     // AIN2=1  → 反转
		PWM_SetCompare3(-Speed);             // TIM5_CH3 PWMA (PA2)
	}
}

/**
 * 右电机：DIN1=PA11, DIN2=PA12, PWMD=PA3(TIM5_CH4)
 * PWM_SetCompare4 对应 TIM5_CH4
 */
void MotorRight_SetSpeed(int8_t Speed)
{
	Speed = Motor_DeadZoneComp(Speed);
	if (Speed >= 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_11);    // DIN1=1
		GPIO_ResetBits(GPIOA, GPIO_Pin_12);  // DIN2=0 → 正转
		PWM_SetCompare4(Speed);              // TIM5_CH4 PWMD (PA3)
	}
	else
	{
		GPIO_ResetBits(GPIOA, GPIO_Pin_11);  // DIN1=0
		GPIO_SetBits(GPIOA, GPIO_Pin_12);    // DIN2=1 → 反转
		PWM_SetCompare4(-Speed);             // TIM5_CH4 PWMD (PA3)
	}
}
