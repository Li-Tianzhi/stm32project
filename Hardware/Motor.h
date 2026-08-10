#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

/* ====== 电机死区补偿参数 ====== */
#define MOTOR_DEAD_ZONE  20     // 电机启动死区阈值(0~100)
                                 // PWM低于此值时电机不转，
                                 // 自动补偿至此值以克服静摩擦

void Motor_Init(void);
void MotorLeft_SetSpeed(int8_t Speed);
void MotorRight_SetSpeed(int8_t Speed);

#endif
