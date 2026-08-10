#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

// LED引脚定义，默认PA0
#define LED_PIN     GPIO_Pin_0
#define LED_PORT    GPIOA
#define LED_RCC     RCC_APB2Periph_GPIOA

void LED_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);

#endif