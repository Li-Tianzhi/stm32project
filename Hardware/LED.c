#include "LED.h"

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(LED_RCC, ENABLE);

    GPIO_InitStruct.GPIO_Pin   = LED_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStruct);

    LED_Off();  // Ä¬ÈÏÏ¨Ãð
}

void LED_On(void)
{
    GPIO_SetBits(LED_PORT, LED_PIN);
}

void LED_Off(void)
{
    GPIO_ResetBits(LED_PORT, LED_PIN);
}

void LED_Toggle(void)
{
    if(GPIO_ReadOutputDataBit(LED_PORT, LED_PIN))
        LED_Off();
    else
        LED_On();
}