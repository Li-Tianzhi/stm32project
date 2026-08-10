/*
 * Key.c — 非阻塞按键驱动（PA4=KEY1, PA5=KEY2）
 *
 * 使用 uwTick 做防抖，边沿检测，不阻塞主循环。
 */

#include "stm32f10x.h"

extern volatile uint32_t uwTick;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;   // PA4=KEY1, PA5=KEY2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*
 * Key_GetNum: 非阻塞，每按一次返回1或2，长按不重复
 */
uint8_t Key_GetNum(void)
{
	static uint32_t lastScan = 0;
	static uint8_t  prevK1 = 1, prevK2 = 1;   // 1=未按下
	uint8_t key = 0;

	/* 每20ms扫描一次（防抖）*/
	if (uwTick - lastScan < 20)
		return 0;
	lastScan = uwTick;

	/* 读当前状态（低电平=按下）*/
	uint8_t k1 = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET);
	uint8_t k2 = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == Bit_RESET);

	/* 下降沿检测：1→0 转换时触发一次 */
	if (k1 && !prevK1) key = 1;
	if (k2 && !prevK2 && !key) key = 2;
	prevK1 = k1;
	prevK2 = k2;

	return key;
}
