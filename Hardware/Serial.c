#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

char Serial_RxPacket[100];
volatile uint8_t Serial_RxFlag;
float   g_SerialBallX      = 0.0f;
uint8_t g_SerialBallValid  = 0;
uint16_t g_SerialRawCount   = 0;      /* 接收原始字节计数 */
uint8_t  g_SerialLastByte   = 0;      /* 最后一个接收字节（调测）*/
uint8_t  g_SerialRxState    = 0;      /* 状态机当前状态（调测）*/

/* ASCII 命令缓冲区 */
char    g_SerialCmdBuf[32];
volatile uint8_t g_SerialCmdReady = 0;

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);
	
	/* RXNE 中断不使能，改用轮询（避免干扰 OLED 软件 I2C 时序）*/
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

/**
 * @brief Serial_Poll — 轮询 USART1 接收 OpenMV 二进制帧
 *
 * 在主循环中调用（替代中断方式），一次性读空 RX FIFO，
 * 避免中断打断 OLED 软件 I2C 时序导致花屏/无显示。
 */
void Serial_Poll(void)
{
	#define P_STATE_HDR1  0
	#define P_STATE_HDR2  1
	#define P_STATE_DATA  2
	#define P_READ_MAX    20   /* 每次最多读20字节，防止断线时死循环 */

	static uint8_t s_State   = P_STATE_HDR1;
	static uint8_t s_Buf[8];
	static uint8_t s_Idx     = 0;
	static uint8_t s_Cksum   = 0;
	static uint8_t s_CmdLen  = 0;

	uint8_t count = 0;

	/* 一次性读完所有已接收字节（有限次） */
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET && count < P_READ_MAX)
	{
		count++;
		uint8_t ch = USART_ReceiveData(USART1);
		g_SerialRawCount++;
		g_SerialLastByte = ch;
		g_SerialRxState  = s_State;

		switch (s_State)
		{
		case P_STATE_HDR1:
			if (ch == 0xAA) { s_State = P_STATE_HDR2; }
			else if (ch >= 0x20 && ch <= 0x7E && s_CmdLen < 30) {
				g_SerialCmdBuf[s_CmdLen++] = (char)ch;
			} else if (ch == 0x0D || ch == 0x0A) {
				if (s_CmdLen > 0) {
					g_SerialCmdBuf[s_CmdLen] = 0;
					g_SerialCmdReady = 1;
					s_CmdLen = 0;
				}
			}
			break;

		case P_STATE_HDR2:
			if (ch == 0x55)
			{
				s_State = P_STATE_DATA;
				s_Idx   = 0;
				s_Cksum = 0xAA + 0x55;
			}
			else
			{
				s_State = P_STATE_HDR1;
			}
			break;

		case P_STATE_DATA:
			s_Buf[s_Idx++] = ch;
			s_Cksum += ch;
			if (s_Idx >= 8)
			{
				if ((uint8_t)(s_Cksum - s_Buf[7]) == s_Buf[7])
				{
					if (s_Buf[6])  /* valid == 1 */
					{
						int16_t x_raw = (int16_t)((uint16_t)s_Buf[2] << 8 | s_Buf[1]);
						g_SerialBallX     = (float)x_raw / 100.0f;
						g_SerialBallValid = 1;
					}
				}
				s_State = P_STATE_HDR1;
			}
			break;
		}
	}
}

/**
 * @brief USART1 中断 — 解析 OpenMV 二进制帧（10字节）
 *
 * 帧格式: 0xAA 0x55 seq x_low x_high area_low area_high quality valid cksum
 *   X 坐标: x_int = (int16_t)(x_high<<8 | x_low)  单位 0.1mm
 *   → x_cm = x_int / 100.0f
 */
void USART1_IRQHandler(void)
{
	#define BIN_STATE_HDR1  0   /* 等待 0xAA */
	#define BIN_STATE_HDR2  1   /* 等待 0x55 */
	#define BIN_STATE_DATA  2   /* 接收 8 字节数据 */

	static uint8_t s_State   = BIN_STATE_HDR1;
	static uint8_t s_Buf[8];
	static uint8_t s_Idx     = 0;
	static uint8_t s_Cksum   = 0;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t ch = USART_ReceiveData(USART1);

		switch (s_State)
		{
		case BIN_STATE_HDR1:
			if (ch == 0xAA) { s_State = BIN_STATE_HDR2; }
			break;

		case BIN_STATE_HDR2:
			if (ch == 0x55)
			{
				s_State = BIN_STATE_DATA;
				s_Idx   = 0;
				s_Cksum = 0xAA + 0x55;
			}
			else
			{
				s_State = BIN_STATE_HDR1;
			}
			break;

		case BIN_STATE_DATA:
			s_Buf[s_Idx++] = ch;
			s_Cksum += ch;
			if (s_Idx >= 8)
			{
				/* s_Buf: [0]seq [1]x_low [2]x_high [3]area_low
				 *        [4]area_high [5]quality [6]valid [7]rcv_cksum */
				/* 校验: 前9字节(0xAA+0x55+s_Buf[0..6])累加 == s_Buf[7] */
				if ((uint8_t)(s_Cksum - s_Buf[7]) == s_Buf[7])
				{
					if (s_Buf[6])  /* valid == 1 */
					{
						int16_t x_raw = (int16_t)((uint16_t)s_Buf[2] << 8 | s_Buf[1]);
						g_SerialBallX     = (float)x_raw / 100.0f;  /* 0.1mm→cm */
						g_SerialBallValid = 1;
					}
				}
				s_State = BIN_STATE_HDR1;  /* 复位等下一帧 */
			}
			break;
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
