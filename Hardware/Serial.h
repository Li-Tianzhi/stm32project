#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>

extern char Serial_RxPacket[];
extern volatile uint8_t Serial_RxFlag;

/* 二进制帧接收：OpenMV 小球坐标 */
extern float   g_SerialBallX;       /* 球 X 坐标(cm) */
extern uint8_t  g_SerialBallValid;  /* 收到有效帧标志 */
extern uint16_t g_SerialRawCount;   /* 原始字节计数（调试用）*/
extern uint8_t  g_SerialLastByte;   /* 最后一个接收字节 */
extern uint8_t  g_SerialRxState;    /* 状态机当前状态 */

/* ASCII 命令缓冲区 */
extern char    g_SerialCmdBuf[32];
extern volatile uint8_t g_SerialCmdReady;

void Serial_Init(void);
void Serial_Poll(void);
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

#endif
