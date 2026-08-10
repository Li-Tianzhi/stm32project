/*
 * main.c — 多任务循迹系统（任务2~6，KEY1单键操作）
 *
 * 按键逻辑：
 *   IDLE:        KEY1短按 → 进入任务选择
 *   TASK_SELECT: KEY1短按=轮换任务, KEY2短按=启动
 *   RUNNING:     KEY1任意按 → 中止回IDLE
 *   COMPLETE:    KEY1任意按 → 回IDLE
 *
 * 任务列表：
 *   任务2: 循迹一圈回A停车（≤20s，停车偏差≤2cm）
 *   任务3: 球电机速度闭环演示（主轮锁死，球 0→+5→-5 rps）
 *   任务4: A→B直线循迹（≤8s）
 *   任务5: 循迹一圈回A（≤30s，不做小球仅循迹）
 *   任务6: 循迹一圈回A（≤30s，不做小球仅循迹）
 *
 * 芯片: STM32F103RCT6
 */

#include "stm32f10x.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "OLED.h"
#include "Serial.h"
#include "Key.h"
#include "GrayTrack.h"
#include "BallMoter.h"
#include "BallSpeed.h"



extern volatile uint32_t uwTick;

/* ====== 任务索引 ====== */
#define TASK_2_IDX      0
#define TASK_3_IDX      1
#define TASK_4_IDX      2
#define TASK_5_IDX      3
#define TASK_6_IDX      4
#define TASK_COUNT      5

/* ====== 参数常量 ====== */
#define PARKING_THRESH      5       /* 停车线检测：≥5个传感器检测黑线 */
#define KEY_DEBOUNCE_MS     20      /* 按键消抖(ms) */
#define WHEEL_CIRC_M        0.2042f /* 车轮周长(m) = π × 0.065m */
#define DIST_AB_M           1.65f   /* A→B 距离(m) */

/* ====== 状态机 ====== */
#define STATE_IDLE          0
#define STATE_TASK_SELECT   1
#define STATE_RUNNING       2
#define STATE_COMPLETE      3

/* ====== 全局变量 ====== */
static uint8_t  g_State = STATE_IDLE;
static uint8_t  g_TaskIdx = TASK_2_IDX;
static uint32_t g_StartTick = 0;
static uint32_t g_LapTimeMs = 0;
static uint8_t  g_DisplayReady = 0;
static uint8_t  g_ParkingArmed = 0;
static float    g_TravelDistM = 0.0f;
static float    s_BallPosRaw = 0.0f;   /* OpenMV 原始球位置(cm) */

/* ====== 任务参数表 ====== */
/* 基础速度(RPS)。车轮直径6.5cm，周长0.2042m */
static const float g_TaskSpeed[TASK_COUNT] = {
    1.9f,   /* 任务2: 满圈~6.14m, 1.7RPS×0.2042≈0.347m/s → ~17.7s */
    0.0f,   /* 任务3: 静止 */
    1.3f,   /* 任务4: A→B=1.9m, 1.3RPS≈0.265m/s → ~7.2s */
    1.3f,   /* 任务5: 满圈≤30s, 1.2RPS≈0.245m/s → ~25.1s */
    1.3f    /* 任务6: 满圈≤30s, 1.2RPS≈0.245m/s → ~25.1s */
};

/* 16文字符串（OLED 16列×4行，全部精确16字符）*/
static char *g_TaskSelName[TASK_COUNT] = {
    "[2] Lap         ",
    "[3] Ball Ctrl   ",
    "[4] A->B Line   ",
    "[5] Lap+Center  ",
    "[6] Lap+Target  "
};

static char *g_TaskDesc[TASK_COUNT] = {
    "Lap<20s  1.9rps ",
    "Ball 0->+5->-5  ",
    "A->B<8s  1.3rps ",
    "Lap<30s  1.3rps ",
    "Lap<30s  1.3rps "
};

/* ================================================================
 *  KEY1 短按检测（非阻塞，20ms消抖）
 * ================================================================
 * 引脚: PA4 = GPIO_Mode_IPU, 未按=高(1), 按下=低(0)
 */
static uint8_t  s_k1_Now = 1;
static uint8_t  s_k1_Prev = 1;
static uint8_t  s_k1_ShortEvt = 0;
static uint32_t s_k1_LastScan = 0;

static void KEY1_Scan(void)
{
    uint32_t now = uwTick;
    if (now - s_k1_LastScan < KEY_DEBOUNCE_MS)
        return;
    s_k1_LastScan = now;

    s_k1_Prev = s_k1_Now;
    s_k1_Now = (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == Bit_RESET) ? 0 : 1;

    /* 上升沿（释放）→ 短按 */
    if (s_k1_Now == 1 && s_k1_Prev == 0)
        s_k1_ShortEvt = 1;
}

static void ClearKeyEvents(void)
{
    s_k1_ShortEvt = 0;
}

/* ====== 电机停止 ====== */
static void StopMotors(void)
{
    PID_Reset(&g_PID_Left);
    PID_Reset(&g_PID_Right);
    g_BaseSpeedRPS = 0;
    g_PID_TargetL  = 0;
    g_PID_TargetR  = 0;
    BallSpeed_Stop();
}

/* ====== 转速显示（带符号，0.1rps 精度） ====== */
static void ShowRpsAt(uint8_t row, uint8_t col, float rps)
{
    int a = (int)((rps >= 0.0f ? rps : -rps) * 10.0f + 0.5f);
    OLED_ShowChar(row, col,     (rps < 0.0f) ? '-' : ' ');
    OLED_ShowChar(row, col + 1, (a >= 100) ? '0' + a / 100 : ' ');
    OLED_ShowChar(row, col + 2, '0' + (a % 100) / 10);
    OLED_ShowChar(row, col + 3, '.');
    OLED_ShowChar(row, col + 4, '0' + a % 10);
    OLED_ShowChar(row, col + 5, 'r');
    OLED_ShowChar(row, col + 6, 'p');
    OLED_ShowChar(row, col + 7, 's');
}

/* ====== IDLE 画面 ====== */
static void ShowIdleScreen(void)
{
    OLED_ShowString(1, 1, "Ready! KEY1=Sel ");
    OLED_ShowString(2, 1, "[2] Lap         ");
    OLED_ShowString(3, 1, "Lap<20s  1.9rps ");
    OLED_ShowString(4, 1, "KEY2=Start      ");
}

/* ====== 时钟配置 ====== */
void SystemClock_Config(void)
{
    RCC_HSEConfig(RCC_HSE_ON);
    while (RCC_WaitForHSEStartUp() != SUCCESS);
    FLASH_SetLatency(FLASH_Latency_2);
    FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
    RCC_PLLCmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    while (RCC_GetSYSCLKSource() != 0x08);
    RCC_HCLKConfig(RCC_SYSCLK_Div1);
    RCC_PCLK1Config(RCC_HCLK_Div2);
    RCC_PCLK2Config(RCC_HCLK_Div1);
}

/* ================================================================
 *  主函数
 * ================================================================ */
int main(void)
{
    SystemClock_Config();

    if (SysTick_Config(SystemCoreClock / 1000))
        while (1);

    Motor_Init();

    PID_Init(&g_PID_Left,  PID_KP, PID_KI, PID_KD,
             PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    PID_Init(&g_PID_Right, PID_KP, PID_KI, PID_KD,
             PID_INTEGRAL_LIMIT, PID_OUTPUT_LIMIT);
    PID_Init(&g_PID_Track, TRACK_KP, TRACK_KI, TRACK_KD,
             TRACK_INTEGRAL_LIMIT, TRACK_OUTPUT_LIMIT);

    Encoder_Init();
    OLED_Init();
    Serial_Init();
    Key_Init();
    GrayTrack_Init();
		BallMoter_Init();
		BallSpeed_Init();


    /* 上电稳定延时 */
    {
        uint32_t i, j;
        for (i = 0; i < 1000; i++)
            for (j = 0; j < 7200; j++);
    }

    PID_Reset(&g_PID_Left);
    PID_Reset(&g_PID_Right);
    g_BaseSpeedRPS = g_TaskSpeed[TASK_2_IDX];

    /* ---- OLED IDLE 画面 ---- */
    ShowIdleScreen();

    while (1)
    {	
        KEY1_Scan();
        uint8_t key2 = Key_GetNum();    /* KEY2 返回2表示短按 */

        /* 提取并清空按键事件（防重复处理）*/
        uint8_t evShort = s_k1_ShortEvt;
        ClearKeyEvents();

        /* ---- 串口轮询：OpenMV 二进制帧 → 小球 X 坐标 ---- */
        Serial_Poll();
        if (g_SerialBallValid)
        {
            s_BallPosRaw = g_SerialBallX;
            g_SerialBallValid = 0;
        }

        switch (g_State)
        {
        /* ==========================================================
         *  STATE_IDLE — 待机，短按进选择
         * ========================================================== */
        case STATE_IDLE:
            if (evShort)
            {
                g_TaskIdx = TASK_2_IDX;
                OLED_Clear();
                OLED_ShowString(1, 1, g_TaskSelName[TASK_2_IDX]);
                OLED_ShowString(2, 1, "K1:Cycle        ");
                OLED_ShowString(3, 1, "K2:Start        ");
                OLED_ShowString(4, 1, g_TaskDesc[TASK_2_IDX]);
                g_State = STATE_TASK_SELECT;
            }
            /* 串口数据监测（200ms刷新第4行）*/
            {
                static uint32_t s_IdleTick = 0;
                if (uwTick - s_IdleTick >= 150)
                {
                    s_IdleTick = uwTick;
                    char line[17] = "S:X=            ";
                    float ax = (s_BallPosRaw >= 0.0f) ? s_BallPosRaw : -s_BallPosRaw;
                    int xi = (int)ax;
                    int xd = (int)(ax * 10.0f + 0.5f) % 10;
                    int p = 4;
                    line[p++] = (s_BallPosRaw >= 0.0f) ? '+' : '-';
                    if (xi >= 10) {
                        line[p++] = '0' + xi / 10;
                        line[p++] = '0' + xi % 10;
                    } else {
                        line[p++] = ' ';
                        line[p++] = '0' + xi;
                    }
                    line[p++] = '.';
                    line[p++] = '0' + xd;
                    line[p++] = 'c';
                    line[p++] = 'm';
                    OLED_ShowString(4, 1, line);
                }
            }
            break;

        /* ==========================================================
         *  STATE_TASK_SELECT — 循环选择，长按启动
         * ========================================================== */
        case STATE_TASK_SELECT:
            if (evShort)
            {
                g_TaskIdx++;
                if (g_TaskIdx >= TASK_COUNT)
                    g_TaskIdx = TASK_2_IDX;
                OLED_ShowString(1, 1, g_TaskSelName[g_TaskIdx]);
                OLED_ShowString(4, 1, g_TaskDesc[g_TaskIdx]);
            }
            if (key2 == 2)
            {
                /* 准备执行任务 */
                StopMotors();
                g_BaseSpeedRPS = g_TaskSpeed[g_TaskIdx];
                g_StartTick     = uwTick;
                g_DisplayReady  = 0;
                g_ParkingArmed  = 0;
                g_TravelDistM   = 0.0f;
                if (g_TaskIdx != TASK_3_IDX)
                    Encoder_ResetPulses();

                OLED_Clear();
                g_State = STATE_RUNNING;
            }
            break;

        /* ==========================================================
         *  STATE_RUNNING — 执行任务
         * ========================================================== */
        case STATE_RUNNING:
        {
            uint32_t elapsed = uwTick - g_StartTick;
            uint32_t sec = elapsed / 1000;
            uint32_t dec = (elapsed % 1000) / 100;

            /* ---- 任务 3：球速度闭环演示 0→+5→-5 ---- */
            if (g_TaskIdx == TASK_3_IDX)
            {
                if (!g_DisplayReady)
                {
                    g_DisplayReady = 1;
                    OLED_Clear();
                    OLED_ShowString(1, 1, "**Task3:Ball*** ");
                }

                /* 目标转速时序: 0-2s=0, 2-5s=+5, 5-8s=-5, 8s后=0 */
                if      (elapsed < 2000) g_BallTargetRPS = 0.0f;
                else if (elapsed < 5000) g_BallTargetRPS = 5.0f;
                else if (elapsed < 8000) g_BallTargetRPS = -5.0f;
                else                     g_BallTargetRPS = 0.0f;

                /* 100ms刷新显示：时间/实际转速/目标转速 */
                {
                    static uint32_t oledTick = 0;
                    if (uwTick - oledTick >= 100)
                    {
                        oledTick = uwTick;

                        /* 第2行：时间 */
                        OLED_ShowChar(2, 1, 'T');
                        OLED_ShowChar(2, 2, ':');
                        OLED_ShowChar(2, 3, ' ');
                        OLED_ShowChar(2, 4, '0' + sec / 10);
                        OLED_ShowChar(2, 5, '0' + sec % 10);
                        OLED_ShowChar(2, 6, '.');
                        OLED_ShowChar(2, 7, '0' + dec);
                        OLED_ShowChar(2, 8, 's');

                        /* 第3行：实际转速 */
                        OLED_ShowChar(3, 1, 'A');
                        OLED_ShowChar(3, 2, ':');
                        OLED_ShowChar(3, 3, ' ');
                        ShowRpsAt(3, 4, BallSpeed_GetRPS());

                        /* 第4行：目标转速 */
                        OLED_ShowChar(4, 1, 'T');
                        OLED_ShowChar(4, 2, ':');
                        OLED_ShowChar(4, 3, ' ');
                        ShowRpsAt(4, 4, g_BallTargetRPS);
                    }
                }

                /* 8s 演示结束 */
                if (elapsed >= 8000)
                {
                    StopMotors();           /* 含 BallSpeed_Stop */
                    g_LapTimeMs = elapsed;
                    g_State = STATE_COMPLETE;
                    break;
                }
            }
            /* ---- 任务 2/4/5/6：循迹 ---- */
            else
            {
                GrayTrack_Step();

                /* 时间显示（第1行右侧）*/
                if (!g_DisplayReady)
                {
                    g_DisplayReady = 1;
                    switch (g_TaskIdx)
                    {
                    case TASK_2_IDX:
                        OLED_ShowString(1, 1, "~~Task2:Lap~~~~ ");
                        break;
                    case TASK_4_IDX:
                        OLED_ShowString(1, 1, "**Task4:A->B**  ");
                        break;
                    case TASK_5_IDX:
                        OLED_ShowString(1, 1, "~~Task5:Lap~~~~ ");
                        break;
                    case TASK_6_IDX:
                        OLED_ShowString(1, 1, "~~Task6:Lap~~~~ ");
                        break;
                    }
                }

                /* Task 4 距离计算（控制用，每循环执行）*/
                if (g_TaskIdx == TASK_4_IDX)
                {
                    int32_t avgPulses = (Encoder_GetLeftPulses()
                                       + Encoder_GetRightPulses()) / 2;
                    g_TravelDistM = (float)avgPulses
                                  / (float)ENC_COUNTS_PER_REV * WHEEL_CIRC_M;
                    if (g_TravelDistM < 0.0f) g_TravelDistM = 0.0f;
                }

                /* OLED 定期刷新（100ms间隔，减少主循环开销）*/
                {
                    static uint32_t oledTick = 0;
                    if (uwTick - oledTick >= 100)
                    {
                        oledTick = uwTick;

                        /* 第2行：时间显示 */
                        OLED_ShowChar(2, 1, 'T');
                        OLED_ShowChar(2, 2, ':');
                        OLED_ShowChar(2, 3, ' ');
                        OLED_ShowChar(2, 4, '0' + sec / 10);
                        OLED_ShowChar(2, 5, '0' + sec % 10);
                        OLED_ShowChar(2, 6, '.');
                        OLED_ShowChar(2, 7, '0' + dec);
                        OLED_ShowChar(2, 8, 's');

                        if (g_TaskIdx == TASK_4_IDX)
                        {
                            /* 任务4：距离显示 */
                            uint32_t d_cm = (uint32_t)(g_TravelDistM * 100.0f + 0.5f);
                            OLED_ShowChar(3, 1, 'D');
                            OLED_ShowChar(3, 2, ':');
                            OLED_ShowChar(3, 3, ' ');
                            OLED_ShowChar(3, 4, '0' + d_cm / 100);
                            OLED_ShowChar(3, 5, '.');
                            OLED_ShowChar(3, 6, '0' + (d_cm % 100) / 10);
                            OLED_ShowChar(3, 7, '0' + d_cm % 10);
                            OLED_ShowChar(3, 8, 'm');
                            OLED_ShowString(4, 1, "KEY1=Abort      ");
                        }
                        else
                        {
                            /* 任务 2/5/6：传感器 + 误差 */
                            uint8_t cnt = GrayTrack_GetActiveCount();
                            OLED_ShowChar(3, 1, 'S');
                            OLED_ShowChar(3, 2, ':');
                            OLED_ShowChar(3, 3, '0' + cnt);
                            OLED_ShowChar(3, 4, ' ');
                            OLED_ShowString(3, 6, "E:");
                            if (g_TrackError >= 0)
                            {
                                OLED_ShowChar(3, 8, ' ');
                                OLED_ShowChar(3, 9, '0' + (g_TrackError / 10) % 10);
                                OLED_ShowChar(3, 10, '0' + g_TrackError % 10);
                            }
                            else
                            {
                                OLED_ShowChar(3, 8, '-');
                                int16_t absE = -g_TrackError;
                                OLED_ShowChar(3, 9, '0' + (absE / 10) % 10);
                                OLED_ShowChar(3, 10, '0' + absE % 10);
                            }
                            OLED_ShowString(4, 1, g_ParkingArmed ? "ARMED           " : "WAIT            ");
                        }
                    }
                }

                /* ====== 停车检测（任务2/5/6）====== */
                if (g_TaskIdx == TASK_2_IDX || g_TaskIdx == TASK_5_IDX
                    || g_TaskIdx == TASK_6_IDX)
                {
                    if (!g_ParkingArmed)
                    {
                        if (elapsed >= 2000)
                        {
                            if (GrayTrack_GetActiveCount() < PARKING_THRESH)
                                g_ParkingArmed = 1;
                        }
                    }
                    else
                    {
                        if (GrayTrack_GetActiveCount() >= PARKING_THRESH)
                        {
                            StopMotors();
                            g_LapTimeMs = elapsed;
                            g_State = STATE_COMPLETE;
                            break;
                        }
                    }
                }

                /* ====== 任务4完成检测（A→B距离到） ====== */
                if (g_TaskIdx == TASK_4_IDX)
                {
                    if (g_TravelDistM >= DIST_AB_M)
                    {
                        StopMotors();
                        g_LapTimeMs = elapsed;
                        g_State = STATE_COMPLETE;
                        break;
                    }
                    /* 若 GrayTrack 因离线已停车（距离>10cm才视作有效）*/
                    if (g_TravelDistM > 0.10f && g_BaseSpeedRPS == 0.0f)
                    {
                        g_LapTimeMs = elapsed;
                        g_State = STATE_COMPLETE;
                        break;
                    }
                }
            }

            /* ---- RUNNING 期间：KEY1 中止回 IDLE ---- */
            if (evShort)
            {
                StopMotors();
                g_TaskIdx = TASK_2_IDX;
                g_State = STATE_IDLE;
                OLED_Clear();
                ShowIdleScreen();
            }
        }
        break;

        /* ==========================================================
         *  STATE_COMPLETE — 显示结果，KEY1回IDLE
         * ========================================================== */
        case STATE_COMPLETE:
            if (!g_DisplayReady)
            {
                g_DisplayReady = 1;
                OLED_Clear();

                if (g_TaskIdx == TASK_4_IDX)
                {
                    /* ---- 任务4：显示距离 ---- */
                    OLED_ShowString(1, 1, "** Task4 Done** ");
                    uint32_t d_cm = (uint32_t)(g_TravelDistM * 100.0f + 0.5f);
                    OLED_ShowChar(2, 1, 'D');
                    OLED_ShowChar(2, 2, ':');
                    OLED_ShowChar(2, 3, ' ');
                    OLED_ShowChar(2, 4, '0' + d_cm / 100);
                    OLED_ShowChar(2, 5, '.');
                    OLED_ShowChar(2, 6, '0' + (d_cm % 100) / 10);
                    OLED_ShowChar(2, 7, '0' + d_cm % 10);
                    OLED_ShowChar(2, 8, 'm');
                }
                else
                {
                    /* ---- 任务2/3/5/6：显示时间 ---- */
                    uint32_t s = g_LapTimeMs / 1000;
                    uint32_t d = (g_LapTimeMs % 1000) / 100;
                    switch (g_TaskIdx)
                    {
                    case TASK_2_IDX:
                        OLED_ShowString(1, 1, "** Task2 Done** ");
                        break;
                    case TASK_3_IDX:
                        OLED_ShowString(1, 1, "** Task3 Done** ");
                        break;
                    case TASK_5_IDX:
                        OLED_ShowString(1, 1, "** Task5 Done** ");
                        break;
                    case TASK_6_IDX:
                        OLED_ShowString(1, 1, "** Task6 Done** ");
                        break;
                    }
                    OLED_ShowChar(2, 1, 'T');
                    OLED_ShowChar(2, 2, ':');
                    OLED_ShowChar(2, 3, ' ');
                    OLED_ShowChar(2, 4, '0' + s / 10);
                    OLED_ShowChar(2, 5, '0' + s % 10);
                    OLED_ShowChar(2, 6, '.');
                    OLED_ShowChar(2, 7, '0' + d);
                    OLED_ShowChar(2, 8, 's');
                }
                OLED_ShowString(3, 1, "                ");
                OLED_ShowString(4, 1, "KEY1=Back       ");
            }

            if (evShort)
            {
                g_DisplayReady = 0;
                g_TaskIdx = TASK_2_IDX;
                g_State = STATE_IDLE;
                OLED_Clear();
                ShowIdleScreen();
            }
            break;
        }

    }
}
