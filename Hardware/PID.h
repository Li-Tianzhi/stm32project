#ifndef __PID_H  
#define __PID_H

#include "stm32f10x.h"

/* ====== 内环速度PID参数 ====== */
#define PID_KP              160.0f   // 比例增益 — 降低防弯道拍死到0
#define PID_KI              1.0f    // 积分增益 — 消除静差，过大会超调
#define PID_KD              0.3f    // 微分增益 — 抑制震荡，过大会引入噪声
#define PID_INTEGRAL_LIMIT  150.0f   // 积分累计上限，防积分饱和
#define PID_OUTPUT_LIMIT    100.0f  // 输出限幅，匹配PWM 0~100

/* ====== 外环循迹PID参数 ====== */
#define TRACK_KP             0.14f   // 循迹比例增益 — 误差3即输出0.42rps，R=0.5m够用
#define TRACK_KI             0.0f    // 循迹积分增益
#define TRACK_KD             0.12f   // 循迹微分阻尼 — 加大抑制直线微晃
#define TRACK_INTEGRAL_LIMIT 10.0f
#define TRACK_OUTPUT_LIMIT   3.0f   // 最大差速修正(rps)，Kp*MaxError=0.3*7=2.1，放宽让修正随误差变化
      
/* ====== PID控制器结构体 ====== */
typedef struct {
    /* 参数 */
    float Kp, Ki, Kd;
    float IntegralLimit, OutputLimit;

    /* 状态 */
    float Integral;     // 积分累计
    float PrevError;    // 上周期偏差
    float Out;          // 上周起输出
} PID_t;

/* ====== 全局PID实例 + 目标转速（左轮/右轮独立） ====== */
extern PID_t g_PID_Left;
extern PID_t g_PID_Right;
extern PID_t g_PID_Track;       // 外环循迹PID
extern float g_PID_TargetL;     // TIM6 ISR内读取，主循环写入
extern float g_PID_TargetR;

/* ====== API ====== */
void PID_Init(PID_t *pid, float Kp, float Ki, float Kd,
              float ilimit, float olimit);
float PID_Calc(PID_t *pid, float target, float actual);
void PID_Reset(PID_t *pid);     // 清积分/偏差，不停车调用
void PID_ClearI(PID_t *pid);    // 仅清积分，换向时用

#endif
