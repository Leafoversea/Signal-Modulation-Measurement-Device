#ifndef __SWEEP_H
#define __SWEEP_H

#include "stdint.h"   // HAL 环境下使用标准整数类型

typedef struct
{
    uint32_t start_freq;    // 起始频率
    uint32_t stop_freq;     // 结束频率
    uint32_t step_freq;     // 步进频率
    uint16_t dwell_ms;      // 驻留时间(ms)
    uint32_t current_freq;  // 当前频率
    uint8_t running;        // 扫频运行标志
    uint8_t finished;       // 扫频完成标志
    uint8_t stop_by_level;  // A3 高电平停扫标志
    uint8_t re_scan;        // 重新扫频标志
    uint8_t last_level;     // 上次 A3 电平状态
} SweepFreq_t;

extern SweepFreq_t Sweep;

void Sweep_Init(void);
void Sweep_Start(void);
void Sweep_Stop(void);
void Sweep_StopByLevel(void);
void Sweep_Check_Level_Stop(void);
void Sweep_Check_Level(void);
void Sweep_TimerHandler(void);
void Sweep_SetParam(uint32_t start_freq, uint32_t stop_freq, uint32_t step_freq, uint16_t dwell_ms);
void Sweep_UpdateTimerPeriod(uint16_t dwell_ms);
uint8_t Sweep_ReadLevel_Debounced(void);
uint32_t Sweep_GetCurrentFreq(void);
uint8_t Sweep_IsRunning(void);
uint8_t Sweep_IsFinished(void);
uint8_t Sweep_IsStopByLevel(void);
uint8_t Sweep_NeedRescan(void);
void Sweep_ClearRescanFlag(void);

#endif

