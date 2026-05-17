#include "sweep.h"
#include "./TIMER/gtim.h"
#include "./AD9959/ad9959.h"
#include "stm32f1xx_hal.h"

/* 全局扫频变量 */
SweepFreq_t Sweep;

/* A3 高低电平消抖计数 */
static uint8_t high_cnt = 0;
static uint8_t low_cnt  = 0;

/* GPIO 端口定义 */
#define SWEEP_STOP_GPIO_PORT   GPIOA
#define SWEEP_STOP_GPIO_PIN    GPIO_PIN_3
#define HIGH_COUNT_THRESHOLD   3
#define LOW_COUNT_THRESHOLD    10

/* 读取 A3 电平（HAL 方式） */
static uint8_t Sweep_ReadA3Level(void)
{
    return (HAL_GPIO_ReadPin(SWEEP_STOP_GPIO_PORT, SWEEP_STOP_GPIO_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

/* 消抖处理，返回稳定电平 */
uint8_t Sweep_ReadLevel_Debounced(void)
{
    uint8_t level = Sweep_ReadA3Level();

    if(level == 1)
    {
        if(high_cnt < HIGH_COUNT_THRESHOLD) high_cnt++;
        low_cnt = 0;
    }
    else
    {
        if(low_cnt < LOW_COUNT_THRESHOLD) low_cnt++;
        high_cnt = 0;
    }

    if(high_cnt >= HIGH_COUNT_THRESHOLD)
    {
        high_cnt = HIGH_COUNT_THRESHOLD;
        return 1;
    }
    if(low_cnt >= LOW_COUNT_THRESHOLD)
    {
        low_cnt = LOW_COUNT_THRESHOLD;
        return 0;
    }

    return Sweep.last_level;
}

/* 输出频率到 AD9959 */
void Sweep_OutputFrequency(uint32_t freq)
{
    ad9959_set_signal_out(0, freq, 0, 1023); // 0通道输出
	ad9959_set_signal_out(1, freq, 0, 1023); // 0通道输出
}

/* 初始化 Sweep */
void Sweep_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 初始化 A3 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin  = SWEEP_STOP_GPIO_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(SWEEP_STOP_GPIO_PORT, &GPIO_InitStructure);

    Sweep.start_freq    = 800000;
    Sweep.stop_freq     = 1000000;
    Sweep.step_freq     = 1000000;
    Sweep.dwell_ms      = 10;
    Sweep.current_freq  = Sweep.start_freq;
    Sweep.running       = 0;
    Sweep.finished      = 0;
    Sweep.stop_by_level = 0;
    Sweep.re_scan       = 0;
    Sweep.last_level    = Sweep_ReadA3Level();
}

/* 开始扫频 */
void Sweep_Start(void)
{
    Sweep.current_freq   = Sweep.start_freq;
    Sweep.running        = 1;
    Sweep.finished       = 0;
    Sweep.stop_by_level  = 0;
    Sweep.re_scan        = 0;
    Sweep.last_level     = Sweep_ReadLevel_Debounced();

    Sweep_OutputFrequency(Sweep.current_freq);

    /* 启动 HAL 定时器中断 */
    __HAL_TIM_SET_COUNTER(&g_timx_handle, 0);
    HAL_TIM_Base_Start_IT(&g_timx_handle);
}

/* 停止扫频 */
void Sweep_Stop(void)
{
    Sweep.running = 0;
    HAL_TIM_Base_Stop_IT(&g_timx_handle);
}

/* 高电平停扫触发 */
void Sweep_StopByLevel(void)
{
    Sweep.running       = 0;
    Sweep.finished      = 0;
    Sweep.stop_by_level = 1;
    Sweep.re_scan       = 0;
    Sweep.last_level    = 1;
    HAL_TIM_Base_Stop_IT(&g_timx_handle);
}

/* 检测 A3 高电平停扫（立即停扫） */
void Sweep_Check_Level_Stop(void)
{
    if(Sweep_ReadLevel_Debounced() == 1)
    {
        Sweep_StopByLevel();
    }
}

/* 检测 A3 低电平稳定后重新扫频 */
void Sweep_Check_Level(void)
{
    uint8_t level = Sweep_ReadLevel_Debounced();

    if(Sweep.running)
    {
        if(level == 1)
        {
            Sweep.last_level = 1;
            Sweep_StopByLevel();
        }
    }
    else
    {
        if((Sweep.stop_by_level == 1) && (Sweep.last_level == 1) && (level == 0))
        {
            Sweep.re_scan = 1;
            Sweep.stop_by_level = 0;
        }
    }

    Sweep.last_level = level;
}

/* 扫频步进处理 */
void Sweep_TimerHandler(void)
{
    if(Sweep.running == 0) return;

    if(Sweep.current_freq + Sweep.step_freq > Sweep.stop_freq)
    {
        Sweep.running  = 0;
        Sweep.finished = 1;
        HAL_TIM_Base_Stop_IT(&g_timx_handle);
        return;
    }

    Sweep.current_freq += Sweep.step_freq;
    Sweep_OutputFrequency(Sweep.current_freq);
}

/* 设置扫频参数 */
void Sweep_SetParam(uint32_t start_freq, uint32_t stop_freq, uint32_t step_freq, uint16_t dwell_ms)
{
    if(start_freq == 0 || stop_freq == 0 || step_freq == 0 || dwell_ms == 0) return;
    if(start_freq >= stop_freq) return;

    Sweep.start_freq   = start_freq;
    Sweep.stop_freq    = stop_freq;
    Sweep.step_freq    = step_freq;
    Sweep.dwell_ms     = dwell_ms;
    Sweep.current_freq = start_freq;
}

/* 更新定时器周期（HAL 定时器使用，可空实现） */
void Sweep_UpdateTimerPeriod(uint16_t dwell_ms)
{
    (void)dwell_ms;
}

/* 获取当前频率 */
uint32_t Sweep_GetCurrentFreq(void)
{
    return Sweep.current_freq;
}

/* 判断扫频是否运行 */
uint8_t Sweep_IsRunning(void)
{
    return Sweep.running;
}

/* 判断扫频是否完成 */
uint8_t Sweep_IsFinished(void)
{
    return Sweep.finished;
}

/* 判断是否高电平停扫触发 */
uint8_t Sweep_IsStopByLevel(void)
{
    return Sweep.stop_by_level;
}

/* 判断是否需要重新扫频 */
uint8_t Sweep_NeedRescan(void)
{
    return Sweep.re_scan;
}

/* 清除重新扫频标志 */
void Sweep_ClearRescanFlag(void)
{
    Sweep.re_scan = 0;
}

