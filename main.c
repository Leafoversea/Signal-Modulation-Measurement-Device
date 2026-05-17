//***************************************************************************************
//
// 程序实验: AD9959扫频/扫相/扫幅测试实验
// 作　　者: 科一电子
// 日    期: 2020-11-11
// 实验平台: 科一电子  STM32F103核心板  2.8寸液晶屏
// 说　　明:
//
//***************************************************************************************

#include "stm32f1xx_hal.h"
#include "stdio.h"
#include "./LED/led.h"
#include "./KEY/key.h"
#include "./AD9959/ad9959.h"
#include "./TIMER/gtim.h"
#include "sweep.h"
#include "./LCD/lcd.h"

char str[32];

/*
 * ??????????????,????????
 */
#define FIXED_OUT_CH        2

/*
 * AD9959 ???
 */
#define AD9959_FULL_AMP     1023

/*
 * ????????:
 * F = SWP + 200kHz
 *
 * ??:?? 200kHz ?????,???????
 */
#define DUT_FREQ_OFFSET_HZ  200000UL

#define FREQ_SEL_INVALID    0xFF

/*
 * ?????? GPIO ???
 *
 * ?????:
 * ?? -> ??:E2 E4 E0 B8
 *
 * PE2 -> bit3
 * PE4 -> bit2
 * PE0 -> bit1
 * PB8 -> bit0
 *
 * ????:
 * 0000 -> ???
 * 0001 -> 1kHz
 * 0010 -> 2kHz
 * ...
 * 1001 -> 9kHz
 * 1010 -> 10kHz
 *
 * 1011~1111 -> ????,????????
 */
static void FreqSel_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*
     * PE2?PE4?PE0
     */
    GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*
     * PB8
     */
    GPIO_InitStruct.Pin   = GPIO_PIN_8;
    GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*
 * ??????
 *
 * ?? -> ??:E2 E4 E0 B8
 *
 * PE2 -> bit3
 * PE4 -> bit2
 * PE0 -> bit1
 * PB8 -> bit0
 */
static uint8_t FreqSel_ReadCode(void)
{
    uint8_t code = 0;

    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_SET)
    {
        code |= 0x08;
    }

    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET)
    {
        code |= 0x04;
    }

    if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_SET)
    {
        code |= 0x02;
    }

    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)
    {
        code |= 0x01;
    }

    return code;
}

/*
 * ?????
 *
 * E2 E4 E0 B8
 *
 * 0000 -> ???
 * 0001 -> 1kHz
 * 0010 -> 2kHz
 * 0011 -> 3kHz
 * 0100 -> 4kHz
 * 0101 -> 5kHz
 * 0110 -> 6kHz
 * 0111 -> 7kHz
 * 1000 -> 8kHz
 * 1001 -> 9kHz
 * 1010 -> 10kHz
 *
 * 1011~1111 -> ????,??????
 */
static uint32_t FreqSel_CodeToFreq(uint8_t code)
{
    if(code == 0)
    {
        return 0;              // 0000 -> ???
    }

    if(code <= 10)
    {
        return ((uint32_t)code) * 1000UL;
    }

    return 0;                  //)
    {
        return ((uint32_t)code) * 1000UL;
    }

    return 0;                  // 1011~1111 -> ????,??????
}

/*
 * ??????????
 *
 * ???????????? AD9959,????????????
 */
static void FreqSel_Process(void)
{
    static uint8_t last_code = FREQ_SEL_INVALID;

    uint8_t code;
    uint32_t out_freq;

    code = FreqSel_ReadCode();

    if(code == last_code)
    {
        return;
    }

    last_code = code;

    out_freq = FreqSel_CodeToFreq(code);

    /*
     * 0000 ? 1011~1111:?????????
     * ??:??? 1000Hz,??? 0
     */
    if(out_freq == 0)
    {
        ad9959_set_signal_out(FIXED_OUT_CH, 1000, 0, 0);
        IO_update();
        return;
    }

    /*
     * 0001~1010:????????????
     *
     * 0001 -> 1kHz
     * 0010 -> 2kHz
     * ...
     * 1010 -> 10kHz
     */
    ad9959_set_signal_out(FIXED_OUT_CH, (double)out_freq, 0, AD9959_FULL_AMP);
    IO_update();
}

static void PB0_Toggle_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

static void PA1_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
}

int main(void)
{
    uint16_t key = 0;
    uint16_t cnt = 0;
    uint16_t pb0_cnt = 0;
    uint32_t freq = 0;
    uint32_t dut_freq = 0;

    HAL_Init();
    SystemClock_Config();

    LED_Init();
    KEY_Init();
    LCD_Init();
    LCD_Clear(WHITE);

    ad9959_gpio_init();
    ad9959_init();

    /*
     * ??? PE2?PE4?PE0?PB8 ?????????
     */
    FreqSel_GPIO_Init();

    Sweep_Init();
    gtim_timx_int_init(100-1, 72-1);

    Sweep_SetParam(800000, 199800000, 200000, 60);

    LCD_Show_String(10, 40, 240, 24, 24, "AD9959 Manual Sweep", RED);
    LCD_Show_String(10, 80, 240, 16, 16, "CH0: 0.8M-199.8M ", BLUE);
    LCD_Show_String(10, 110, 240, 16, 16, "State: Running ", BLUE);

    /*
     * F:????????
     * SWP:????????,???
     */
    LCD_Show_String(10, 130, 240, 16, 16, "F: waiting     ", BLUE);
    LCD_Show_String(10, 150, 240, 16, 16, "SWP: ---.---MHz", BLUE);

    Sweep_Start();
    PB0_Toggle_Init();

    /*
     * ???????? E2/E4/E0/B8 ????????????
     */
    FreqSel_Process();

    while(1)
    {
        key = KEY_Scan();

        /*
         * ???? E2?E4?E0?B8
         * ????????
         *
         * ?? -> ??:E2 E4 E0 B8
         *
         * 0000 -> ???
         * 0001 -> 1kHz
         * 0010 -> 2kHz
         * 0011 -> 3kHz
         * 0100 -> 4kHz
         * 0101 -> 5kHz
         * 0110 -> 6kHz
         * 0111 -> 7kHz
         * 1000 -> 8kHz
         * 1001 -> 9kHz
         * 1010 -> 10kHz
         *
         * 1011~1111 -> ????,??????
         */
        FreqSel_Process();

        /* ?????? */
        if(key == K2_PRES)
        {
            Sweep_Stop();
            Sweep_ClearRescanFlag();
            LCD_Show_String(10, 110, 240, 16, 16, "State: Stop    ", BLUE);
            LCD_Show_String(10, 130, 240, 16, 16, "F: waiting     ", BLUE);
        }

        /*
         * A3 ????????,????????
         */
        if((Sweep_IsRunning() == 0) && (Sweep_IsStopByLevel() == 1))
        {
            Sweep_Check_Level();
        }

        /*
         * ??????????
         */
        if(Sweep_NeedRescan() == 1)
        {
            Sweep_ClearRescanFlag();
            Sweep_SetParam(800000, 199800000, 200000, 60);
            Sweep_Start();
            LCD_Show_String(10, 110, 240, 16, 16, "State: ReSweep ", BLUE);
            LCD_Show_String(10, 130, 240, 16, 16, "F: waiting     ", BLUE);
            LCD_Show_String(10, 150, 240, 16, 16, "SWP: ---.---MHz", BLUE);
        }

        /*
         * ????
         */
        cnt++;
        if(cnt > 10)
        {
            cnt = 0;

            /*
             * SWP:????????,????,????
             */
            freq = Sweep_GetCurrentFreq();

            sprintf((char *)str,
                    "SWP:%lu.%03luMHz   ",
                    (unsigned long)(freq / 1000000),
                    (unsigned long)((freq % 1000000) / 1000));

            LCD_Show_String(10, 150, 240, 16, 16, str, BLUE);

            /*
             * F:????
             *
             * ?????????
             * ????:Sweep_IsStopByLevel() == 1
             *
             * F = SWP + ?? 200kHz
             */
            if(Sweep_IsStopByLevel() == 1)
            {
                dut_freq = freq + DUT_FREQ_OFFSET_HZ;

                sprintf((char *)str,
                        "F:%lu.%03luMHz    ",
                        (unsigned long)(dut_freq / 1000000),
                        (unsigned long)((dut_freq % 1000000) / 1000));

                LCD_Show_String(10, 130, 240, 16, 16, str, BLUE);
            }
            else
            {
                LCD_Show_String(10, 130, 240, 16, 16, "F: waiting     ", BLUE);
            }

            /*
             * ????
             */
            if(Sweep_IsRunning() == 1)
            {
                LCD_Show_String(10, 110, 240, 16, 16, "State: Running ", BLUE);
            }
            else if(Sweep_IsStopByLevel() == 1)
            {
                LCD_Show_String(10, 110, 240, 16, 16, "State: Locked  ", BLUE);
            }
            else if(Sweep_IsFinished() == 1)
            {
                LCD_Show_String(10, 110, 240, 16, 16, "State: Finish  ", BLUE);
            }
            else
            {
                LCD_Show_String(10, 110, 240, 16, 16, "State: Stop    ", BLUE);
            }

            LED1_TOGGLE;
//            PB0_Toggle();
        }

        /*
         * ??????????
         */
        pb0_cnt++;
        if(pb0_cnt >= 1000)
        {
            pb0_cnt = 0;
            PA1_Toggle();
        }

        HAL_Delay(5);
    }
}