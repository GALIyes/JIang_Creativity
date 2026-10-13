#include "app_timebase.h"

#include "app_config.h"
#include "tim.h"

static volatile uint8_t g_flag_20ms;
static volatile uint8_t g_flag_1s;
static uint8_t g_tim3_div20;
static uint8_t g_tim3_div1s;
/* Free-running TIM3 interrupt counter, used to cross-check this timebase
 * against HAL_GetTick() (see app_gpsdo10_print_gps_stat). */
static volatile uint32_t g_tim3_tick_count;

/*
 * TIM3's prescaler must be derived here, not inherited from tim.c.
 *
 * MX_TIM3_Init() carries "Prescaler = 71999", which was meant to divide a
 * 72 MHz timer clock down to 1 kHz.  Two things are wrong with it:
 *
 *   1. TIM3 is a 16 bit timer, so PSC only holds 0..65535.  Writing 71999
 *      silently truncates to 6431 (71999 == 0x1193F -> 0x193F), which makes the
 *      counter clock 9552 Hz instead of 1 kHz and the "10 ms" interrupt
 *      1.047 ms:
 *          measured 1 s flag period = 100 x 1.047 ms = 104.7 ms
 *      -- which is exactly the 106 ms gap seen between two consecutive
 *      GPSDO_RESULT rows (MS=19074 then MS=19180) on the real board.
 *   2. Even without the overflow it would still be wrong whenever SYSCLK
 *      changes, because the divider is hardcoded for 72 MHz.
 *
 * So compute it from the actual timer clock, widening the auto-reload period
 * if the prescaler would not fit in 16 bits.
 */
HAL_StatusTypeDef app_timebase_init(void)
{
    uint32_t hclk;
    uint32_t tim_clk;
    uint32_t period_ticks;
    uint32_t psc;

    g_flag_20ms = 0U;
    g_flag_1s = 0U;
    g_tim3_div20 = 0U;
    g_tim3_div1s = 0U;
    g_tim3_tick_count = 0UL;

    hclk = HAL_RCC_GetHCLKFreq();
    tim_clk = HAL_RCC_GetPCLK1Freq();
    if (tim_clk != hclk) {
        tim_clk *= 2U;              /* APB1 timer clock doubler */
    }

    period_ticks = htim3.Init.Period + 1U;
    if (period_ticks == 0U) {
        period_ticks = 10U;
    }

    psc = tim_clk / (APP_TIM3_IRQ_HZ * period_ticks);
    while ((psc > 65536U) && (period_ticks < 65536U)) {
        period_ticks *= 2U;
        if (period_ticks > 65536U) {
            period_ticks = 65536U;
        }
        htim3.Init.Period = period_ticks - 1U;
        psc = tim_clk / (APP_TIM3_IRQ_HZ * period_ticks);
    }
    if (psc == 0U) {
        psc = 1U;
    }
    htim3.Init.Prescaler = psc - 1U;

    /* Re-applies PSC/ARR on the already initialised handle (MspInit is skipped
     * while the state is READY, so the GPIO/NVIC setup is not repeated). */
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_TIM_Base_Start_IT(&htim3);
}

uint8_t app_timebase_take_20ms_flag(void)
{
    uint8_t ret;

    __disable_irq();
    ret = g_flag_20ms;
    g_flag_20ms = 0U;
    __enable_irq();

    return ret;
}

uint8_t app_timebase_take_1s_flag(void)
{
    uint8_t ret;

    __disable_irq();
    ret = g_flag_1s;
    g_flag_1s = 0U;
    __enable_irq();

    return ret;
}

uint32_t app_timebase_get_tick_count(void)
{
    return g_tim3_tick_count;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == (TIM_HandleTypeDef *)0) || (htim->Instance != TIM3)) {
        return;
    }

    g_tim3_tick_count++;

    g_tim3_div20++;
    if (g_tim3_div20 >= APP_TICK20_TIM3_DIV) {
        g_tim3_div20 = 0U;
        g_flag_20ms = 1U;
    }

    g_tim3_div1s++;
    if (g_tim3_div1s >= APP_TIM3_IRQ_HZ) {
        g_tim3_div1s = 0U;
        g_flag_1s = 1U;
    }
}
