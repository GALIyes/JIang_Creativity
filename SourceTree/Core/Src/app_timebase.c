#include "app_timebase.h"

#include "app_config.h"
#include "tim.h"

static volatile uint8_t g_flag_20ms;
static volatile uint8_t g_flag_1s;
static uint8_t g_tim3_div20;
static uint8_t g_tim3_div1s;

HAL_StatusTypeDef app_timebase_init(void)
{
    g_flag_20ms = 0U;
    g_flag_1s = 0U;
    g_tim3_div20 = 0U;
    g_tim3_div1s = 0U;

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim == (TIM_HandleTypeDef *)0) || (htim->Instance != TIM3)) {
        return;
    }

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
