#include "app_pps.h"

#define PPS_NO_TICK_MS          0xFFFFFFFFUL
#define PPS_INTERVAL_MIN_MS     900UL
#define PPS_INTERVAL_MAX_MS    1100UL

static volatile uint32_t g_pps_count;
static volatile uint32_t g_pps_last_tick_ms;
static volatile uint32_t g_pps_interval_ms;
static volatile uint32_t g_pps_invalid_interval_count;
static volatile uint8_t g_pps_valid_interval_streak;

void app_pps_init(void)
{
    g_pps_count = 0UL;
    g_pps_last_tick_ms = PPS_NO_TICK_MS;
    g_pps_interval_ms = 0UL;
    g_pps_invalid_interval_count = 0UL;
    g_pps_valid_interval_streak = 0U;
}

void app_pps_get_snapshot(app_pps_snapshot_t *snapshot)
{
    if (snapshot == (app_pps_snapshot_t *)0) {
        return;
    }

    __disable_irq();
    snapshot->count = g_pps_count;
    snapshot->last_tick_ms = g_pps_last_tick_ms;
    snapshot->interval_ms = g_pps_interval_ms;
    snapshot->invalid_interval_count = g_pps_invalid_interval_count;
    snapshot->valid_interval_streak = g_pps_valid_interval_streak;
    __enable_irq();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now;

    if (GPIO_Pin != GPS_PPS_Pin) {
        return;
    }

    now = HAL_GetTick();
    if (g_pps_last_tick_ms != PPS_NO_TICK_MS) {
        g_pps_interval_ms = now - g_pps_last_tick_ms;
        if ((g_pps_interval_ms >= PPS_INTERVAL_MIN_MS) &&
            (g_pps_interval_ms <= PPS_INTERVAL_MAX_MS)) {
            if (g_pps_valid_interval_streak < 255U) {
                g_pps_valid_interval_streak++;
            }
        } else {
            g_pps_valid_interval_streak = 0U;
            g_pps_invalid_interval_count++;
        }
    }
    g_pps_last_tick_ms = now;
    g_pps_count++;
}
