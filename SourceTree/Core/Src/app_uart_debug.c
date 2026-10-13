#include "app_uart_debug.h"

#include "app_config.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define APP_LOG_TIMEOUT_MS 50U
#define APP_LOG_BUFFER_LEN 512U
#define APP_UART_RX_RING_LEN 128U

static uint8_t g_log_ready;
static uint8_t g_rx_byte;
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static uint8_t g_rx_ring[APP_UART_RX_RING_LEN];
static app_log_flags_t g_log_flags;

static void app_uart_debug_start_rx(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1U);
}

void app_log_init(void)
{
    g_log_ready = 1U;
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_log_flags.gpsdo2 = GPSDO_LOG_GPSDO2_ENABLE;
    g_log_flags.legacy_gpsdo10 = GPSDO_LOG_LEGACY_GPSDO10_ENABLE;
    g_log_flags.rtc_phase = GPSDO_LOG_RTC_PHASE_ENABLE;
    g_log_flags.phase10 = GPSDO_LOG_PHASE10_ENABLE;
    g_log_flags.event = GPSDO_LOG_EVENT_ENABLE;
    g_log_flags.dac_adjust = GPSDO_LOG_DAC_ADJUST_ENABLE;
    g_log_flags.gpsdo_phase = GPSDO_LOG_GPSDO_PHASE_ENABLE;
    g_log_flags.gpsdo_avg   = GPSDO_LOG_GPSDO_AVG_ENABLE;
    app_uart_debug_start_rx();
}

void app_log_write(const char *text)
{
    size_t len;

    if ((g_log_ready == 0U) || (text == (const char *)0)) {
        return;
    }

    len = strlen(text);
    if (len == 0U) {
        return;
    }

    if (len > 0xFFFFU) {
        len = 0xFFFFU;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, APP_LOG_TIMEOUT_MS);
}

void app_log_printf(const char *fmt, ...)
{
    char buf[APP_LOG_BUFFER_LEN];
    va_list ap;
    int n;

    if ((g_log_ready == 0U) || (fmt == (const char *)0)) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    buf[sizeof(buf) - 1U] = '\0';
    app_log_write(buf);
}

uint8_t app_uart_debug_get_char(uint8_t *ch)
{
    if (ch == (uint8_t *)0) {
        return 0U;
    }

    __disable_irq();
    if (g_rx_head == g_rx_tail) {
        __enable_irq();
        return 0U;
    }

    *ch = g_rx_ring[g_rx_tail];
    g_rx_tail++;
    if (g_rx_tail >= APP_UART_RX_RING_LEN) {
        g_rx_tail = 0U;
    }
    __enable_irq();

    return 1U;
}

static uint8_t app_log_bool(uint8_t enable)
{
    return (enable != 0U) ? 1U : 0U;
}

void app_log_get_flags(app_log_flags_t *flags)
{
    if (flags != (app_log_flags_t *)0) {
        *flags = g_log_flags;
    }
}

void app_log_set_rtc_phase(uint8_t enable)
{
    g_log_flags.rtc_phase = app_log_bool(enable);
}

void app_log_set_phase10(uint8_t enable)
{
    g_log_flags.phase10 = app_log_bool(enable);
}

void app_log_set_phase_detail(uint8_t enable)
{
    app_log_set_rtc_phase(enable);
    app_log_set_phase10(enable);
}

void app_log_set_legacy_gpsdo10(uint8_t enable)
{
    g_log_flags.legacy_gpsdo10 = app_log_bool(enable);
}

void app_log_set_event(uint8_t enable)
{
    g_log_flags.event = app_log_bool(enable);
}

void app_log_set_dac_adjust(uint8_t enable)
{
    g_log_flags.dac_adjust = app_log_bool(enable);
}

uint8_t app_log_gpsdo2_enabled(void)
{
    return g_log_flags.gpsdo2;
}

uint8_t app_log_legacy_gpsdo10_enabled(void)
{
    return g_log_flags.legacy_gpsdo10;
}

uint8_t app_log_rtc_phase_enabled(void)
{
    return g_log_flags.rtc_phase;
}

uint8_t app_log_phase10_enabled(void)
{
    return g_log_flags.phase10;
}

uint8_t app_log_event_enabled(void)
{
    return g_log_flags.event;
}

uint8_t app_log_dac_adjust_enabled(void)
{
    return g_log_flags.dac_adjust;
}

uint8_t app_log_gpsdo_phase_enabled(void)
{
    return g_log_flags.gpsdo_phase;
}

uint8_t app_log_gpsdo_avg_enabled(void)
{
    return g_log_flags.gpsdo_avg;
}

void app_log_print_voltage_uv(uint32_t uv)
{
    app_log_printf("%lu.%04lu",
                   (unsigned long)(uv / 1000000UL),
                   (unsigned long)((uv % 1000000UL) / 100UL));
}

void app_log_print_voltage_uv6(uint32_t uv)
{
    app_log_printf("%lu.%06lu",
                   (unsigned long)(uv / 1000000UL),
                   (unsigned long)(uv % 1000000UL));
}

uint32_t app_log_dac_code_to_uv(uint16_t code)
{
    uint64_t num = ((uint64_t)code * (uint64_t)DAC_VREF_UV) + 32767ULL;
    return (uint32_t)(num / 65535ULL);
}

void app_log_dac_adjust_event(const char *source, uint16_t from_code, uint16_t to_code,
                               uint32_t from_uv, uint32_t to_uv, const char *reason)
{
    int32_t step_uv;

    if ((app_log_event_enabled() == 0U) || (app_log_dac_adjust_enabled() == 0U)) {
        return;
    }

    step_uv = (int32_t)((int64_t)to_uv - (int64_t)from_uv);
    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=DAC_ADJUST,SOURCE=%s,FROM_CODE=%lu,TO_CODE=%lu,FROM_V=",
                   (unsigned long)HAL_GetTick(),
                   (source != (const char *)0) ? source : "?",
                   (unsigned long)from_code,
                   (unsigned long)to_code);
    app_log_print_voltage_uv6(from_uv);
    app_log_write(",TO_V=");
    app_log_print_voltage_uv6(to_uv);
    app_log_printf(",STEP_UV=%ld,REASON=%s\r\n",
                   (long)step_uv,
                   (reason != (const char *)0) ? reason : "?");
}

void app_log_dac_adjust_fail_event(const char *source, uint16_t from_code, uint16_t to_code,
                                   const char *reason, HAL_StatusTypeDef status)
{
    if ((app_log_event_enabled() == 0U) || (app_log_dac_adjust_enabled() == 0U)) {
        return;
    }

    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=DAC_ADJUST_FAIL,SOURCE=%s,FROM_CODE=%lu,TO_CODE=%lu,REASON=%s,STATUS=%lu\r\n",
                   (unsigned long)HAL_GetTick(),
                   (source != (const char *)0) ? source : "?",
                   (unsigned long)from_code,
                   (unsigned long)to_code,
                   (reason != (const char *)0) ? reason : "?",
                   (unsigned long)status);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if ((huart == (UART_HandleTypeDef *)0) || (huart->Instance != USART1)) {
        return;
    }

    next = (uint16_t)(g_rx_head + 1U);
    if (next >= APP_UART_RX_RING_LEN) {
        next = 0U;
    }

    if (next != g_rx_tail) {
        g_rx_ring[g_rx_head] = g_rx_byte;
        g_rx_head = next;
    }

    app_uart_debug_start_rx();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == (UART_HandleTypeDef *)0) || (huart->Instance != USART1)) {
        return;
    }

    app_uart_debug_start_rx();
}
