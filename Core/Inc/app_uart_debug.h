#ifndef APP_UART_DEBUG_H
#define APP_UART_DEBUG_H

#include "main.h"

void app_log_init(void);
void app_log_write(const char *text);
void app_log_printf(const char *fmt, ...);
/* Send a labelled banner at a spread of candidate baud rates so a terminal on
 * the wrong rate can still be matched.  Blocking; call before app_log_init(). */
void app_uart_baud_sweep(void);
/* Re-runs app_uart_baud_sweep() every APP_UART1_BAUD_SWEEP_PERIOD_MS when
 * APP_UART1_BAUD_SWEEP_REPEAT_ENABLE is set; no-op otherwise.  Call from the
 * main loop. */
void app_uart_baud_sweep_tick(void);
/* Measure HCLK against the PC's transmitted byte cadence (see app_config.h).
 * Returns k x 10000, where k = HCLK_real / 72 MHz, or 0 until enough bytes of
 * a back-to-back burst have been seen. */
void app_clk_measure_init(void);
uint32_t app_clk_measure_k_x10000(void);
uint32_t app_clk_measure_frames(void);
uint8_t app_uart_debug_get_char(uint8_t *ch);
/* Number of log lines dropped because the non-blocking TX ring was full. */
uint32_t app_log_get_tx_drops(void);

typedef struct {
    uint8_t gpsdo2;
    uint8_t legacy_gpsdo10;
    uint8_t rtc_phase;
    uint8_t phase10;
    uint8_t event;
    uint8_t dac_adjust;
    uint8_t gpsdo_phase;
    uint8_t gpsdo_avg;
    uint8_t result;
} app_log_flags_t;

void app_log_get_flags(app_log_flags_t *flags);
void app_log_set_rtc_phase(uint8_t enable);
void app_log_set_phase10(uint8_t enable);
void app_log_set_phase_detail(uint8_t enable);
void app_log_set_legacy_gpsdo10(uint8_t enable);
void app_log_set_event(uint8_t enable);
void app_log_set_dac_adjust(uint8_t enable);
void app_log_set_result(uint8_t enable);
uint8_t app_log_result_enabled(void);
uint8_t app_log_gpsdo2_enabled(void);
uint8_t app_log_legacy_gpsdo10_enabled(void);
uint8_t app_log_rtc_phase_enabled(void);
uint8_t app_log_phase10_enabled(void);
uint8_t app_log_event_enabled(void);
uint8_t app_log_dac_adjust_enabled(void);
uint8_t app_log_gpsdo_phase_enabled(void);
uint8_t app_log_gpsdo_avg_enabled(void);
void app_log_print_voltage_uv(uint32_t uv);
void app_log_print_voltage_uv6(uint32_t uv);
uint32_t app_log_dac_code_to_uv(uint16_t code);
void app_log_dac_adjust_event(const char *source, uint16_t from_code, uint16_t to_code,
                              uint32_t from_uv, uint32_t to_uv, const char *reason);
void app_log_dac_adjust_fail_event(const char *source, uint16_t from_code, uint16_t to_code,
                                   const char *reason, HAL_StatusTypeDef status);

#endif /* APP_UART_DEBUG_H */
