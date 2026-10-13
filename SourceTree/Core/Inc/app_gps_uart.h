#ifndef APP_GPS_UART_H
#define APP_GPS_UART_H

#include "main.h"

typedef struct {
    uint32_t rx_bytes;
    uint32_t nmea_lines;
    uint32_t rmc_lines;
    uint32_t overflow_lines;
    uint32_t dollar_count;
    uint32_t last_rmc_tick_ms;
} app_gps_uart_stats_t;

HAL_StatusTypeDef app_gps_uart_init(void);
void app_gps_uart_poll(void);
void app_gps_uart_get_stats(app_gps_uart_stats_t *stats);

#endif /* APP_GPS_UART_H */
