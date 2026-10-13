#include "app_gps_uart.h"

#include "app_gps_rmc.h"
#include "usart.h"

#include <string.h>

#define GPS_DMA_RX_LEN 256U
#define GPS_LINE_LEN   96U

static uint8_t g_gps_dma_rx[GPS_DMA_RX_LEN];
static uint16_t g_gps_old_pos;
static char g_gps_line[GPS_LINE_LEN];
static uint16_t g_gps_line_len;
static uint8_t g_gps_started;
static app_gps_uart_stats_t g_gps_stats;

static void app_gps_handle_line(const char *line)
{
    if ((line == (const char *)0) || (line[0] == '\0')) {
        return;
    }

    g_gps_stats.nmea_lines++;
    if ((strncmp(line, "$GNRMC", 6U) == 0) || (strncmp(line, "$GPRMC", 6U) == 0)) {
        g_gps_stats.rmc_lines++;
        g_gps_stats.last_rmc_tick_ms = HAL_GetTick();
        app_rmc_store_line(line);
    }
}

static void app_gps_handle_byte(uint8_t b)
{
    g_gps_stats.rx_bytes++;
    if (b == (uint8_t)'$') {
        g_gps_stats.dollar_count++;
        g_gps_line_len = 0U;
    }

    if (b == (uint8_t)'\r') {
        return;
    }

    if (b == (uint8_t)'\n') {
        g_gps_line[g_gps_line_len] = '\0';
        app_gps_handle_line(g_gps_line);
        g_gps_line_len = 0U;
        return;
    }

    if (g_gps_line_len < (GPS_LINE_LEN - 1U)) {
        g_gps_line[g_gps_line_len] = (char)b;
        g_gps_line_len++;
    } else {
        g_gps_stats.overflow_lines++;
        g_gps_line_len = 0U;
    }
}

HAL_StatusTypeDef app_gps_uart_init(void)
{
    HAL_StatusTypeDef st;

    memset(&g_gps_stats, 0, sizeof(g_gps_stats));
    g_gps_old_pos = 0U;
    g_gps_line_len = 0U;
    g_gps_started = 0U;

    st = HAL_UART_Receive_DMA(&huart2, g_gps_dma_rx, GPS_DMA_RX_LEN);
    if (st == HAL_OK) {
        g_gps_started = 1U;
        if (huart2.hdmarx != (DMA_HandleTypeDef *)0) {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
    }

    return st;
}

void app_gps_uart_poll(void)
{
    uint16_t pos;

    if ((g_gps_started == 0U) || (huart2.hdmarx == (DMA_HandleTypeDef *)0)) {
        return;
    }

    pos = (uint16_t)(GPS_DMA_RX_LEN - (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx));
    if (pos >= GPS_DMA_RX_LEN) {
        pos = 0U;
    }

    while (g_gps_old_pos != pos) {
        app_gps_handle_byte(g_gps_dma_rx[g_gps_old_pos]);
        g_gps_old_pos++;
        if (g_gps_old_pos >= GPS_DMA_RX_LEN) {
            g_gps_old_pos = 0U;
        }
    }
}

void app_gps_uart_get_stats(app_gps_uart_stats_t *stats)
{
    if (stats == (app_gps_uart_stats_t *)0) {
        return;
    }

    *stats = g_gps_stats;
}
