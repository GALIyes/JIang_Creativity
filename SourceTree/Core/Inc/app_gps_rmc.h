#ifndef APP_GPS_RMC_H
#define APP_GPS_RMC_H

#include "main.h"

#define RMC_LINE_MAX 128U

typedef struct {
    uint8_t utc_hour;
    uint8_t utc_min;
    uint8_t utc_sec;
    uint16_t day;
    uint16_t month;
    uint16_t year;
} app_rmc_time_t;

typedef struct {
    app_rmc_time_t bj;
    uint8_t rtc_valid;
    uint8_t rtc_synced;
    uint16_t sec_since_sync;
    uint8_t rmc_valid;
} app_rtc_state_t;

void app_rmc_init(void);
void app_rmc_store_line(const char *line);
void app_rmc_process_pending(void);
uint8_t app_rmc_check_checksum(const char *line);
uint8_t app_rmc_parse_time(const char *line, app_rmc_time_t *t);
void app_rmc_utc_to_beijing(const app_rmc_time_t *utc, app_rmc_time_t *bj);
void app_rtc_tick_second(void);
const app_rtc_state_t *app_rtc_get_state(void);
uint32_t app_rmc_get_checksum_ok(void);
uint32_t app_rmc_get_checksum_fail(void);
uint32_t app_rmc_get_parse_fail(void);

#endif /* APP_GPS_RMC_H */
