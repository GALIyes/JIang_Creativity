#ifndef APP_GPSDO10_H
#define APP_GPSDO10_H

#include "main.h"

typedef struct {
    uint8_t valid;
    uint32_t seq0;
    uint32_t seq1;
    uint32_t ms0;
    uint32_t ms1;
    int32_t tick0;
    int32_t tick1;
    uint32_t dt_seconds;
    int32_t dtick;
    int32_t slope_x1000;
    int32_t freq_err_mhz;
    uint8_t disc10_valid;
    uint8_t valid_windows;
    uint32_t count10;
    int32_t err10;
    uint16_t dac_code;
    uint32_t dacv_x10000;
} app_phase10_diag_t;

void app_gpsdo10_init(void);
void app_gpsdo10_poll_1s(void);
void app_gpsdo10_print_phase_stat(void);
uint8_t app_gpsdo10_get_phase10(app_phase10_diag_t *out);
void app_gpsdo10_reset_phase_history(void);
uint8_t app_gpsdo10_phase10_is_valid(void);

#endif /* APP_GPSDO10_H */
