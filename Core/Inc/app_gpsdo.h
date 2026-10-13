#ifndef APP_GPSDO_H
#define APP_GPSDO_H

#include "main.h"

typedef enum {
    APP_DISC_STATE_WARM = 0,
    APP_DISC_STATE_TRACK10,
    APP_DISC_STATE_NORMAL,
    APP_DISC_STATE_HOLDOVER,
    APP_DISC_STATE_MANUAL
} app_discipline_state_t;

typedef struct {
    uint8_t enable;
    uint8_t applied;
    int32_t step_uv;
    uint32_t last_ms;
    const char *reason;
    int32_t last_step_uv;
    uint32_t last_adjust_ms;
    int8_t confirm_dir;
    uint8_t confirm_count;
} app_phase_fine_status_t;

typedef struct {
    uint8_t enable;
    uint8_t applied;
    int32_t target_ns;
    int32_t raw_ns;
    int32_t corr_ns;
    int32_t step_cnt;
    int32_t step_ns;
    uint32_t phase_status;
    uint32_t phase_seq;
    int32_t phase_ticks;
    uint8_t phase_valid;
    const char *reason;
    int8_t confirm_dir;
    uint8_t confirm_count;
} app_rtc_phase_step_status_t;

void app_gpsdo_init(void);
void app_gpsdo_poll_1s(void);
void app_gpsdo_on_manual_dac_set(uint16_t code, uint32_t uv, uint8_t restart_auto);
app_discipline_state_t app_gpsdo_get_state(void);
const char *app_gpsdo_get_state_string(void);
uint8_t app_gpsdo_get_last_disc10(uint32_t *count10, int32_t *err10);
uint8_t app_gpsdo_get_last_disc10_info(uint8_t *seq10, uint32_t *count10, int32_t *err10);
uint8_t app_gpsdo_get_last_disc10_snapshot(uint8_t *seq10, uint32_t *count10,
                                           int32_t *err10, const char **state);
uint8_t app_gpsdo_get_last_disc50(uint32_t *count50_msp, int32_t *err50);
uint8_t app_gpsdo_get_sw_e50(int32_t *err50_out);
uint8_t app_gpsdo_get_sw_e50_valid(void);
int32_t app_gpsdo_get_sw_e50_last(void);
uint8_t app_gpsdo_get_sw_e50_new(void);
uint8_t app_gpsdo_get_sw_e50_count(void);
/* Monotonic: increments once per completed 50 s SW_E50 result.  A change since
 * the previous read means a new window result is available.  Never reset with
 * the accumulators, so it cannot be confused with a window reset. */
uint32_t app_gpsdo_get_sw_e50_result_seq(void);

uint8_t app_gpsdo_get_sw_e100(int32_t *out);
uint8_t app_gpsdo_get_sw_e100_valid(void);
int32_t app_gpsdo_get_sw_e100_last(void);
uint8_t app_gpsdo_get_sw_e100_new(void);
uint8_t app_gpsdo_get_sw_e100_count(void);

uint8_t app_gpsdo_get_sw_e200(int32_t *out);
uint8_t app_gpsdo_get_sw_e200_valid(void);
int32_t app_gpsdo_get_sw_e200_last(void);
uint8_t app_gpsdo_get_sw_e200_new(void);
uint8_t app_gpsdo_get_sw_e200_count(void);

uint8_t app_gpsdo_get_sw_e1000(int32_t *out);
uint8_t app_gpsdo_get_sw_e1000_valid(void);
int32_t app_gpsdo_get_sw_e1000_last(void);
uint8_t app_gpsdo_get_sw_e1000_new(void);
uint8_t app_gpsdo_get_sw_e1000_count(void);
uint8_t app_gpsdo_get_disc10_applied(void);
int32_t app_gpsdo_get_disc10_step_code(void);
const char *app_gpsdo_get_disc10_reason(void);
uint8_t app_gpsdo_get_disc50_applied(void);
int32_t app_gpsdo_get_disc50_step_code(void);
const char *app_gpsdo_get_disc50_reason(void);
uint8_t app_gpsdo_get_disc50_confirm_count(void);
uint32_t app_gpsdo_get_disc50_last_ms(void);
uint8_t app_gpsdo_get_disc_cooldown_windows(void);
uint32_t app_gpsdo_get_last_disc_adjust_ms(void);
int32_t app_gpsdo_get_last_disc_step_code(void);
uint8_t app_gpsdo_get_phase_fine_zone(void);
uint8_t app_gpsdo_get_pz_state_ok(void);
uint8_t app_gpsdo_get_pz_gps_ok(void);
uint8_t app_gpsdo_get_pz_e10_ok(void);
uint8_t app_gpsdo_get_pz_e50_ok(void);
uint8_t app_gpsdo_get_pz_e50_valid(void);
const char *app_gpsdo_get_disc_reason(void);
void app_gpsdo_print_phase_fine_status(void);
void app_gpsdo_get_phase_fine_status(app_phase_fine_status_t *out);
void app_gpsdo_get_rtc_phase_step_status(app_rtc_phase_step_status_t *out);

#endif /* APP_GPSDO_H */
