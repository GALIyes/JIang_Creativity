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
/* One fixed-format ASCII row: current actuator setting + frequency error. */
void app_gpsdo10_print_result(void);
/* GPS / PPS availability: says whether the module is silent, unfixed, or the
 * PPS wiring is what stops the discipline loop from ever starting. */
void app_gpsdo10_print_gps_stat(void);
/*
 * Report-only simulator (console command SIM).  Substitutes a caller-supplied
 * SW_E50 count into the PRINTED rows (GPSDO_RESULT and GPSDO_MEAS50) so the
 * whole reporting path can be verified without a GPS fix.  It never reaches the
 * control loop and never moves the DAC; every affected row carries SIM=1.
 * Zero disables and restores the real values.
 */
void app_gpsdo10_set_sim(int32_t sw_e50_counts);
uint8_t app_gpsdo10_get_sim(int32_t *out);
/*
 * Console command EFC?.  Prints the VCTCXO EFC window and how much correction
 * travel remains in each direction, in Hz and in SW_E50 counts, so it can be
 * compared directly against the measured SW_E50.  Answers "can the loop even
 * reach nominal, or is it saturating against the EFC limit?".
 */
void app_gpsdo10_print_efc_budget(void);
void app_gpsdo10_print_phase_stat(void);
uint8_t app_gpsdo10_get_phase10(app_phase10_diag_t *out);
void app_gpsdo10_reset_phase_history(void);
uint8_t app_gpsdo10_phase10_is_valid(void);

#endif /* APP_GPSDO10_H */
