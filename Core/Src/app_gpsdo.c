#include "app_gpsdo.h"

#include "app_config.h"
#include "app_cpld_spi.h"
#include "app_dac8571.h"
#include "app_gps_rmc.h"
#include "app_gps_uart.h"
#include "app_gpsdo10.h"
#include "app_pps.h"
#include "app_uart_debug.h"

typedef enum {
    HOLD_REASON_NONE = 0,
    HOLD_REASON_GPS_INVALID,
    HOLD_REASON_FPGA_NOT_READY,
    HOLD_REASON_PPS_TIMEOUT,
    /* The receiver lost its fix while the loop was running, so the TIMEPULSE is
     * free running on the module's own crystal and is no longer a reference. */
    HOLD_REASON_GPS_FIX_LOST
} hold_reason_t;

/* VCTCXO EFC travel limits live in app_config.h: they are hardware
 * parameters and the pull budget printed by EFC? is derived from them. */
#define DAC_CODE_MIN_SAFE APP_DAC_EFC_CODE_MIN
#define DAC_CODE_MAX_SAFE APP_DAC_EFC_CODE_MAX
#define GPSDO_SEQ10_MAX_AGE_MS 15000UL

static app_discipline_state_t g_state = APP_DISC_STATE_WARM;
static hold_reason_t g_last_hold_reason = HOLD_REASON_NONE;
static uint8_t g_boot_banner_printed;
static uint8_t g_warm_first_seq10_seen;
static uint8_t g_warm_first_seq10;
static uint8_t g_last_seq10;
static uint8_t g_last_seq10_valid;
/*
 * PPS counter at the last accepted window, taken from the STM32's own PA1
 * capture.  Consecutive CPLD windows must be exactly 10 one-second pulses
 * apart -- the strongest available proof that the window really spans ten GPS
 * seconds and that nothing (a missed edge, a glitch, an extra latch) was
 * swallowed in between.  The interval-streak check above only proves each gap
 * was 900..1100 ms; it cannot tell 9 pulses from 11.
 *
 * Borrowed from the supervisor's reference implementation, which reaches
 * 0.03 Hz using this same gate (see 学长版本对照.md).
 */
static uint32_t g_last_window_pps;
static uint8_t g_last_window_pps_valid;
static uint8_t g_auto_discipline_enabled = 1U;
static uint32_t g_current_dac_uv = VCTRL_NEAR_LOCK_UV;
static uint16_t g_current_dac_code = DAC_CODE_INIT;
static uint8_t g_last_disc10_valid;
static uint8_t g_last_disc10_seq;
static uint32_t g_last_disc10_count;
static int32_t g_last_disc10_err;
static const char *g_last_disc10_state;
static uint8_t g_last_disc50_valid;
static uint32_t g_last_disc50_count;
static int32_t g_last_disc50_err;
static uint8_t g_sw_e50_valid;
static int32_t g_sw_e50_err;
static int32_t g_sw_e50_last;
static uint8_t g_sw_e50_count;
static uint8_t g_sw_e50_last_seq10;
static uint8_t g_sw_e50_last_seq10_valid;
static uint8_t g_sw_e50_new;
/*
 * Monotonic count of completed SW_E50 results, for the reporting layer.
 *
 * Deliberately NOT reset with the accumulators and deliberately not a
 * take/clear flag: a flag would be consumed by whichever caller happened to run
 * first, and a counter that resets would make "a new window landed" and "the
 * accumulator was cleared" look identical.  A change in a monotonic value can
 * only mean a new 50 s result.
 */
static uint32_t g_sw_e50_result_seq;

static uint8_t g_sw_e100_valid;
static int32_t g_sw_e100_sum;
static int32_t g_sw_e100_last;
static uint8_t g_sw_e100_count;
static uint8_t g_sw_e100_new;

static uint8_t g_sw_e200_valid;
static int32_t g_sw_e200_sum;
static int32_t g_sw_e200_last;
static uint8_t g_sw_e200_count;
static uint8_t g_sw_e200_new;

static uint8_t g_sw_e1000_valid;
static int32_t g_sw_e1000_sum;
static int32_t g_sw_e1000_last;
static uint8_t g_sw_e1000_count;
static uint8_t g_sw_e1000_new;
static uint8_t g_disc10_last_applied;
static int32_t g_disc10_last_step_code;
static const char *g_disc10_last_reason;
static uint8_t g_disc50_last_applied;
static int32_t g_disc50_last_step_code;
static const char *g_disc50_last_reason;
static uint8_t g_disc50_confirm_count;
static uint32_t g_disc50_last_ms;
static uint8_t g_sig_retry_ok_printed;
static uint32_t g_last_seq10_change_ms;
static uint32_t g_gps_invalid_detail_last_ms;
static uint8_t g_gps_invalid_detail_last_ok;
static uint32_t g_warm_wait_log_last_ms;
static int32_t g_phase_fine_last_step_uv;
static uint32_t g_phase_fine_last_ms;
static const char *g_phase_fine_last_reason;
static uint8_t g_phase_fine_last_applied;
static int32_t g_phase_fine_last_adjust_step_uv;
static uint32_t g_phase_fine_last_adjust_ms;
static int8_t g_phase_fine_confirm_dir;
static uint8_t g_phase_fine_confirm_count;
static uint8_t g_e10_lock_cnt;
static int32_t g_e10_lock_recent[3];
static uint8_t g_e10_lock_recent_idx;
static uint32_t g_track10_enter_ms;
static int8_t g_track10_signflip_last_sign;
static uint8_t g_track10_signflip_valid;
static uint8_t g_track10_signflip_hold_count;
static uint8_t g_wait_first_e50_after_warm;
static uint8_t g_normal_e10_out_cnt;
static int8_t g_normal_e10_out_sign;
static uint32_t g_last_disc_adjust_ms;
static int32_t g_last_disc_step_code;
static uint8_t g_disc_adjust_cooldown_windows;
static uint8_t g_phase_fine_zone;
static uint8_t g_pz_state_ok;
static uint8_t g_pz_gps_ok;
static uint8_t g_pz_e10_ok;
static uint8_t g_pz_e50_ok;
static uint8_t g_pz_e50_valid;
static const char *g_last_disc_reason = "INIT";
static int32_t g_rtc_phase_step_target_ns;
static int32_t g_rtc_phase_step_raw_ns;
static int32_t g_rtc_phase_step_corr_ns;
static int32_t g_rtc_phase_step_last_cnt;
static int32_t g_rtc_phase_step_last_ns;
static const char *g_rtc_phase_step_last_reason;
static uint8_t g_rtc_phase_step_last_applied;
static int8_t g_rtc_phase_step_confirm_dir;
static uint8_t g_rtc_phase_step_confirm_count;
static uint32_t g_rtc_phase_step_last_seq;
static int32_t g_rtc_phase_step_last_ticks;
static uint32_t g_rtc_phase_step_last_status;
static uint8_t g_rtc_phase_step_last_valid;

static void app_phase_fine_mark(uint32_t ms, uint8_t applied, int32_t step_uv, const char *reason)
{
    g_phase_fine_last_ms = ms;
    g_phase_fine_last_applied = applied;
    g_phase_fine_last_step_uv = step_uv;
    g_phase_fine_last_reason = reason;
    if (applied != 0U) {
        g_phase_fine_last_adjust_ms = ms;
        g_phase_fine_last_adjust_step_uv = step_uv;
    }
}

static void app_phase_fine_confirm_reset(void)
{
    g_phase_fine_confirm_dir = 0;
    g_phase_fine_confirm_count = 0U;
}

static void app_rtc_phase_step_confirm_reset(void)
{
    g_rtc_phase_step_confirm_dir = 0;
    g_rtc_phase_step_confirm_count = 0U;
}

static void app_rtc_phase_step_mark(uint8_t applied, int32_t step_cnt, int32_t step_ns, const char *reason)
{
    g_rtc_phase_step_last_applied = applied;
    g_rtc_phase_step_last_cnt = step_cnt;
    g_rtc_phase_step_last_ns = step_ns;
    g_rtc_phase_step_last_reason = (reason != (const char *)0) ? reason : "?";
}

static void app_disc10_mark(uint8_t applied, int32_t step_code, const char *reason)
{
    g_disc10_last_applied = applied;
    g_disc10_last_step_code = step_code;
    g_disc10_last_reason = (reason != (const char *)0) ? reason : "?";
}

/* Only the count survives, because the GPSDO_AVG log still reports it.
 * The direction field and the DISC10 confirmation mechanism were never wired
 * into a decision, so their state was write only and has been removed. */
static void app_disc50_confirm_reset(void)
{
    g_disc50_confirm_count = 0U;
}

static void app_disc50_mark(uint8_t applied, int32_t step_code, const char *reason)
{
    g_disc50_last_applied = applied;
    g_disc50_last_step_code = step_code;
    g_disc50_last_reason = (reason != (const char *)0) ? reason : "?";
    if (applied != 0U) {
        g_disc50_last_ms = HAL_GetTick();
    }
}

static void app_disc_adjust_mark(void)
{
    g_last_disc_adjust_ms = HAL_GetTick();
    g_disc_adjust_cooldown_windows = GPSDO_DISC10_COOLDOWN_WINDOWS;
}

static void app_log_disc50_event(const char *type, int32_t e50, int32_t step_code,
                                 uint16_t from_code, uint16_t to_code, const char *reason)
{
    if (app_log_event_enabled() == 0U) {
        return;
    }

    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=%s,E50=%ld,STEP_CODE=%ld,FROM_CODE=%lu,TO_CODE=%lu,REASON=%s\r\n",
                   (unsigned long)HAL_GetTick(),
                   (type != (const char *)0) ? type : "?",
                   (long)e50,
                   (long)step_code,
                   (unsigned long)from_code,
                   (unsigned long)to_code,
                   (reason != (const char *)0) ? reason : "?");
}

static void app_disc_reason_set(const char *reason)
{
    g_last_disc_reason = (reason != (const char *)0) ? reason : "?";
}

static int32_t app_abs_i32(int32_t v);

static const char *app_state_str(void)
{
    switch (g_state) {
        case APP_DISC_STATE_WARM: return "WARM";
        case APP_DISC_STATE_TRACK10: return "TRACK10";
        case APP_DISC_STATE_NORMAL: return "NORMAL50";
        case APP_DISC_STATE_HOLDOVER: return "HOLDOVER";
        case APP_DISC_STATE_MANUAL: return "MANUAL";
        default: return "?";
    }
}

static const char *app_hold_reason_str(hold_reason_t reason)
{
    switch (reason) {
        case HOLD_REASON_GPS_INVALID: return "GPS_INVALID";
        case HOLD_REASON_FPGA_NOT_READY: return "FPGA_NOT_READY";
        case HOLD_REASON_PPS_TIMEOUT: return "PPS_TIMEOUT";
        case HOLD_REASON_GPS_FIX_LOST: return "GPS_FIX_LOST";
        default: return "UNKNOWN";
    }
}

static void app_print_uv(uint32_t uv)
{
    app_log_printf("%lu.%04luV",
                   (unsigned long)(uv / 1000000UL),
                   (unsigned long)((uv % 1000000UL) / 100UL));
}

static uint32_t app_code_to_uv(uint16_t code)
{
    uint64_t num = ((uint64_t)code * (uint64_t)DAC_VREF_UV) + 32767ULL;
    return (uint32_t)(num / 65535ULL);
}

static void app_log_event_line(const char *type, const char *reason)
{
    if (app_log_event_enabled() == 0U) {
        return;
    }

    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=%s,REASON=%s\r\n",
                   (unsigned long)HAL_GetTick(),
                   (type != (const char *)0) ? type : "?",
                   (reason != (const char *)0) ? reason : "?");
}

static void app_print_step_uv(int32_t step_uv)
{
    if (step_uv >= 0) {
        app_log_write("+");
        app_print_uv((uint32_t)step_uv);
    } else {
        app_log_write("-");
        app_print_uv((uint32_t)(-step_uv));
    }
}

static uint8_t app_abs_err_in_range(int32_t err, int32_t limit)
{
    int64_t a = (int64_t)err;
    if (a < 0) {
        a = -a;
    }
    return (a <= (int64_t)limit) ? 1U : 0U;
}

static uint8_t app_gps_ok(void)
{
    const app_rtc_state_t *r = app_rtc_get_state();
    /* GPS valid = RMC valid (NMEA-driven). rtc_synced is NOT required because
       this STM32 version has no PPS and rtc_synced can transiently expire
       between RMC arrivals even when GPS is healthy. */
    return ((r->rtc_valid != 0U) && (r->rmc_valid != 0U)) ? 1U : 0U;
}

static void app_log_gps_invalid_detail(uint32_t ms, uint8_t gps_ok)
{
    const app_rtc_state_t *rtc = app_rtc_get_state();
    app_pps_snapshot_t pps;
    app_gps_uart_stats_t gps_stats;
    uint32_t pps_age_ms;
    uint32_t rmc_age_s;
    uint8_t state_changed;

#if GPSDO_LOG_GPS_INVALID_DETAIL_ENABLE == 0U
    (void)ms;
    (void)gps_ok;
    return;
#endif

    state_changed = (gps_ok != g_gps_invalid_detail_last_ok) ? 1U : 0U;
    g_gps_invalid_detail_last_ok = gps_ok;

    if (state_changed == 0U) {
        if ((ms - g_gps_invalid_detail_last_ms) < GPSDO_GPS_INVALID_DETAIL_PERIOD_MS) {
            return;
        }
    }
    g_gps_invalid_detail_last_ms = ms;

    app_pps_get_snapshot(&pps);
    app_gps_uart_get_stats(&gps_stats);
    /* Handle case where no PPS has been received */
    if (pps.last_tick_ms == 0xFFFFFFFFUL) {
        pps_age_ms = 999999UL;  /* Indicate no PPS received */
    } else {
        pps_age_ms = ms - pps.last_tick_ms;
    }
    rmc_age_s = (ms - gps_stats.last_rmc_tick_ms) / 1000UL;

    app_log_printf("GPS_INVALID_DETAIL,MS=%lu,gps_ok=%lu,pps_age_ms=%lu,rmc_valid=%lu,rmc_age_s=%lu,rtc_synced=%lu\r\n",
                   (unsigned long)ms,
                   (unsigned long)gps_ok,
                   (unsigned long)pps_age_ms,
                   (unsigned long)rtc->rmc_valid,
                   (unsigned long)rmc_age_s,
                   (unsigned long)rtc->rtc_synced);
}

static uint8_t app_fpga_ready(uint32_t status, uint32_t sig)
{
    return (((status & CPLD_STATUS_ALIVE_MASK) != 0UL) && (sig == CPLD_SIGNATURE_VALUE)) ? 1U : 0U;
}

static void app_log_fpga_not_ready_detail(const app_cpld_signature_stable_t *sig_detail,
                                          uint32_t status, HAL_StatusTypeDef st_status,
                                          uint32_t window_seq, HAL_StatusTypeDef st_seq)
{
    uint8_t alive = ((status & CPLD_STATUS_ALIVE_MASK) != 0UL) ? 1U : 0U;

    app_log_printf("FPGA_NOT_READY_DETAIL: SIG1=0x%08lX SIG2=0x%08lX STATUS=0x%08lX ALIVE=%lu HAL_SIG1=%lu HAL_SIG2=%lu HAL_STATUS=%lu WINDOW_SEQ=0x%08lX HAL_SEQ=%lu SIG_RETRY=%lu SHIFTED_COUNT=%lu\r\n",
                   (unsigned long)sig_detail->sig1,
                   (unsigned long)sig_detail->sig2,
                   (unsigned long)status,
                   (unsigned long)alive,
                   (unsigned long)sig_detail->st1,
                   (unsigned long)sig_detail->st2,
                   (unsigned long)st_status,
                   (unsigned long)window_seq,
                   (unsigned long)st_seq,
                   (unsigned long)sig_detail->retry_count,
                   (unsigned long)sig_detail->shifted_count);
}

static void app_reset_sw_accumulators(void)
{
    g_sw_e50_valid = 0U;
    g_sw_e50_err = 0;
    g_sw_e50_last = 0;
    g_sw_e50_count = 0U;
    g_sw_e50_last_seq10 = 0U;
    g_sw_e50_last_seq10_valid = 0U;
    g_sw_e50_new = 0U;
    g_sw_e100_valid = 0U;
    g_sw_e100_sum = 0;
    g_sw_e100_last = 0;
    g_sw_e100_count = 0U;
    g_sw_e100_new = 0U;
    g_sw_e200_valid = 0U;
    g_sw_e200_sum = 0;
    g_sw_e200_last = 0;
    g_sw_e200_count = 0U;
    g_sw_e200_new = 0U;
    g_sw_e1000_valid = 0U;
    g_sw_e1000_sum = 0;
    g_sw_e1000_last = 0;
    g_sw_e1000_count = 0U;
    g_sw_e1000_new = 0U;
}
static void app_reset_tracking_for_warm(void)
{
    g_last_seq10_valid = 0U;
    g_last_window_pps_valid = 0U;
    g_last_seq10 = 0U;
    g_last_seq10_change_ms = HAL_GetTick();
    g_warm_first_seq10_seen = 0U;
    g_warm_first_seq10 = 0U;
    g_phase_fine_zone = 0U;
    g_e10_lock_cnt = 0U;
    g_e10_lock_recent[0] = 0;
    g_e10_lock_recent[1] = 0;
    g_e10_lock_recent[2] = 0;
    g_e10_lock_recent_idx = 0U;
    g_track10_enter_ms = 0UL;
    g_track10_signflip_last_sign = 0;
    g_track10_signflip_valid = 0U;
    g_track10_signflip_hold_count = 0U;
    g_wait_first_e50_after_warm = 1U;
    app_reset_sw_accumulators();
    g_disc10_last_applied = 0U;
    g_disc10_last_step_code = 0;
    g_disc10_last_reason = "RESET";
    app_disc50_confirm_reset();
    g_disc50_last_applied = 0U;
    g_disc50_last_step_code = 0;
    g_disc50_last_reason = "RESET";
    g_disc50_last_ms = 0UL;
    app_rtc_phase_step_confirm_reset();
    app_rtc_phase_step_mark(0U, 0, 0, "RESET");
}

static void app_clear_windows_and_log(const char *reason)
{
    app_gpsdo10_reset_phase_history();
    if (app_cpld_clear_measurement_windows() == HAL_OK) {
        app_log_printf("FPGA_CLEAR: CTRL=0x00000003, REASON=%s\r\n", reason);
    } else {
        app_log_write("FPGA_CLEAR_WARN: REASON=SPI_WRITE_FAIL\r\n");
    }
}

static void app_enter_warm(const char *reason)
{
    g_state = APP_DISC_STATE_WARM;
    g_last_hold_reason = HOLD_REASON_NONE;
    app_gpsdo10_reset_phase_history();
    app_reset_tracking_for_warm();
    app_log_printf("DISC_WARM: REASON=%s, SKIP_10S=1\r\n", reason);
    if (app_log_event_enabled() != 0U) {
        app_log_printf("GPSDO_EVT,MS=%lu,TYPE=WARM,REASON=%s,SKIP_10S=1\r\n",
                       (unsigned long)HAL_GetTick(),
                       (reason != (const char *)0) ? reason : "?");
    }
}

static void app_enter_holdover(hold_reason_t reason)
{
    app_gpsdo10_reset_phase_history();
    if ((g_state != APP_DISC_STATE_HOLDOVER) || (reason != g_last_hold_reason)) {
        app_log_printf("DISC_HOLD: REASON=%s\r\n", app_hold_reason_str(reason));
        app_log_event_line("HOLD", app_hold_reason_str(reason));
    }
    g_last_hold_reason = reason;
    g_state = APP_DISC_STATE_HOLDOVER;
    g_phase_fine_zone = 0U;
    app_rtc_phase_step_confirm_reset();
    app_rtc_phase_step_mark(0U, 0, 0, "HOLDOVER");
}

static void app_enter_fpga_not_ready_holdover(const app_cpld_signature_stable_t *sig_detail,
                                              uint32_t status, HAL_StatusTypeDef st_status,
                                              uint32_t window_seq, HAL_StatusTypeDef st_seq)
{
    if ((g_state != APP_DISC_STATE_HOLDOVER) || (g_last_hold_reason != HOLD_REASON_FPGA_NOT_READY)) {
        app_log_fpga_not_ready_detail(sig_detail, status, st_status, window_seq, st_seq);
    }
    app_enter_holdover(HOLD_REASON_FPGA_NOT_READY);
}

static void app_warm_wait_log(uint8_t fpga_ok, uint8_t gps_valid, uint8_t count10_valid,
                               uint8_t seq10_same, uint32_t seqraw, uint8_t seq10,
                               uint8_t seq50, uint8_t clr10, uint8_t clr50, uint32_t count10)
{
    uint32_t ms = HAL_GetTick();

#if GPSDO_LOG_WARM_WAIT_ENABLE == 0U
    (void)fpga_ok; (void)gps_valid; (void)count10_valid;
    (void)seq10_same; (void)seqraw; (void)seq10; (void)seq50;
    (void)clr10; (void)clr50; (void)count10;
    return;
#endif

    if ((ms - g_warm_wait_log_last_ms) < GPSDO_WARM_WAIT_LOG_PERIOD_MS) {
        return;
    }
    g_warm_wait_log_last_ms = ms;

    app_log_printf("DISC_WARM_WAIT: FPGA=%lu,GPS=%lu,COUNT10_VALID=%lu,SEQRAW=0x%08lX,SEQ10=%lu,SEQ50=%lu,CLR10=%lu,CLR50=%lu,COUNT10=%lu",
                   (unsigned long)fpga_ok,
                   (unsigned long)gps_valid,
                   (unsigned long)count10_valid,
                   (unsigned long)seqraw,
                   (unsigned long)seq10,
                   (unsigned long)seq50,
                   (unsigned long)clr10,
                   (unsigned long)clr50,
                   (unsigned long)count10);
    if (seq10_same != 0U) {
        app_log_write(",SEQ10_SAME=1");
    }
    app_log_write("\r\n");
}

static uint32_t app_clamp_uv_to_range(int64_t uv)
{
    if (uv < 0) {
        return 0UL;
    }
    if (uv > 3300000LL) {
        return 3300000UL;
    }
    return (uint32_t)uv;
}

static uint16_t app_step_code_from_err(int32_t err, uint16_t step_code)
{
    uint16_t current = g_current_dac_code;

    if ((err == 0) || (step_code == 0U)) {
        return current;
    }

#if DAC_INCREASE_MAKES_FREQ_HIGHER
    if (err > 0) {
        return (current > step_code) ? (uint16_t)(current - step_code) : 0U;
    }
    return ((uint32_t)current + (uint32_t)step_code > 65535UL) ? 65535U : (uint16_t)(current + step_code);
#else
    if (err > 0) {
        return ((uint32_t)current + (uint32_t)step_code > 65535UL) ? 65535U : (uint16_t)(current + step_code);
    }
    return (current > step_code) ? (uint16_t)(current - step_code) : 0U;
#endif
}

static uint16_t app_clamp_dac_code_safe(uint16_t code)
{
    if (code < DAC_CODE_MIN_SAFE) {
        return DAC_CODE_MIN_SAFE;
    }
    if (code > DAC_CODE_MAX_SAFE) {
        return DAC_CODE_MAX_SAFE;
    }
    return code;
}

static uint16_t app_step_code_from_err_safe(int32_t err, uint16_t step_code, uint8_t *clamped)
{
    uint16_t raw;
    uint16_t clamped_code;

    raw = app_step_code_from_err(err, step_code);
    clamped_code = app_clamp_dac_code_safe(raw);
    if (clamped != (uint8_t *)0) {
        *clamped = (clamped_code != raw) ? 1U : 0U;
    }
    return clamped_code;
}

static int32_t app_step_uv_to_code(uint16_t target_code)
{
    uint32_t from_uv = app_log_dac_code_to_uv(g_current_dac_code);
    uint32_t to_uv = app_log_dac_code_to_uv(target_code);

    return (int32_t)((int64_t)to_uv - (int64_t)from_uv);
}

static int32_t app_clamp_track10_slope_num(void)
{
    int32_t slope = (int32_t)GPSDO_DAC_SLOPE_DEFAULT_COUNT_PER_100MV;
    if (slope < (int32_t)GPSDO_DAC_SLOPE_MIN_COUNT_PER_100MV) {
        slope = (int32_t)GPSDO_DAC_SLOPE_MIN_COUNT_PER_100MV;
    } else if (slope > (int32_t)GPSDO_DAC_SLOPE_MAX_COUNT_PER_100MV) {
        slope = (int32_t)GPSDO_DAC_SLOPE_MAX_COUNT_PER_100MV;
    }
    return slope;
}

static uint16_t app_track10_code_step(int32_t abs_err10, const char **reason)
{
    uint16_t step_code;

    if (abs_err10 <= GPSDO_TRACK10_DEADBAND_COUNT) {
        if (reason != (const char **)0) {
            *reason = "TRACK10_LOCK_E10_LE1";
        }
        return 0U;
    }
    if (abs_err10 <= 3L) {
        if (reason != (const char **)0) {
            *reason = "TRACK10_STEP_CODE8";
        }
        return GPSDO_TRACK10_STEP_E2_3;
    }
    if (abs_err10 <= 7L) {
        if (reason != (const char **)0) {
            *reason = "TRACK10_STEP_CODE16";
        }
        return GPSDO_TRACK10_STEP_E4_7;
    }
    if (abs_err10 <= 15L) {
        if (reason != (const char **)0) {
            *reason = "TRACK10_STEP_CODE32";
        }
        return GPSDO_TRACK10_STEP_E8_15;
    }
    if (abs_err10 <= 31L) {
        if (reason != (const char **)0) {
            *reason = "TRACK10_STEP_CODE64";
        }
        return GPSDO_TRACK10_STEP_E16_31;
    }
    if (abs_err10 <= 63L) {
        if (reason != (const char **)0) {
            *reason = "TRACK10_STEP_CODE128";
        }
        return GPSDO_TRACK10_STEP_E32_63;
    }
    if (reason != (const char **)0) {
        *reason = "TRACK10_STEP_CODE256";
    }
    step_code = GPSDO_TRACK10_STEP_E64_UP;
    if (step_code > GPSDO_TRACK10_MAX_STEP_CODE) {
        step_code = GPSDO_TRACK10_MAX_STEP_CODE;
    }
    return step_code;
}

static uint8_t app_str_equal(const char *a, const char *b)
{
    if ((a == (const char *)0) || (b == (const char *)0)) {
        return 0U;
    }
    while ((*a != '\0') && (*b != '\0')) {
        if (*a != *b) {
            return 0U;
        }
        a++;
        b++;
    }
    return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

static const char *app_track10_clamp_reason(const char *reason, uint8_t clamped)
{
    if (clamped == 0U) {
        return reason;
    }
    if (reason == (const char *)0) {
        return "TRACK10_CLAMP";
    }
    if (app_str_equal(reason, "TRACK10_CONFIRM2_CODE12") != 0U) {
        return "TRACK10_CONFIRM2_CODE12_CLAMP";
    }
    if (app_str_equal(reason, "TRACK10_FINE_CODE24") != 0U) {
        return "TRACK10_FINE_CODE24_CLAMP";
    }
    if (app_str_equal(reason, "TRACK10_COARSE_CODE64") != 0U) {
        return "TRACK10_COARSE_CODE64_CLAMP";
    }
    if (app_str_equal(reason, "TRACK10_FAST_CODE128") != 0U) {
        return "TRACK10_FAST_CODE128_CLAMP";
    }
    if (app_str_equal(reason, "TRACK10_FAST_CODE256") != 0U) {
        return "TRACK10_FAST_CODE256_CLAMP";
    }
    if (app_str_equal(reason, "TRACK10_FAST_CODE512") != 0U) {
        return "TRACK10_FAST_CODE512_CLAMP";
    }
    return "TRACK10_CLAMP";
}

static void app_print_disc_line(const char *source, uint32_t count_value,
                                const char *count_label, const char *err_label,
                                int32_t err_value, int32_t step_uv,
                                uint32_t from_uv, uint32_t to_uv, uint8_t applied,
                                const char *reason, const char *method,
                                uint8_t target_valid, uint32_t target_uv,
                                uint8_t seq10_valid, uint8_t seq10,
                                uint32_t slope_num, uint32_t slope_den)
{
    if (app_log_legacy_gpsdo10_enabled() == 0U) {
        return;
    }

    app_log_printf("%s: ", source);
    if (seq10_valid != 0U) {
        app_log_printf("SEQ10=%lu,", (unsigned long)seq10);
    }
    app_log_printf("%s=%lu,%s=%ld", count_label, (unsigned long)count_value, err_label, (long)err_value);
    if (target_valid != 0U) {
        app_log_write(",TARGET=");
        app_print_uv(target_uv);
    }
    app_log_write(",STEP=");
    app_print_step_uv(step_uv);
    app_log_write(",FROM=");
    app_print_uv(from_uv);
    app_log_write(",TO=");
    app_print_uv(to_uv);
    app_log_printf(",APPLIED=%lu,STATE=%s,SLOPE=%lu/%lu",
                   (unsigned long)applied,
                   app_state_str(),
                   (unsigned long)slope_num,
                   (unsigned long)slope_den);
    if (reason != (const char *)0) {
        app_log_printf(",REASON=%s", reason);
    }
    if (method != (const char *)0) {
        app_log_printf(",METHOD=%s", method);
    }
    app_log_write("\r\n");
}

static void app_print_train_line(const char *source, uint32_t count, const char *err_label, int32_t err)
{
    const app_rtc_state_t *rtc = app_rtc_get_state();

    if (app_log_legacy_gpsdo10_enabled() == 0U) {
        return;
    }

    app_log_printf("%s: DAC=", source);
    app_print_uv(g_current_dac_uv);
    app_log_printf(",CODE=%lu,COUNT%s=%lu,%s=%ld,RTC_SYNC=%lu,RMC_VALID=%lu\r\n",
                   (unsigned long)g_current_dac_code,
                   (source[5] == '0') ? "10" : "50",
                   (unsigned long)count,
                   err_label,
                   (long)err,
                   (unsigned long)rtc->rtc_synced,
                   (unsigned long)rtc->rmc_valid);
}

static uint8_t app_apply_code_and_log(const char *source, uint32_t count_value,
                                      const char *count_label, const char *err_label,
                                      int32_t err_value, int32_t step_uv, uint8_t use_apply,
                                      const char *reason, const char *method,
                                      uint8_t target_valid, uint32_t target_uv,
                                      uint8_t seq10_valid, uint8_t seq10,
                                      uint32_t slope_num, uint32_t slope_den,
                                      uint8_t target_code_valid, uint16_t target_code,
                                      uint8_t clear_after_adjust);

static void app_log_disc_dac_adjust_event(const char *source, uint16_t from_code, uint16_t to_code,
                                          uint32_t from_uv, uint32_t to_uv, const char *reason)
{
    if ((app_log_event_enabled() == 0U) || (app_log_dac_adjust_enabled() == 0U)) {
        return;
    }
    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=DAC_ADJUST,SOURCE=%s,FROM_CODE=%lu,TO_CODE=%lu,FROM_V=",
                   (unsigned long)HAL_GetTick(),
                   (source != (const char *)0) ? source : "?",
                   (unsigned long)from_code,
                   (unsigned long)to_code);
    app_log_print_voltage_uv6(from_uv);
    app_log_write(",TO_V=");
    app_log_print_voltage_uv6(to_uv);
    app_log_printf(",STEP_UV=%ld,REASON=%s,E10=%ld,E50=%ld,STEP_CODE=%ld\r\n",
                   (long)((int32_t)((int64_t)to_uv - (int64_t)from_uv)),
                   (reason != (const char *)0) ? reason : "?",
                   (long)g_last_disc10_err,
                   (long)((g_last_disc50_valid != 0U) ? g_last_disc50_err : 0L),
                   (long)((int32_t)to_code - (int32_t)from_code));
}

static uint8_t app_phase_fine_zone_calc(uint32_t ms, int32_t err10, uint8_t e10_valid,
                                        uint32_t count50, int32_t err50, uint8_t e50_valid)
{
    const app_rtc_state_t *rtc = app_rtc_get_state();
    (void)count50;
    (void)err50;
    (void)e50_valid;

    g_pz_state_ok = (g_state == APP_DISC_STATE_NORMAL) ? 1U : 0U;
    (void)ms;
    g_pz_gps_ok = ((app_gps_ok() != 0U) &&
                   (rtc->rmc_valid != 0U)) ? 1U : 0U;
    g_pz_e10_ok = ((e10_valid != 0U) && (app_abs_i32(err10) <= 1L)) ? 1U : 0U;
    g_pz_e50_valid = 0U;
    g_pz_e50_ok = 0U;

    g_phase_fine_zone = ((g_pz_state_ok != 0U) &&
                         (g_pz_gps_ok != 0U) &&
                         (g_pz_e10_ok != 0U)) ? 1U : 0U;
    return g_phase_fine_zone;
}

static uint8_t app_phase_fine_zone_update(uint32_t ms)
{
    return app_phase_fine_zone_calc(ms,
                                    g_last_disc10_err,
                                    g_last_disc10_valid,
                                    g_last_disc50_count,
                                    g_last_disc50_err,
                                    g_last_disc50_valid);
}

static uint8_t app_apply_code_and_log(const char *source, uint32_t count_value,
                                      const char *count_label, const char *err_label,
                                      int32_t err_value, int32_t step_uv, uint8_t use_apply,
                                      const char *reason, const char *method,
                                      uint8_t target_valid, uint32_t target_uv,
                                      uint8_t seq10_valid, uint8_t seq10,
                                      uint32_t slope_num, uint32_t slope_den,
                                      uint8_t target_code_valid, uint16_t target_code,
                                      uint8_t clear_after_adjust)
{
    uint16_t old_code = g_current_dac_code;
    uint32_t from_uv = (target_code_valid != 0U) ?
                       app_log_dac_code_to_uv(old_code) :
                       g_current_dac_uv;
    uint32_t planned_to_uv = (target_code_valid != 0U) ?
                             app_log_dac_code_to_uv(target_code) :
                             app_clamp_uv_to_range((int64_t)from_uv + (int64_t)step_uv);
    uint8_t applied = 0U;
    HAL_StatusTypeDef dac_status = HAL_OK;

    if ((step_uv != 0) &&
        (use_apply != 0U) &&
        (g_auto_discipline_enabled != 0U) &&
        ((g_state == APP_DISC_STATE_TRACK10) || (g_state == APP_DISC_STATE_NORMAL))) {
#if DAC_DISCIPLINE_APPLY_ENABLE
        if ((GPSDO_DRY_RUN == 0U) && (DAC_AUTO_WRITE != 0U)) {
            if (target_code_valid != 0U) {
                dac_status = app_dac8571_write_code(target_code);
            } else {
                dac_status = app_dac8571_set_voltage_uv(planned_to_uv);
            }
            if (dac_status == HAL_OK) {
                g_current_dac_code = app_dac8571_get_last_code();
                g_current_dac_uv = app_log_dac_code_to_uv(g_current_dac_code);
                applied = (old_code != g_current_dac_code) ? 1U : 0U;
            }
        }
#endif
    }

    app_print_disc_line(source, count_value, count_label, err_label, err_value, step_uv,
                        from_uv, planned_to_uv, applied, reason, method, target_valid,
                        target_uv, seq10_valid, seq10, slope_num, slope_den);

    if (applied != 0U) {
        app_log_write("DAC_ADJUST: FROM=");
        app_log_print_voltage_uv6(from_uv);
        app_log_write(", TO=");
        app_log_print_voltage_uv6(g_current_dac_uv);
        app_log_printf(", CODE=%lu, SOURCE=%s\r\n", (unsigned long)g_current_dac_code, source);
        app_log_disc_dac_adjust_event(source, old_code, g_current_dac_code, from_uv, g_current_dac_uv,
                                      (reason != (const char *)0) ? reason : source);
        g_last_disc_step_code = (int32_t)g_current_dac_code - (int32_t)old_code;
        app_disc_adjust_mark();
        if (clear_after_adjust != 0U) {
            app_clear_windows_and_log("DAC_ADJUST");
            app_enter_warm("DAC_ADJUST");
        }
    }
    if ((step_uv != 0) && (applied == 0U) && (use_apply != 0U) &&
        (g_auto_discipline_enabled != 0U) &&
        ((g_state == APP_DISC_STATE_TRACK10) || (g_state == APP_DISC_STATE_NORMAL)) &&
        (GPSDO_DRY_RUN == 0U) && (DAC_AUTO_WRITE != 0U)) {
        app_log_dac_adjust_fail_event(source, app_dac8571_voltage_to_code(from_uv),
                                      app_dac8571_voltage_to_code(planned_to_uv),
                                      (reason != (const char *)0) ? reason : source,
                                      dac_status);
    }

    g_last_disc_step_code = (int32_t)g_current_dac_code - (int32_t)old_code;
    return applied;
}

static uint8_t app_disc50_process_window(uint8_t seq_ok, uint8_t seq10,
                                         uint8_t ignore_interval, uint8_t stop_after_skip)
{
    int32_t sw_e50 = g_sw_e50_last;
    int32_t abs_e50 = app_abs_i32(sw_e50);
    int32_t disc50_step_code = 0;
    int64_t disc50_step64;
    uint16_t disc50_from_code = g_current_dac_code;
    uint16_t disc50_target_code = g_current_dac_code;
    uint8_t disc50_code_valid = 0U;
    uint8_t disc50_applied = 0U;
    uint8_t disc50_clamped = 0U;
    const char *disc50_reason = "DISC50_IDLE";

    g_last_disc50_valid = 1U;
    g_last_disc50_count = GPSDO_SW_E50_WINDOWS;
    g_last_disc50_err = sw_e50;

    if ((ignore_interval == 0U) && ((HAL_GetTick() - g_disc50_last_ms) < GPSDO_DISC50_MIN_INTERVAL_MS)) {
        return 1U;
    }

    if (abs_e50 <= (int32_t)GPSDO_DISC50_DEADBAND_COUNT) {
        disc50_reason = "DISC50_E50_ZERO";
    } else {
        /* Proportional step.  The previous fixed table {1,2,4,8}, saturated at
         * 8 LSB, which limited the slew rate to ~0.82 mHz/s and made large
         * errors take tens of minutes to pull in.  Scaling the step with the
         * error keeps the same small-signal gain while removing the ceiling. */
        disc50_step64 = ((int64_t)abs_e50 * (int64_t)GPSDO_DISC50_STEP_LSB_PER_COUNT);

        if (disc50_step64 < (int64_t)GPSDO_DISC50_STEP_LSB_MIN) {
            disc50_step64 = (int64_t)GPSDO_DISC50_STEP_LSB_MIN;
        } else if (disc50_step64 > (int64_t)GPSDO_DISC50_MAX_STEP_CODE) {
            disc50_step64 = (int64_t)GPSDO_DISC50_MAX_STEP_CODE;
        }
        disc50_step_code = (int32_t)disc50_step64;

        if (abs_e50 <= 1L) {
            disc50_reason = "DISC50_STEP_TRIM";
        } else if (abs_e50 <= 10L) {
            disc50_reason = "DISC50_STEP_FINE";
        } else if (abs_e50 <= 100L) {
            disc50_reason = "DISC50_STEP_COARSE";
        } else {
            disc50_reason = "DISC50_STEP_SLEW";
        }

        disc50_target_code = app_step_code_from_err_safe(sw_e50, (uint16_t)disc50_step_code, &disc50_clamped);
        disc50_code_valid = 1U;

        /*
         * If the proportional step wanted to go past the VCTCXO's EFC window,
         * the target is pinned to the limit.  Report that explicitly instead of
         * keeping the step-size reason: a saturated loop is indistinguishable
         * from "the loop is broken" once it stops moving, and unlike a gain
         * problem it can NEVER be fixed by tuning the control law -- the EFC
         * window (or the park point) has to change.  EFC? prints how much
         * travel is left, in the same SW_E50 units as this error.
         */
        if (disc50_clamped != 0U) {
            disc50_reason = (sw_e50 < 0) ? "DISC50_EFC_SAT_UP" : "DISC50_EFC_SAT_DOWN";
        }

        disc50_applied = app_apply_code_and_log("DISC50", (uint32_t)GPSDO_SW_E50_WINDOWS, "SW_E50_WIN", "SW_E50",
                                                sw_e50, app_step_uv_to_code(disc50_target_code),
                                                DAC_DISCIPLINE_USE_50S_APPLY, disc50_reason,
                                                "SW_E50", 0U, 0UL, seq_ok, seq10,
                                                (uint32_t)DAC_SLOPE_NUM, (uint32_t)DAC_SLOPE_DEN,
                                                disc50_code_valid, disc50_target_code, 0U);
        if (disc50_applied != 0U) {
            app_disc50_mark(1U, (int32_t)g_last_disc_step_code, disc50_reason);
            app_log_disc50_event("DISC50_APPLY", sw_e50, (int32_t)g_last_disc_step_code,
                                 disc50_from_code, g_current_dac_code, disc50_reason);
            app_disc_reason_set(disc50_reason);
            return 1U;
        }
        /* Once pinned at the limit the target equals the current code, so the
         * write is a no-op -- keep the saturation reason rather than masking it
         * with NOCHANGE, otherwise a saturated loop goes silent. */
        if (disc50_clamped == 0U) {
            disc50_reason = "DISC50_APPLY_NOCHANGE";
        }
    }

    app_disc50_mark(0U, 0, disc50_reason);
    app_log_disc50_event("DISC50_SKIP", sw_e50, 0, disc50_from_code, disc50_from_code, disc50_reason);
    app_disc_reason_set(disc50_reason);
    return stop_after_skip;
}

static void app_boot_clear_windows_if_ready(uint8_t ready)
{
    uint32_t ctrl_status = 0UL;
    uint32_t window_seq = 0UL;
    HAL_StatusTypeDef st_clear;
    HAL_StatusTypeDef st_ctrl;
    HAL_StatusTypeDef st_seq;

    if (ready == 0U) {
        return;
    }

    st_clear = app_cpld_clear_measurement_windows();
    st_ctrl = app_cpld_read_reg32(CPLD_REG_CTRL_STATUS, &ctrl_status);
    st_seq = app_cpld_read_reg32(CPLD_REG_WINDOW_SEQ, &window_seq);

    if (st_clear == HAL_OK) {
        app_gpsdo10_reset_phase_history();
        g_state = APP_DISC_STATE_WARM;
        g_last_hold_reason = HOLD_REASON_NONE;
        app_reset_tracking_for_warm();
        app_log_printf("FPGA_BOOT_CLEAR: OK CTRL_STATUS=0x%08lX WINDOW_SEQ=0x%08lX\r\n",
                       (unsigned long)ctrl_status,
                       (unsigned long)window_seq);
    } else {
        app_log_printf("FPGA_BOOT_CLEAR: FAIL ST_CLEAR=%lu ST_CTRL=%lu CTRL_STATUS=0x%08lX ST_SEQ=%lu WINDOW_SEQ=0x%08lX\r\n",
                       (unsigned long)st_clear,
                       (unsigned long)st_ctrl,
                       (unsigned long)ctrl_status,
                       (unsigned long)st_seq,
                       (unsigned long)window_seq);
    }
}

static void app_gpsdo_print_fpga_init(uint32_t status, uint32_t sig)
{
    uint8_t alive;
    uint8_t ready;

    if (g_boot_banner_printed != 0U) {
        return;
    }
    g_boot_banner_printed = 1U;
    alive = ((status & CPLD_STATUS_ALIVE_MASK) != 0UL) ? 1U : 0U;
    ready = app_fpga_ready(status, sig);
    app_log_printf("FPGA_INIT: SIG=0x%08lX, ALIVE=%lu, READY=%lu\r\n",
                   (unsigned long)sig,
                   (unsigned long)alive,
                   (unsigned long)ready);
    app_boot_clear_windows_if_ready(ready);
    app_log_write("DISC_WARM: REASON=BOOT, SKIP_10S=1\r\n");
}

void app_gpsdo_init(void)
{
    g_state = APP_DISC_STATE_WARM;
    g_last_hold_reason = HOLD_REASON_NONE;
    g_boot_banner_printed = 0U;
    g_auto_discipline_enabled = 1U;
    g_current_dac_uv = VCTRL_NEAR_LOCK_UV;
    g_current_dac_code = DAC_CODE_INIT;
    g_phase_fine_last_step_uv = 0;
    g_phase_fine_last_ms = 0UL;
    g_phase_fine_last_reason = "INIT";
    g_phase_fine_last_applied = 0U;
    g_phase_fine_last_adjust_step_uv = 0;
    g_phase_fine_last_adjust_ms = 0UL;
    app_phase_fine_confirm_reset();
    g_rtc_phase_step_target_ns = 0;
    g_rtc_phase_step_raw_ns = 0;
    g_rtc_phase_step_corr_ns = 0;
    g_rtc_phase_step_last_cnt = 0;
    g_rtc_phase_step_last_ns = 0;
    g_rtc_phase_step_last_reason = "INIT";
    g_rtc_phase_step_last_applied = 0U;
    g_rtc_phase_step_last_seq = 0UL;
    g_rtc_phase_step_last_ticks = 0;
    g_rtc_phase_step_last_status = 0UL;
    g_rtc_phase_step_last_valid = 0U;
    app_rtc_phase_step_confirm_reset();
    g_last_disc10_valid = 0U;
    g_last_disc10_seq = 0U;
    g_last_disc10_state = app_state_str();
    g_disc10_last_applied = 0U;
    g_disc10_last_step_code = 0;
    g_disc10_last_reason = "INIT";
    g_last_disc50_valid = 0U;
    g_disc50_last_applied = 0U;
    g_disc50_last_step_code = 0;
    g_disc50_last_reason = "INIT";
    g_disc50_last_ms = 0UL;
    app_disc50_confirm_reset();
    g_sig_retry_ok_printed = 0U;
    g_gps_invalid_detail_last_ms = 0U;
    g_gps_invalid_detail_last_ok = 0xFFU;
    g_warm_wait_log_last_ms = 0U;
    app_reset_tracking_for_warm();

    app_log_printf("GPSDO TARGET: C10=%lu C50=%lu OFF=%ld\r\n",
                   (unsigned long)GPSDO_TARGET_COUNT10,
                   (unsigned long)GPSDO_TARGET_COUNT50,
                   (long)GPSDO_TARGET_COUNT10_OFFSET);
    app_log_write("ERR_SIGN=COUNT_MINUS_TARGET\r\n");
    app_log_printf("GPSDO_DRY_RUN=%lu TARGET10=%lu OFFSET=%ld DAC_AUTO_WRITE=%lu\r\n",
                   (unsigned long)GPSDO_DRY_RUN,
                   (unsigned long)GPSDO_TARGET_COUNT10,
                   (long)GPSDO_TARGET_COUNT10_OFFSET,
                   (unsigned long)DAC_AUTO_WRITE);
    app_log_printf("PHASE_FINE_DAC=%s RTC_PHASE_STEP=%s PH_TARGET_NS=%ld\r\n",
                    (GPSDO_RTC_PHASE_USE_DAC_FINE != 0U) ? "ENABLED" : "DISABLED",
                    (GPSDO_RTC_PHASE_STEP_ENABLE != 0U) ? "ENABLED" : "DISABLED_FOR_PHASE_LOG_ONLY",
                    (long)g_rtc_phase_step_target_ns);
}

void app_gpsdo_on_manual_dac_set(uint16_t code, uint32_t uv, uint8_t restart_auto)
{
    g_current_dac_code = code;
    g_current_dac_uv = uv;

    app_reset_tracking_for_warm();
    if (restart_auto != 0U) {
        g_auto_discipline_enabled = 1U;
        g_last_hold_reason = HOLD_REASON_NONE;
        g_state = APP_DISC_STATE_WARM;
        app_log_write("DAC_AUTO_RESTART: WARM\r\n");
        app_log_write("DISC_WARM: REASON=DAC_SET, SKIP_10S=1\r\n");
    } else {
        g_auto_discipline_enabled = 0U;
        g_state = APP_DISC_STATE_MANUAL;
        app_log_write("DAC_MANUAL: ON\r\n");
    }

    app_log_write("TRAIN_MARK: DAC_SET=");
    app_print_uv(uv);
    app_log_printf(", CODE=%lu, MODE=%s\r\n",
                   (unsigned long)code,
                   (restart_auto != 0U) ? "AUTO" : "MANUAL");
    app_log_write("TRAIN_WAIT: DAC=");
    app_print_uv(uv);
    app_log_write(", WAIT=COUNT10_COUNT50\r\n");
}

static int32_t app_abs_i32(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Only reachable from app_gpsdo_rtc_phase_step_poll(); compiled out together
 * with it so the disabled build has no unreferenced static functions. */
#if GPSDO_RTC_PHASE_STEP_ENABLE != 0U
static int32_t app_clamp_rtc_phase_step_count(int32_t step_count)
{
    if (step_count > GPSDO_RTC_PHASE_MAX_STEP_COUNT) {
        return GPSDO_RTC_PHASE_MAX_STEP_COUNT;
    }
    if (step_count < -GPSDO_RTC_PHASE_MAX_STEP_COUNT) {
        return -GPSDO_RTC_PHASE_MAX_STEP_COUNT;
    }
    return step_count;
}

static int app_cpld_write_rtc_phase_step(int32_t step_count)
{
    HAL_StatusTypeDef st;
    int32_t clamped = app_clamp_rtc_phase_step_count(step_count);

    st = app_cpld_write_rtc_phase_step_raw(clamped);
    if (st != HAL_OK) {
        if (app_log_event_enabled() != 0U) {
            app_log_printf("GPSDO_EVT,MS=%lu,TYPE=RTC_PHASE_STEP_WRITE_FAIL,STEP_CNT=%ld,STATUS=%lu\r\n",
                           (unsigned long)HAL_GetTick(),
                           (long)clamped,
                           (unsigned long)st);
        }
        return -1;
    }

    return 0;
}

static void app_log_rtc_phase_step_event(uint32_t ms, int32_t step_cnt, int32_t step_ns,
                                         int32_t ph_raw_ns, int32_t ph_target_ns,
                                         int32_t ph_corr_ns, const char *reason)
{
    if (app_log_event_enabled() == 0U) {
        return;
    }

    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=RTC_PHASE_STEP,STEP_CNT=%ld,STEP_NS=%ld,PH_RAW_NS=%ld,PH_TARGET_NS=%ld,PH_CORR_NS=%ld,REASON=%s\r\n",
                   (unsigned long)ms,
                   (long)step_cnt,
                   (long)step_ns,
                   (long)ph_raw_ns,
                   (long)ph_target_ns,
                   (long)ph_corr_ns,
                   (reason != (const char *)0) ? reason : "?");
}
#endif /* GPSDO_RTC_PHASE_STEP_ENABLE */

/* Only reachable from app_gpsdo_phase_fine_poll(). */
#if GPSDO_PHASE_FINE_ENABLE != 0U
static void app_gpsdo_phase_fine_log(uint32_t ms, int32_t ferr_mhz, int32_t err10, int32_t err50,
                                     int32_t step_uv, uint32_t from_uv, uint32_t to_uv,
                                     uint16_t code, uint8_t applied, const char *reason)
{
    app_log_printf("PHASE_FINE,MS=%lu,FERR_MHZ=%ld,E10=%ld,E50=%ld",
                   (unsigned long)ms,
                   (long)ferr_mhz,
                   (long)err10,
                   (long)err50);
    if (applied != 0U) {
        app_log_printf(",STEP_UV=%ld", (long)step_uv);
    }
    app_log_write(",FROM=");
    app_print_uv(from_uv);
    app_log_write(",TO=");
    app_print_uv(to_uv);
    if (applied != 0U) {
        app_log_printf(",CODE=%lu", (unsigned long)code);
    }
    app_log_printf(",APPLIED=%lu,REASON=%s\r\n",
                   (unsigned long)applied,
                   (reason != (const char *)0) ? reason : "?");
}
#endif /* GPSDO_PHASE_FINE_ENABLE */

static void app_gpsdo_phase_fine_poll(void)
{
#if GPSDO_PHASE_FINE_ENABLE == 0U
    /* Compiled out instead of returned early from, so the disabled build leaves
     * no statically unreachable code behind (armcc #111-D). */
    app_phase_fine_mark(HAL_GetTick(), 0U, 0, "DISABLED");
#else
    uint32_t ms = HAL_GetTick();
    app_phase10_diag_t phase10;
    app_cpld_phase_t phase_now;
    const app_rtc_state_t *rtc;
    uint32_t count10 = 0UL;
    uint32_t count50 = 0UL;
    int32_t err10 = 0;
    int32_t err50 = 0;
    int32_t step_uv;
    int8_t dir;
    uint32_t from_uv;
    uint32_t to_uv;
    uint16_t code;
    HAL_StatusTypeDef st;

    if (g_state != APP_DISC_STATE_NORMAL) {
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "NOT_NORMAL");
        return;
    }
    if ((app_gpsdo_get_last_disc10(&count10, &err10) == 0U) || (app_abs_i32(err10) > 1L)) {
        (void)count10;
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "E10_NOT_SMALL");
        return;
    }
    /* Fetch DISC50 for logging only, no DAC gating */
    (void)app_gpsdo_get_last_disc50(&count50, &err50);
    g_phase_fine_zone = app_phase_fine_zone_update(ms);
    if (g_phase_fine_zone == 0U) {
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "NOT_PHASE_ZONE");
        return;
    }
    st = app_cpld_read_phase(&phase_now);
    if (st != HAL_OK) {
        app_gpsdo10_reset_phase_history();
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "SPI_FAIL");
        return;
    }
    if ((phase_now.phase_status & CPLD_RTC_PHASE_VALID_MASK) == 0UL) {
        app_gpsdo10_reset_phase_history();
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "WAIT_PHASE_SETTLE");
        return;
    }
    if ((phase_now.phase_status & CPLD_RTC_PHASE_ALIVE_MASK) == 0UL) {
        app_gpsdo10_reset_phase_history();
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "WAIT_PHASE_SETTLE");
        return;
    }
    if (((phase_now.phase_status & CPLD_RTC_PHASE_GPS_SEEN_MASK) == 0UL) ||
        ((phase_now.phase_status & CPLD_RTC_PHASE_RTC_SEEN_MASK) == 0UL)) {
        app_gpsdo10_reset_phase_history();
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "WAIT_PHASE_SETTLE");
        return;
    }
    if (app_gpsdo10_get_phase10(&phase10) == 0U) {
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "WAIT_PHASE_SETTLE");
        return;
    }
    if (phase10.valid_windows < 2U) {
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "WAIT_PHASE_SETTLE");
        return;
    }
    if ((phase10.dt_seconds < 9UL) || (phase10.dt_seconds > 11UL) ||
        (phase10.dtick > 1000L) || (phase10.dtick < -1000L)) {
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "WAIT_PHASE_SETTLE");
        return;
    }
    rtc = app_rtc_get_state();
    if ((rtc->rmc_valid == 0U) ||
        (app_gps_ok() == 0U)) {
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "GPS_INVALID");
        return;
    }

    from_uv = app_log_dac_code_to_uv(g_current_dac_code);
    if (app_abs_i32(phase10.freq_err_mhz) <= GPSDO_PHASE_FINE_DEADBAND_MHZ) {
        app_gpsdo_phase_fine_log(ms, phase10.freq_err_mhz, err10, err50, 0, from_uv, from_uv, g_current_dac_code,
                                 0U, "DEADBAND");
        app_phase_fine_confirm_reset();
        app_phase_fine_mark(ms, 0U, 0, "DEADBAND");
        return;
    }

    dir = (phase10.freq_err_mhz > GPSDO_PHASE_FINE_DEADBAND_MHZ) ? 1 : -1;
    if (g_phase_fine_confirm_dir == dir) {
        if (g_phase_fine_confirm_count < 255U) {
            g_phase_fine_confirm_count++;
        }
    } else {
        g_phase_fine_confirm_dir = dir;
        g_phase_fine_confirm_count = 1U;
    }
    if (g_phase_fine_confirm_count < GPSDO_PHASE_FINE_CONFIRM_N) {
        app_phase_fine_mark(ms, 0U, 0, "WAIT_CONFIRM");
        return;
    }
    if ((ms - g_phase_fine_last_adjust_ms) < GPSDO_PHASE_FINE_INTERVAL_MS) {
        app_phase_fine_mark(ms, 0U, 0, "WAIT_INTERVAL");
        return;
    }

    {
        uint16_t current_code = g_current_dac_code;
        uint16_t target_code;

        target_code = current_code;
        if ((dir > 0) && (current_code > 0U)) {
            target_code = (uint16_t)(current_code - GPSDO_PHASE_FINE_MAX_STEP_CODE);
        } else if ((dir < 0) && (current_code < 65535U)) {
            target_code = (uint16_t)(current_code + GPSDO_PHASE_FINE_MAX_STEP_CODE);
        }
        code = target_code;
        to_uv = app_log_dac_code_to_uv(code);
        step_uv = (int32_t)((int64_t)to_uv - (int64_t)from_uv);
    }

    if (code == g_current_dac_code) {
        app_phase_fine_mark(ms, 0U, step_uv, "CODE_UNCHANGED");
        if (app_log_event_enabled() != 0U) {
            app_log_printf("GPSDO_EVT,MS=%lu,TYPE=PHASE_FINE_SKIP,REASON=CODE_UNCHANGED,FERR_MHZ=%ld,CUR_CODE=%lu,STEP_UV=%ld\r\n",
                           (unsigned long)ms,
                           (long)phase10.freq_err_mhz,
                           (unsigned long)g_current_dac_code,
                           (long)step_uv);
        }
        return;
    }

#if GPSDO_RTC_PHASE_USE_DAC_FINE == 0U
    app_phase_fine_mark(ms, 0U, step_uv, "DAC_FINE_DISABLED");
    app_gpsdo_phase_fine_log(ms, phase10.freq_err_mhz, err10, err50, step_uv, from_uv, to_uv, code,
                             0U, "DAC_FINE_DISABLED");
    app_phase_fine_confirm_reset();
#else
    if ((step_uv != 0) && (DAC_AUTO_WRITE != 0U) && (GPSDO_DRY_RUN == 0U)) {
        st = app_dac8571_write_code(code);
        if (st == HAL_OK) {
            uint16_t old_code = g_current_dac_code;
            g_current_dac_uv = to_uv;
            g_current_dac_code = code;
            code = g_current_dac_code;
            app_phase_fine_mark(ms, 1U, step_uv, "PHASE_FINE");
            app_gpsdo_phase_fine_log(ms, phase10.freq_err_mhz, err10, err50, step_uv, from_uv, to_uv, code,
                                     1U, "PHASE_FINE");
            if ((app_log_event_enabled() != 0U) && (app_log_dac_adjust_enabled() != 0U)) {
                app_log_printf("GPSDO_EVT,MS=%lu,TYPE=DAC_ADJUST,SOURCE=PHASE_FINE,FROM_CODE=%lu,TO_CODE=%lu,FROM_V=",
                               (unsigned long)ms,
                               (unsigned long)old_code,
                               (unsigned long)code);
                app_log_print_voltage_uv6(from_uv);
                app_log_write(",TO_V=");
                app_log_print_voltage_uv6(to_uv);
                app_log_printf(",STEP_UV=%ld,REASON=PHASE_FINE,PHFERR=%ld,PFC_CNT=%lu\r\n",
                               (long)step_uv,
                               (long)phase10.freq_err_mhz,
                               (unsigned long)g_phase_fine_confirm_count);
            }
            app_phase_fine_confirm_reset();
        } else {
            app_phase_fine_mark(ms, 0U, step_uv, "DAC_FAIL");
            app_log_dac_adjust_fail_event("PHASE_FINE", app_dac8571_voltage_to_code(from_uv),
                                          code, "PHASE_FINE", st);
        }
    }
#endif /* GPSDO_RTC_PHASE_USE_DAC_FINE */
#endif /* GPSDO_PHASE_FINE_ENABLE */
}

static void app_gpsdo_rtc_phase_step_poll(void)
{
#if GPSDO_RTC_PHASE_STEP_ENABLE == 0U
    /* Compiled out instead of returned early from; see app_gpsdo_phase_fine_poll. */
    g_rtc_phase_step_last_valid = 0U;
    app_rtc_phase_step_confirm_reset();
    app_rtc_phase_step_mark(0U, 0, 0, "DISABLED");
#else
    uint32_t ms = HAL_GetTick();
    app_cpld_phase_t phase_now;
    app_phase10_diag_t phase10;
    const app_rtc_state_t *rtc = app_rtc_get_state();
    HAL_StatusTypeDef st;
    int32_t err10 = 0;
    int32_t abs_err10;
    int32_t step_cnt = 0;
    int32_t step_ns = 0;
    int8_t dir = 0;
    uint32_t runtime_s = ms / 1000UL;
    const char *reason = "IDLE";

    st = app_cpld_read_phase(&phase_now);
    if (st == HAL_OK) {
        g_rtc_phase_step_last_seq = phase_now.phase_seq;
        g_rtc_phase_step_last_ticks = phase_now.phase_ticks;
        g_rtc_phase_step_last_status = phase_now.phase_status;
        g_rtc_phase_step_last_valid = 1U;
        g_rtc_phase_step_raw_ns = phase_now.phase_ticks * 20L;
        g_rtc_phase_step_target_ns = 0;
        g_rtc_phase_step_corr_ns = g_rtc_phase_step_raw_ns - g_rtc_phase_step_target_ns;
    } else {
        g_rtc_phase_step_last_valid = 0U;
        g_rtc_phase_step_raw_ns = 0;
        g_rtc_phase_step_corr_ns = 0;
    }

    if (g_state != APP_DISC_STATE_NORMAL) {
        reason = "STATE_NOT_NORMAL";
        goto pause_and_exit;
    }
    if ((runtime_s < GPSDO_RTC_PHASE_MIN_RUN_S) || (g_last_disc10_valid == 0U)) {
        reason = (runtime_s < GPSDO_RTC_PHASE_MIN_RUN_S) ? "RUN_LT_MIN" : "NO_E10";
        goto pause_and_exit;
    }
    err10 = g_last_disc10_err;
    abs_err10 = app_abs_i32(err10);
    if (abs_err10 >= 4L) {
        reason = "E10_GTE4";
        goto pause_and_exit;
    }
    if (abs_err10 > 2L) {
        reason = "E10_NOT_SMALL";
        goto pause_and_exit;
    }
    if ((app_gps_ok() == 0U) || (rtc->rmc_valid == 0U)) {
        reason = "GPS_INVALID";
        goto pause_and_exit;
    }
    if (st != HAL_OK) {
        reason = "SPI_READ_FAIL";
        goto pause_and_exit;
    }
    if ((phase_now.phase_status & (CPLD_RTC_PHASE_VALID_MASK | CPLD_RTC_PHASE_GPS_SEEN_MASK |
                                   CPLD_RTC_PHASE_RTC_SEEN_MASK | CPLD_RTC_PHASE_ALIVE_MASK)) !=
        (CPLD_RTC_PHASE_VALID_MASK | CPLD_RTC_PHASE_GPS_SEEN_MASK |
         CPLD_RTC_PHASE_RTC_SEEN_MASK | CPLD_RTC_PHASE_ALIVE_MASK)) {
        reason = "PHASE_INVALID";
        goto pause_and_exit;
    }
    if ((g_last_seq10_valid == 0U) || (g_last_disc10_valid == 0U)) {
        reason = "SEQ10_INVALID";
        goto pause_and_exit;
    }
    if ((app_gpsdo10_get_phase10(&phase10) == 0U) || (phase10.valid_windows < GPSDO_RTC_PHASE_CONFIRM_N)) {
        reason = "WAIT_STABLE_WINDOWS";
        goto pause_and_exit;
    }

    if (app_abs_i32(g_rtc_phase_step_corr_ns) <= GPSDO_RTC_PHASE_DEADBAND_NS) {
        reason = "DEADBAND";
        app_rtc_phase_step_confirm_reset();
        app_rtc_phase_step_mark(0U, 0, 0, reason);
        return;
    }

    dir = (g_rtc_phase_step_corr_ns > 0) ? -1 : 1;
    if (g_rtc_phase_step_confirm_dir == dir) {
        if (g_rtc_phase_step_confirm_count < 255U) {
            g_rtc_phase_step_confirm_count++;
        }
    } else {
        g_rtc_phase_step_confirm_dir = dir;
        g_rtc_phase_step_confirm_count = 1U;
    }
    if (g_rtc_phase_step_confirm_count < GPSDO_RTC_PHASE_CONFIRM_N) {
        app_rtc_phase_step_mark(0U, 0, 0, "WAIT_CONFIRM");
        return;
    }

    step_cnt = app_clamp_rtc_phase_step_count((int32_t)dir);
    step_ns = step_cnt * 81;
    if (app_cpld_write_rtc_phase_step(step_cnt) != 0) {
        app_rtc_phase_step_mark(0U, 0, 0, "SPI_WRITE_FAIL");
        return;
    }

    app_rtc_phase_step_mark(1U, step_cnt, step_ns, "APPLY");
    app_log_rtc_phase_step_event(ms, step_cnt, step_ns, g_rtc_phase_step_raw_ns,
                                 g_rtc_phase_step_target_ns, g_rtc_phase_step_corr_ns, "APPLY");
    app_rtc_phase_step_confirm_reset();
    return;

pause_and_exit:
    app_rtc_phase_step_confirm_reset();
    if (app_cpld_write_rtc_phase_step(0) == 0) {
        app_rtc_phase_step_mark(0U, 0, 0, reason);
    } else {
        app_rtc_phase_step_mark(0U, 0, 0, "SPI_WRITE_FAIL");
    }
#endif /* GPSDO_RTC_PHASE_STEP_ENABLE */
}

#if GPSDO_REQUIRE_GPS_FIX_FOR_DISC
/* Log the "waiting for a fix" interlock only once per WARM episode. */
static uint8_t g_warm_fix_interlock_logged;
/* Last time a valid fix was seen, for the fix-loss holdover check. */
static uint32_t g_gps_fix_last_ms;
static uint8_t g_gps_fix_seen;
#endif

void app_gpsdo_poll_1s(void)
{
    uint32_t sig = 0UL;
    uint32_t status = 0UL;
    uint32_t count10 = 0UL;
    uint32_t count50 = 0UL;
    uint8_t seq10 = 0U;
    uint8_t seq50 = 0U;
    uint8_t clr10_cnt = 0U;
    uint8_t clr50_cnt = 0U;
    uint8_t seq_ok;
    uint8_t seq10_changed;
    uint8_t seq10_consecutive;
    uint32_t seq10_age_ms;
    uint8_t fpga_ok;
    uint8_t gps_valid;
    uint8_t valid10;
    uint32_t seqraw;
    int32_t err10_local;
    int32_t abs_err10;
    int32_t step10_uv = 0;
    int32_t track10_slope_num = (int32_t)DAC_SLOPE_NUM;
    uint32_t disc10_slope_num_log = (uint32_t)DAC_SLOPE_NUM;
    uint8_t step10_code_valid = 0U;
    uint16_t step10_code = 0U;
    uint16_t step10_target_code = 0U;
    uint8_t disc_cooldown_active = 0U;
    uint8_t disc_clear_after_adjust = GPSDO_CLEAR_AFTER_DISC10_ADJUST;
    const char *disc10_reason = (const char *)0;
    uint8_t disc10_applied;
    uint32_t window_dac_uv;
    uint16_t window_dac_code;
    HAL_StatusTypeDef st_sig;
    HAL_StatusTypeDef st_status;
    HAL_StatusTypeDef st_seq;
    const app_cpld_signature_stable_t *sig_detail;
    app_cpld_measurement_t measurement;
    app_pps_snapshot_t pps_window;
    HAL_StatusTypeDef st_measurement;

    st_measurement = app_cpld_read_measurement(&measurement);
    sig_detail = app_cpld_get_last_signature_stable();
    st_sig = st_measurement;
    st_status = st_measurement;
    st_seq = st_measurement;
    sig = measurement.signature;
    status = measurement.status;
    count10 = measurement.count10;
    count50 = 0UL;
    seq10 = measurement.seq10;
    seq50 = measurement.seq50;
    clr10_cnt = measurement.clr10_cnt;
    clr50_cnt = measurement.clr50_cnt;
    (void)count50;
    (void)disc_cooldown_active;

    if ((g_sig_retry_ok_printed == 0U) &&
        (sig_detail->retry != 0U) &&
        (st_sig == HAL_OK) &&
        (sig == CPLD_SIGNATURE_VALUE)) {
        g_sig_retry_ok_printed = 1U;
        app_log_printf("FPGA_SIG_RETRY_OK: TRY=%lu SIG=0x%08lX SHIFTED_COUNT=%lu\r\n",
                       (unsigned long)(sig_detail->retry_count + 1U),
                       (unsigned long)sig,
                       (unsigned long)sig_detail->shifted_count);
        if (app_log_event_enabled() != 0U) {
            app_log_printf("GPSDO_EVT,MS=%lu,TYPE=FPGA_SIG_RETRY_OK,TRY=%lu,SIG=0x%08lX\r\n",
                           (unsigned long)HAL_GetTick(),
                           (unsigned long)(sig_detail->retry_count + 1U),
                           (unsigned long)sig);
        }
    }

    app_gpsdo_print_fpga_init(status, sig);

    if (g_auto_discipline_enabled == 0U) {
        g_state = APP_DISC_STATE_MANUAL;
        return;
    }
    if (g_state == APP_DISC_STATE_MANUAL) {
        app_enter_warm("RECOVERY");
        return;
    }

    fpga_ok = ((st_sig == HAL_OK) && (st_status == HAL_OK) && (app_fpga_ready(status, sig) != 0U)) ? 1U : 0U;
    gps_valid = app_gps_ok();

#if GPSDO_REQUIRE_GPS_FIX_FOR_DISC
    /* Remember when a fix was last confirmed, for the fix-loss holdover. */
    if (gps_valid != 0U) {
        g_gps_fix_last_ms = HAL_GetTick();
        g_gps_fix_seen = 1U;
    }
#endif
    valid10 = (((status & CPLD_STATUS_FINISH_10S_MASK) != 0UL) ? 1U : 0U);
    seq_ok = (st_seq == HAL_OK) ? 1U : 0U;
    seqraw = measurement.window_seq_raw;
    seq10_changed = ((seq_ok != 0U) && (g_last_seq10_valid != 0U) && (seq10 != g_last_seq10)) ? 1U : 0U;
    seq10_consecutive = ((seq10_changed != 0U) && ((uint8_t)(g_last_seq10 + 1U) == seq10)) ? 1U : 0U;
    seq10_age_ms = (g_last_seq10_valid != 0U) ? (HAL_GetTick() - g_last_seq10_change_ms) : 0xFFFFFFFFUL;

    if (g_state == APP_DISC_STATE_HOLDOVER) {
        if (fpga_ok == 0U) {
            app_enter_fpga_not_ready_holdover(sig_detail, status, st_status, seqraw, st_seq);
            return;
        }
        /* HOLDOVER recovery: only require FPGA_OK, not GPS_VALID */
        /* GPS_TIME_VALID issues should not prevent recovery */
        app_enter_warm("RECOVERY");
        return;
    }

    if (g_state == APP_DISC_STATE_WARM) {
        if (fpga_ok == 0U) {
            app_enter_fpga_not_ready_holdover(sig_detail, status, st_status, seqraw, st_seq);
            return;
        }
        /* GPS_TIME_VALID issues should not cause HOLD in WARM state */
        if (gps_valid == 0U) {
            app_log_gps_invalid_detail(HAL_GetTick(), gps_valid);
            /* Just log warning, don't enter HOLD */
        }
        if (g_warm_first_seq10_seen == 0U) {
            if ((valid10 != 0U) && (seq_ok != 0U)) {
                g_warm_first_seq10_seen = 1U;
                g_warm_first_seq10 = seq10;
                g_last_seq10 = seq10;
                g_last_seq10_valid = 1U;
                g_last_seq10_change_ms = HAL_GetTick();
                app_log_write("DISC_WARM: FIRST_COUNT10_SEEN, SKIP=1\r\n");
                return;
            }
            app_warm_wait_log(fpga_ok, gps_valid, valid10, 0U, seqraw, seq10, seq50, clr10_cnt, clr50_cnt, count10);
            return;
        }
#if GPSDO_REQUIRE_GPS_FIX_FOR_DISC
        /*
         * INTERLOCK: do not start disciplining on a PPS that may be free running.
         *
         * The CPLD derives every window boundary from the module's TIMEPULSE, so
         * COUNT10 measures the GPS second only while the module actually has a
         * time solution.  Before that the TIMEPULSE comes from the module's own
         * RTC/TCXO, and COUNT10 then reports the MODULE's crystal error rather
         * than the VCTCXO's.  See GPSDO_REQUIRE_GPS_FIX_FOR_DISC in app_config.h
         * for the measurement that pinned this down (-142 counts / -1.16 ppm
         * while the oscillator itself was within +0.01 ppm).
         *
         * Staying in WARM is the right failure mode: the DAC is simply not
         * touched, and the periodic DISC_WARM_WAIT row already shows GPS=0.
         */
        if (gps_valid == 0U) {
            if (g_warm_fix_interlock_logged == 0U) {
                g_warm_fix_interlock_logged = 1U;
                app_log_write("DISC_WARM_BLOCK: REASON=NO_GPS_FIX,PPS_MAY_BE_FREE_RUNNING\r\n");
            }
            app_warm_wait_log(fpga_ok, gps_valid, valid10, 0U, seqraw, seq10, seq50, clr10_cnt, clr50_cnt, count10);
            return;
        }
        g_warm_fix_interlock_logged = 0U;
#endif

        if ((valid10 != 0U) && (seq_ok != 0U) && (seq10_consecutive != 0U) &&
            (seq10_age_ms <= GPSDO_SEQ10_MAX_AGE_MS)) {
            g_state = APP_DISC_STATE_TRACK10;
            g_track10_enter_ms = HAL_GetTick();
            g_e10_lock_recent[0] = 0;
            g_e10_lock_recent[1] = 0;
            g_e10_lock_recent[2] = 0;
            g_e10_lock_recent_idx = 0U;
            app_log_write("DISC_WARM_DONE: ENTER=TRACK10_WAIT_FIRST_E50\r\n");
        } else {
            if ((valid10 != 0U) && (seq_ok != 0U) && (seq10_changed != 0U)) {
                app_log_printf("DISC_WARM_RESTART: REASON=SEQ_GAP,LAST=%lu,NOW=%lu,AGE_MS=%lu\r\n",
                               (unsigned long)g_last_seq10,
                               (unsigned long)seq10,
                               (unsigned long)seq10_age_ms);
                g_warm_first_seq10 = seq10;
                g_last_seq10 = seq10;
                g_last_seq10_change_ms = HAL_GetTick();
                app_reset_sw_accumulators();
                return;
            }
            app_warm_wait_log(fpga_ok, gps_valid, valid10,
                              (uint8_t)(((valid10 != 0U) && (seq_ok != 0U) && (seq10 == g_warm_first_seq10)) ? 1U : 0U),
                              seqraw, seq10, seq50, clr10_cnt, clr50_cnt, count10);
            return;
        }
    }

    if (fpga_ok == 0U) {
        app_enter_fpga_not_ready_holdover(sig_detail, status, st_status, seqraw, st_seq);
        return;
    }

#if GPSDO_REQUIRE_GPS_FIX_FOR_DISC
    /*
     * Fix-loss holdover.  Once disciplined, losing the fix means the module's
     * TIMEPULSE falls back to its own crystal -- still ~1 Hz, still perfectly
     * framed, but no longer a reference (see GPSDO_REQUIRE_GPS_FIX_FOR_DISC).
     * Chasing it would walk the DAC away from an already correct setting, so
     * freeze instead and let the next real fix bring us back.  The delay rides
     * out a brief satellite dropout without breaking a good lock.
     */
    if ((gps_valid == 0U) && (g_gps_fix_seen != 0U) &&
        (g_state != APP_DISC_STATE_HOLDOVER) &&
        ((HAL_GetTick() - g_gps_fix_last_ms) > GPSDO_FIX_LOSS_HOLD_MS)) {
        app_enter_holdover(HOLD_REASON_GPS_FIX_LOST);
        return;
    }
#endif

    /* GPS_TIME_VALID issues should not cause HOLD in TRACK10/NORMAL state */
    if (gps_valid == 0U) {
        app_log_gps_invalid_detail(HAL_GetTick(), gps_valid);
        /* Just log warning, continue with frequency control */
    }

    /* A 50 s result may contain only five strictly adjacent 10 s windows. */
    if ((valid10 != 0U) && (seq_ok != 0U) && (seq10_changed != 0U) &&
        ((seq10_consecutive == 0U) || (seq10_age_ms > GPSDO_SEQ10_MAX_AGE_MS))) {
        app_log_printf("DISC_REJECT: SOURCE=SW_E50,REASON=SEQ_GAP,LAST=%lu,NOW=%lu,AGE_MS=%lu\r\n",
                       (unsigned long)g_last_seq10,
                       (unsigned long)seq10,
                       (unsigned long)seq10_age_ms);
        app_reset_sw_accumulators();
        g_last_seq10 = seq10;
        g_last_seq10_change_ms = HAL_GetTick();
        return;
    }
    /* Update seq10 change timestamp when we detect a new window */
    if ((seq_ok != 0U) && (valid10 != 0U) && (g_last_seq10_valid != 0U) && (seq10 != g_last_seq10)) {
        g_last_seq10_change_ms = HAL_GetTick();
        g_last_seq10 = seq10;
    }

    /* If seq10 tracking was reset, initialize on first valid window */
    if ((g_last_seq10_valid == 0U) && (seq_ok != 0U) && (valid10 != 0U)) {
        g_last_seq10 = seq10;
        g_last_seq10_valid = 1U;
        g_last_seq10_change_ms = HAL_GetTick();
    }

    /* FPGA_SEQ timeout: only HOLD if no valid seq10 change for >25s */
    if (g_last_seq10_valid != 0U) {
        uint32_t fpga_seq_age_ms = HAL_GetTick() - g_last_seq10_change_ms;

        if (fpga_seq_age_ms > 25000UL) {
            app_pps_snapshot_t pps_snap;
            uint32_t pps_gpio_age_ms;
            app_pps_get_snapshot(&pps_snap);
            pps_gpio_age_ms = (pps_snap.last_tick_ms == 0xFFFFFFFFUL) ? 999999UL : (HAL_GetTick() - pps_snap.last_tick_ms);
            app_log_printf("DISC_HOLD_DEBUG: FPGA_SEQ_AGE_MS=%lu,PPS_GPIO_AGE_MS=%lu,valid10=%lu,seq_ok=%lu\r\n",
                           (unsigned long)fpga_seq_age_ms,
                           (unsigned long)pps_gpio_age_ms,
                           (unsigned long)valid10,
                           (unsigned long)seq_ok);
            app_enter_holdover(HOLD_REASON_PPS_TIMEOUT);
            return;
        }
    }

    /* If no new seq10 this poll, skip DISC10 processing (normal between 10s windows) */
    if (!((valid10 != 0U) && (seq_ok != 0U) && (seq10_changed != 0U))) {
        return;
    }

    /* The MCU sees the same PPS net.  Require all ten one-second intervals to
     * be sane before this 10 s count is allowed into a 50 s result. */
    app_pps_get_snapshot(&pps_window);
    if (pps_window.valid_interval_streak < 10U) {
        app_log_printf("DISC_REJECT: SOURCE=SW_E50,REASON=PPS_INTERVAL,STREAK=%lu,LAST_MS=%lu,BAD_COUNT=%lu\r\n",
                       (unsigned long)pps_window.valid_interval_streak,
                       (unsigned long)pps_window.interval_ms,
                       (unsigned long)pps_window.invalid_interval_count);
        app_reset_sw_accumulators();
        return;
    }
    /*
     * Ten GPS seconds exactly.  pps_window.count is the MCU's own PA1 edge
     * count, so this compares the CPLD's window against an independent count of
     * the same one-second pulses: consecutive windows must differ by 10.
     * A first window only establishes the baseline (the counter is cumulative
     * from boot, its absolute value says nothing).
     */
    if (g_last_window_pps_valid == 0U) {
        g_last_window_pps = pps_window.count;
        g_last_window_pps_valid = 1U;
    } else if ((pps_window.count - g_last_window_pps) != 10UL) {
        app_log_printf("DISC_REJECT: SOURCE=SW_E50,REASON=PPS_DELTA,LAST=%lu,NOW=%lu,DELTA=%lu\r\n",
                       (unsigned long)g_last_window_pps,
                       (unsigned long)pps_window.count,
                       (unsigned long)(pps_window.count - g_last_window_pps));
        g_last_window_pps = pps_window.count;
        app_reset_sw_accumulators();
        return;
    } else {
        g_last_window_pps = pps_window.count;
    }
    /* Valid new seq10 window - continue processing */
    if (g_disc_adjust_cooldown_windows > 0U) {
        disc_cooldown_active = 1U;
        g_disc_adjust_cooldown_windows--;
    }

    err10_local = (int32_t)((int64_t)count10 - (int64_t)GPSDO_TARGET_COUNT10);
    abs_err10 = (err10_local < 0) ? -err10_local : err10_local;
    if (app_abs_err_in_range(err10_local, (int32_t)DAC_ERR10_ABS_MAX_VALID) == 0U) {
        app_log_printf("DISC_REJECT: SOURCE=DISC10, REASON=ERR_OUT_OF_RANGE, ERR=%ld\r\n", (long)err10_local);
        app_reset_sw_accumulators();
        return;
    }

    g_last_disc10_valid = 1U;
    g_last_disc10_seq = seq10;
    g_last_disc10_count = count10;
    g_last_disc10_err = err10_local;

    /* Exact software 50 s window: E50 = sum(E10[0..4]).  The shared
     * valid flag is separate from the 8-bit sequence value so wrap 255->0
     * remains a normal consecutive transition. */
    if ((seq_ok != 0U) && (seq10_changed != 0U)) {
        g_sw_e50_new = 0U;
        g_sw_e100_new = 0U;
        g_sw_e200_new = 0U;
        g_sw_e1000_new = 0U;
        if ((g_sw_e50_last_seq10_valid != 0U) &&
            ((uint8_t)(g_sw_e50_last_seq10 + 1U) == seq10)) {
            g_sw_e50_err += err10_local;   g_sw_e50_count++;
            g_sw_e100_sum += err10_local;  g_sw_e100_count++;
            g_sw_e200_sum += err10_local;  g_sw_e200_count++;
            g_sw_e1000_sum += err10_local; g_sw_e1000_count++;
        } else {
            g_sw_e50_err = err10_local;    g_sw_e50_count = 1U;
            g_sw_e100_sum = err10_local;   g_sw_e100_count = 1U;
            g_sw_e200_sum = err10_local;   g_sw_e200_count = 1U;
            g_sw_e1000_sum = err10_local;  g_sw_e1000_count = 1U;
            g_sw_e50_valid = 0U;
            g_sw_e100_valid = 0U;
            g_sw_e200_valid = 0U;
            g_sw_e1000_valid = 0U;
        }
        g_sw_e50_last_seq10 = seq10;
        g_sw_e50_last_seq10_valid = 1U;

        if (g_sw_e50_count >= GPSDO_SW_E50_WINDOWS) {
            g_sw_e50_last = g_sw_e50_err;
            g_sw_e50_valid = 1U;
            g_sw_e50_new = 1U;
            g_sw_e50_result_seq++;
            g_sw_e50_err = 0;
            g_sw_e50_count = 0U;
        }
        if (g_sw_e100_count >= GPSDO_SW_E100_WINDOWS) {
            g_sw_e100_last = g_sw_e100_sum;
            g_sw_e100_valid = 1U;
            g_sw_e100_new = 1U;
            g_sw_e100_sum = 0;
            g_sw_e100_count = 0U;
        }
        if (g_sw_e200_count >= GPSDO_SW_E200_WINDOWS) {
            g_sw_e200_last = g_sw_e200_sum;
            g_sw_e200_valid = 1U;
            g_sw_e200_new = 1U;
            g_sw_e200_sum = 0;
            g_sw_e200_count = 0U;
        }
        if (g_sw_e1000_count >= GPSDO_SW_E1000_WINDOWS) {
            g_sw_e1000_last = g_sw_e1000_sum;
            g_sw_e1000_valid = 1U;
            g_sw_e1000_new = 1U;
            g_sw_e1000_sum = 0;
            g_sw_e1000_count = 0U;
        }
    }

    g_last_disc10_state = app_state_str();
    app_print_train_line("TRAIN10", count10, "ERR10", err10_local);
    window_dac_uv = g_current_dac_uv;
    window_dac_code = g_current_dac_code;
    (void)window_dac_uv;
    (void)window_dac_code;
    g_phase_fine_zone = app_phase_fine_zone_update(HAL_GetTick());

    if (g_wait_first_e50_after_warm != 0U) {
        if ((g_sw_e50_new == 0U) || (g_sw_e50_valid == 0U)) {
            app_disc10_mark(0U, 0, "WAIT_FIRST_E50");
            app_disc50_mark(0U, 0, "WAIT_FIRST_E50");
            app_disc_reason_set("WAIT_FIRST_E50");
            return;
        }

        g_wait_first_e50_after_warm = 0U;
        g_state = APP_DISC_STATE_NORMAL;
        g_e10_lock_cnt = 0U;
        g_normal_e10_out_cnt = 0U;
        g_normal_e10_out_sign = 0;
        app_disc50_confirm_reset();
        app_disc10_mark(0U, 0, "FIRST_E50_NORMAL50");
        app_log_printf("DISC_EVENT: FIRST_E50_READY, E50=%ld, ENTER=NORMAL50\r\n", (long)g_sw_e50_last);
        if (app_log_event_enabled() != 0U) {
            app_log_printf("GPSDO_EVT,MS=%lu,TYPE=FIRST_E50_READY,E50=%ld,ENTER=NORMAL50\r\n",
                           (unsigned long)HAL_GetTick(),
                           (long)g_sw_e50_last);
        }
        (void)app_disc50_process_window(seq_ok, seq10, 1U, 1U);
        return;
    }

    if (g_state == APP_DISC_STATE_NORMAL) {
        if (abs_err10 >= (int32_t)GPSDO_NORMAL_E10_OUT_FAST_ABS) {
            int8_t sign_now = (err10_local > 0) ? 1 : -1;
            if ((g_normal_e10_out_cnt != 0U) && (g_normal_e10_out_sign == sign_now)) {
                g_normal_e10_out_cnt++;
            } else {
                g_normal_e10_out_cnt = 1U;
                g_normal_e10_out_sign = sign_now;
            }
            if (g_normal_e10_out_cnt >= GPSDO_NORMAL_E10_OUT_CONFIRM_N) {
                g_state = APP_DISC_STATE_TRACK10;
                g_e10_lock_cnt = 0U;
                g_normal_e10_out_cnt = 0U;
                g_normal_e10_out_sign = 0;
                app_disc50_confirm_reset();
                app_disc50_mark(0U, 0, "BACK_TO_TRACK10");
                app_log_printf("DISC_EVENT: E10_OUT_OF_LOCK, E10=%ld, BACK_TO_TRACK10\r\n", (long)err10_local);
                if (app_log_event_enabled() != 0U) {
                    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=BACK_TO_TRACK10,E10=%ld\r\n",
                                   (unsigned long)HAL_GetTick(),
                                   (long)err10_local);
                }
            }
        } else {
            g_normal_e10_out_cnt = 0U;
            g_normal_e10_out_sign = 0;
        }
    }

    if (g_state == APP_DISC_STATE_NORMAL) {
        step10_uv = 0;
        disc10_reason = "NORMAL50_ACTIVE";
        app_disc10_mark(0U, 0, disc10_reason);
    } else {
        uint8_t clamped = 0U;
        uint16_t code_step = app_track10_code_step(abs_err10, &disc10_reason);

        if (code_step > GPSDO_TRACK10_MAX_STEP_CODE) {
            code_step = GPSDO_TRACK10_MAX_STEP_CODE;
        }

        track10_slope_num = app_clamp_track10_slope_num();
        disc10_slope_num_log = (uint32_t)track10_slope_num;

        if (DAC_DISCIPLINE_USE_10S_APPLY == 0U) {
            step10_uv = 0;
            step10_code = 0U;
            disc10_reason = "DISC10_DISABLED";
            app_disc10_mark(0U, 0, disc10_reason);
        } else if (abs_err10 <= GPSDO_TRACK10_DEADBAND_COUNT) {
            step10_uv = 0;
            step10_code = 0U;
            g_e10_lock_cnt++;
            if (g_e10_lock_cnt > 255U) {
                g_e10_lock_cnt = 255U;
            }
            /* Track last 3 E10 for NORMAL50 entry condition */
            g_e10_lock_recent[g_e10_lock_recent_idx] = err10_local;
            g_e10_lock_recent_idx = (g_e10_lock_recent_idx + 1U) % 3U;
            app_disc10_mark(0U, 0, disc10_reason);
        } else {
            int8_t current_sign = (err10_local > 0) ? 1 : -1;
            uint8_t sign_flipped = 0U;

            g_e10_lock_cnt = 0U;

            /* Near-lock sign flip protection */
            if ((g_track10_signflip_valid != 0U) && (abs_err10 <= GPSDO_TRACK10_SIGN_FLIP_ABS_MAX)) {
                if (current_sign != g_track10_signflip_last_sign) {
                    sign_flipped = 1U;
                    g_track10_signflip_hold_count++;
                    if (g_track10_signflip_hold_count >= 2U) {
                        /* Oscillation detected - hold this cycle */
                        g_track10_signflip_hold_count = 0U;
                        step10_uv = 0;
                        step10_code = 0U;
                        disc10_reason = "TRACK10_SIGN_FLIP_HOLD";
                        app_disc10_mark(0U, 0, disc10_reason);
                        g_track10_signflip_last_sign = current_sign;
                        goto disc10_done;
                    }
                } else {
                    g_track10_signflip_hold_count = 0U;
                }
            }
            g_track10_signflip_last_sign = current_sign;
            g_track10_signflip_valid = 1U;

            step10_code = code_step;

            /* Reduce step if sign just flipped in near-lock region */
            if ((sign_flipped != 0U) && (abs_err10 <= GPSDO_TRACK10_SIGN_FLIP_ABS_MAX)) {
                uint16_t half_step = step10_code / 2U;
                if (half_step < GPSDO_TRACK10_SIGN_FLIP_MIN_STEP) {
                    half_step = GPSDO_TRACK10_SIGN_FLIP_MIN_STEP;
                }
                if (half_step < step10_code) {
                    step10_code = half_step;
                    disc10_reason = "TRACK10_SIGN_FLIP_REDUCE";
                }
            }

            step10_target_code = app_step_code_from_err_safe(err10_local, step10_code, &clamped);
            step10_uv = app_step_uv_to_code(step10_target_code);
            step10_code_valid = 1U;
            if (sign_flipped == 0U) {
                disc10_reason = app_track10_clamp_reason(disc10_reason, clamped);
            }
            app_disc10_mark(0U, (int32_t)step10_code, disc10_reason);
        }
disc10_done:
        (void)0;
    }

    disc10_applied = app_apply_code_and_log("DISC10", count10, "COUNT10", "ERR10", err10_local,
                                            step10_uv, DAC_DISCIPLINE_USE_10S_APPLY, disc10_reason,
                                            (const char *)0, 0U, 0UL, seq_ok, seq10,
                                            disc10_slope_num_log, (uint32_t)DAC_SLOPE_DEN,
                                            step10_code_valid, step10_target_code, disc_clear_after_adjust);
    if (disc10_applied != 0U) {
        app_disc10_mark(1U, g_last_disc_step_code, disc10_reason);
    }
    app_disc_reason_set(disc10_reason);
    if (disc10_applied != 0U) {
        return;
    }

    /* E10 lock → NORMAL entry */
    if (g_state == APP_DISC_STATE_TRACK10 && g_e10_lock_cnt >= GPSDO_E10_LOCK_ENTER_N) {
        /* Additional checks: abs_sum of last 3 E10 <= 2, and min time in TRACK10 */
        int32_t abs_sum = app_abs_i32(g_e10_lock_recent[0]) +
                          app_abs_i32(g_e10_lock_recent[1]) +
                          app_abs_i32(g_e10_lock_recent[2]);
        uint32_t track10_duration_ms = HAL_GetTick() - g_track10_enter_ms;

        if ((abs_sum > GPSDO_TRACK10_ENTER_NORMAL_ABS_SUM) ||
            (track10_duration_ms < GPSDO_TRACK10_MIN_TIME_MS)) {
            /* Not ready for NORMAL50 yet, keep in TRACK10 */
            if (app_log_event_enabled() != 0U) {
                app_log_printf("GPSDO_EVT,MS=%lu,TYPE=NORMAL50_WAIT,ABS_SUM=%ld,DURATION_MS=%lu\r\n",
                               (unsigned long)HAL_GetTick(),
                               (long)abs_sum,
                               (unsigned long)track10_duration_ms);
            }
        } else {
            g_state = APP_DISC_STATE_NORMAL;
            g_e10_lock_cnt = 0U;
            g_normal_e10_out_cnt = 0U;
            g_normal_e10_out_sign = 0;
            app_disc50_confirm_reset();
            app_disc50_mark(0U, 0, "ENTER_NORMAL50");
            app_log_printf("DISC_EVENT: ENTER_NORMAL50, E10_LOCK_CNT=%lu, ABS_SUM=%ld, DURATION_MS=%lu\r\n",
                           (unsigned long)GPSDO_E10_LOCK_ENTER_N,
                           (long)abs_sum,
                           (unsigned long)track10_duration_ms);
            if (app_log_event_enabled() != 0U) {
                app_log_printf("GPSDO_EVT,MS=%lu,TYPE=ENTER_NORMAL50\r\n", (unsigned long)HAL_GetTick());
            }
            return;
        }
    }

    if ((g_state == APP_DISC_STATE_NORMAL) && (g_sw_e50_new != 0U) && (g_sw_e50_valid != 0U)) {
        if (app_disc50_process_window(seq_ok, seq10, 0U, 0U) != 0U) {
            return;
        }
    }

    app_gpsdo_phase_fine_poll();
    app_gpsdo_rtc_phase_step_poll();
}

app_discipline_state_t app_gpsdo_get_state(void)
{
    return g_state;
}

const char *app_gpsdo_get_state_string(void)
{
    return app_state_str();
}

uint8_t app_gpsdo_get_last_disc10(uint32_t *count10, int32_t *err10)
{
    if ((count10 == (uint32_t *)0) || (err10 == (int32_t *)0) || (g_last_disc10_valid == 0U)) {
        return 0U;
    }
    *count10 = g_last_disc10_count;
    *err10 = g_last_disc10_err;
    return 1U;
}

uint8_t app_gpsdo_get_last_disc10_info(uint8_t *seq10, uint32_t *count10, int32_t *err10)
{
    if ((seq10 == (uint8_t *)0) ||
        (count10 == (uint32_t *)0) ||
        (err10 == (int32_t *)0) ||
        (g_last_disc10_valid == 0U)) {
        return 0U;
    }

    *seq10 = g_last_disc10_seq;
    *count10 = g_last_disc10_count;
    *err10 = g_last_disc10_err;
    return 1U;
}

uint8_t app_gpsdo_get_last_disc10_snapshot(uint8_t *seq10, uint32_t *count10,
                                           int32_t *err10, const char **state)
{
    if ((state == (const char **)0) ||
        (app_gpsdo_get_last_disc10_info(seq10, count10, err10) == 0U)) {
        return 0U;
    }

    *state = g_last_disc10_state;
    return 1U;
}

uint8_t app_gpsdo_get_last_disc50(uint32_t *count50_msp, int32_t *err50)
{
    if ((count50_msp == (uint32_t *)0) || (err50 == (int32_t *)0) || (g_last_disc50_valid == 0U)) {
        return 0U;
    }
    *count50_msp = g_last_disc50_count;
    *err50 = g_last_disc50_err;
    return 1U;
}

uint8_t app_gpsdo_get_disc_cooldown_windows(void)
{
    return g_disc_adjust_cooldown_windows;
}

uint32_t app_gpsdo_get_last_disc_adjust_ms(void)
{
    return g_last_disc_adjust_ms;
}

int32_t app_gpsdo_get_last_disc_step_code(void)
{
    return g_last_disc_step_code;
}

uint8_t app_gpsdo_get_phase_fine_zone(void)
{
    return app_phase_fine_zone_update(HAL_GetTick());
}

uint8_t app_gpsdo_get_pz_state_ok(void)
{
    (void)app_phase_fine_zone_update(HAL_GetTick());
    return g_pz_state_ok;
}

uint8_t app_gpsdo_get_pz_gps_ok(void)
{
    (void)app_phase_fine_zone_update(HAL_GetTick());
    return g_pz_gps_ok;
}

uint8_t app_gpsdo_get_pz_e10_ok(void)
{
    (void)app_phase_fine_zone_update(HAL_GetTick());
    return g_pz_e10_ok;
}

uint8_t app_gpsdo_get_pz_e50_ok(void)
{
    (void)app_phase_fine_zone_update(HAL_GetTick());
    return g_pz_e50_ok;
}

uint8_t app_gpsdo_get_pz_e50_valid(void)
{
    (void)app_phase_fine_zone_update(HAL_GetTick());
    return g_pz_e50_valid;
}

const char *app_gpsdo_get_disc_reason(void)
{
    return (g_last_disc_reason != (const char *)0) ? g_last_disc_reason : "?";
}

void app_gpsdo_print_phase_fine_status(void)
{
    uint16_t code = app_dac8571_get_last_code();
    uint32_t uv = app_code_to_uv(code);

    app_log_printf("PHASE_FINE_STAT,ENABLE=%lu,STATE=%s,LAST_MS=%lu,LAST_STEP_UV=%ld,LAST_APPLIED=%lu,LAST_REASON=%s,DAC=%lu,DACV=",
                   (unsigned long)GPSDO_PHASE_FINE_ENABLE,
                   app_state_str(),
                   (unsigned long)g_phase_fine_last_ms,
                   (long)g_phase_fine_last_step_uv,
                   (unsigned long)g_phase_fine_last_applied,
                   (g_phase_fine_last_reason != (const char *)0) ? g_phase_fine_last_reason : "?",
                   (unsigned long)code);
    app_print_uv(uv);
    app_log_write("\r\n");
}

void app_gpsdo_get_phase_fine_status(app_phase_fine_status_t *out)
{
    if (out == (app_phase_fine_status_t *)0) {
        return;
    }

    out->enable = GPSDO_PHASE_FINE_ENABLE;
    out->applied = g_phase_fine_last_applied;
    out->step_uv = g_phase_fine_last_step_uv;
    out->last_ms = g_phase_fine_last_ms;
#if GPSDO_PHASE_FINE_ENABLE == 0U
    out->reason = "OFF";
#else
    out->reason = (g_phase_fine_last_reason != (const char *)0) ? g_phase_fine_last_reason : "?";
#endif
    out->last_step_uv = g_phase_fine_last_adjust_step_uv;
    out->last_adjust_ms = g_phase_fine_last_adjust_ms;
    out->confirm_dir = g_phase_fine_confirm_dir;
    out->confirm_count = g_phase_fine_confirm_count;
}

void app_gpsdo_get_rtc_phase_step_status(app_rtc_phase_step_status_t *out)
{
    if (out == (app_rtc_phase_step_status_t *)0) {
        return;
    }

    out->enable = GPSDO_RTC_PHASE_STEP_ENABLE;
    out->applied = g_rtc_phase_step_last_applied;
    out->target_ns = g_rtc_phase_step_target_ns;
    out->raw_ns = g_rtc_phase_step_raw_ns;
    out->corr_ns = g_rtc_phase_step_corr_ns;
    out->step_cnt = g_rtc_phase_step_last_cnt;
    out->step_ns = g_rtc_phase_step_last_ns;
    out->phase_status = g_rtc_phase_step_last_status;
    out->phase_seq = g_rtc_phase_step_last_seq;
    out->phase_ticks = g_rtc_phase_step_last_ticks;
    out->phase_valid = g_rtc_phase_step_last_valid;
    out->reason = (g_rtc_phase_step_last_reason != (const char *)0) ? g_rtc_phase_step_last_reason : "?";
    out->confirm_dir = g_rtc_phase_step_confirm_dir;
    out->confirm_count = g_rtc_phase_step_confirm_count;
}

uint8_t app_gpsdo_get_sw_e50(int32_t *err50_out)
{
    if (err50_out == (int32_t *)0) {
        return 0U;
    }
    if (g_sw_e50_valid == 0U) {
        return 0U;
    }
    *err50_out = g_sw_e50_last;
    return 1U;
}

uint8_t app_gpsdo_get_sw_e50_valid(void)
{
    return g_sw_e50_valid;
}

int32_t app_gpsdo_get_sw_e50_last(void)
{
    return g_sw_e50_last;
}

uint8_t app_gpsdo_get_sw_e50_new(void)
{
    return g_sw_e50_new;
}

uint8_t app_gpsdo_get_sw_e50_count(void)
{
    return g_sw_e50_count;
}

uint32_t app_gpsdo_get_sw_e50_result_seq(void)
{
    return g_sw_e50_result_seq;
}

uint8_t app_gpsdo_get_sw_e100(int32_t *out) { if (out) { *out = g_sw_e100_last; } return g_sw_e100_valid; }
uint8_t app_gpsdo_get_sw_e100_valid(void) { return g_sw_e100_valid; }
int32_t app_gpsdo_get_sw_e100_last(void) { return g_sw_e100_last; }
uint8_t app_gpsdo_get_sw_e100_new(void) { return g_sw_e100_new; }
uint8_t app_gpsdo_get_sw_e100_count(void) { return g_sw_e100_count; }

uint8_t app_gpsdo_get_sw_e200(int32_t *out) { if (out) { *out = g_sw_e200_last; } return g_sw_e200_valid; }
uint8_t app_gpsdo_get_sw_e200_valid(void) { return g_sw_e200_valid; }
int32_t app_gpsdo_get_sw_e200_last(void) { return g_sw_e200_last; }
uint8_t app_gpsdo_get_sw_e200_new(void) { return g_sw_e200_new; }
uint8_t app_gpsdo_get_sw_e200_count(void) { return g_sw_e200_count; }

uint8_t app_gpsdo_get_sw_e1000(int32_t *out) { if (out) { *out = g_sw_e1000_last; } return g_sw_e1000_valid; }
uint8_t app_gpsdo_get_sw_e1000_valid(void) { return g_sw_e1000_valid; }
int32_t app_gpsdo_get_sw_e1000_last(void) { return g_sw_e1000_last; }
uint8_t app_gpsdo_get_sw_e1000_new(void) { return g_sw_e1000_new; }
uint8_t app_gpsdo_get_sw_e1000_count(void) { return g_sw_e1000_count; }

uint8_t app_gpsdo_get_disc10_applied(void) { return g_disc10_last_applied; }
int32_t app_gpsdo_get_disc10_step_code(void) { return g_disc10_last_step_code; }
const char *app_gpsdo_get_disc10_reason(void) { return (g_disc10_last_reason != (const char *)0) ? g_disc10_last_reason : "?"; }
uint8_t app_gpsdo_get_disc50_applied(void) { return g_disc50_last_applied; }
int32_t app_gpsdo_get_disc50_step_code(void) { return g_disc50_last_step_code; }
const char *app_gpsdo_get_disc50_reason(void) { return (g_disc50_last_reason != (const char *)0) ? g_disc50_last_reason : "?"; }
uint8_t app_gpsdo_get_disc50_confirm_count(void) { return g_disc50_confirm_count; }
uint32_t app_gpsdo_get_disc50_last_ms(void) { return g_disc50_last_ms; }







