#include "app_gpsdo10.h"

#include "app_config.h"
#include "app_cpld_spi.h"
#include "app_dac8571.h"
#include "app_gps_rmc.h"
#include "app_gps_uart.h"
#include "app_gpsdo.h"
#include "app_pps.h"
#include "app_timebase.h"
#include "app_tmp117.h"
#include "app_uart_debug.h"
#include "app_vct_adc.h"

#define GPSDO10_WINDOW_SEC       10U
#define GPSDO10_GPS_STAT_SEC     5U
#define GPSDO10_SCALE_4DP        10000.0f
#define PHASE10_FORCE_MS         10000UL
#define PHASE_GOOD_TIMEOUT_MS    3000UL
#define PHASE_SEQ_MAX_STEP       3UL
#define PHASE_TICK_ABS_MAX       25000000L

typedef struct {
    uint8_t temp_valid;
    float temp_cur;
    float temp_sum;
    float temp_min;
    float temp_max;
    uint16_t temp_count;

    uint8_t vct_valid;
    uint16_t vct_adc_cur;
    float vct_cur;
    float vct_sum;
    float vct_min;
    float vct_max;
    uint16_t vct_count;

    uint8_t sec_count;
    uint8_t gps_stat_sec;
    uint8_t last_printed_seq_valid;
    uint8_t last_printed_seq10;
} gpsdo10_stats_t;

typedef struct {
    uint8_t valid;
    uint16_t raw;
    int32_t vctv;
    int32_t avg;
    int32_t min;
    int32_t max;
} gpsdo10_vct_snapshot_t;

static gpsdo10_stats_t g_gpsdo10;
static uint8_t g_phase_last_seq_valid;
static uint32_t g_phase_last_seq;
static uint8_t g_phase_force_count;
static uint8_t g_phase_window_valid;
static uint32_t g_phase_seq0;
static uint32_t g_phase_ms0;
static int32_t g_phase_tick0;
static uint32_t g_phase_sample_seq;
static uint32_t g_phase_last_ms;
static int32_t g_phase_last_tick;
static uint8_t g_phase_latest_valid;
static app_cpld_phase_t g_phase_latest;
static uint32_t g_phase_latest_ms;
static app_phase10_diag_t g_phase10_last;
static uint8_t g_phase10_valid_windows;

/* Last-valid phase cache — survives invalid reads */
static uint32_t g_last_phase_seq;
static int32_t  g_last_phase_tick;
static int64_t  g_last_phase_ns;
static uint32_t g_last_phase_status;
static uint32_t g_last_phase_ms;
static uint8_t  g_last_phase_valid;

void app_gpsdo10_reset_phase_history(void)
{
    g_phase_last_seq_valid = 0U;
    g_phase_last_seq = 0UL;
    g_phase_force_count = 0U;
    g_phase_window_valid = 0U;
    g_phase_seq0 = 0UL;
    g_phase_ms0 = 0UL;
    g_phase_tick0 = 0;
    g_phase_sample_seq = 0UL;
    g_phase_last_ms = 0UL;
    g_phase_last_tick = 0;
    g_phase10_last.valid = 0U;
    g_phase10_valid_windows = 0U;
}

static uint8_t app_phase_tick_ok(int32_t ticks)
{
    return ((ticks <= PHASE_TICK_ABS_MAX) && (ticks >= -PHASE_TICK_ABS_MAX)) ? 1U : 0U;
}

static uint8_t app_phase_status_ok(uint32_t status)
{
    uint32_t need = CPLD_RTC_PHASE_VALID_MASK |
                    CPLD_RTC_PHASE_GPS_SEEN_MASK |
                    CPLD_RTC_PHASE_RTC_SEEN_MASK |
                    CPLD_RTC_PHASE_ALIVE_MASK;

    return ((status & need) == need) ? 1U : 0U;
}

static uint8_t app_phase_seq_ok(uint32_t seq)
{
    uint32_t delta;

    if (g_phase_latest_valid == 0U) {
        return 1U;
    }
    if (seq < g_phase_latest.phase_seq) {
        return 0U;
    }
    delta = seq - g_phase_latest.phase_seq;
    return (delta <= PHASE_SEQ_MAX_STEP) ? 1U : 0U;
}

static uint8_t app_phase_sample_ok(const app_cpld_phase_t *phase)
{
    if (phase == (const app_cpld_phase_t *)0) {
        return 0U;
    }
    if (app_phase_status_ok(phase->phase_status) == 0U) {
        return 0U;
    }
    if (app_phase_tick_ok(phase->phase_ticks) == 0U) {
        return 0U;
    }
    if (app_phase_seq_ok(phase->phase_seq) == 0U) {
        return 0U;
    }
    return 1U;
}

static int32_t app_gpsdo10_float_to_scaled(float value, float scale)
{
    float scaled;

    if (value >= 0.0f) {
        scaled = (value * scale) + 0.5f;
    } else {
        scaled = (value * scale) - 0.5f;
    }
    return (int32_t)scaled;
}

static void gpsdo10_get_temp_fields(int32_t *t, int32_t *tavg, int32_t *tmin, int32_t *tmax)
{
    int32_t invalid = app_gpsdo10_float_to_scaled(APP_TEMP_INVALID_C, 1000.0f);

    if ((t == (int32_t *)0) ||
        (tavg == (int32_t *)0) ||
        (tmin == (int32_t *)0) ||
        (tmax == (int32_t *)0)) {
        return;
    }

    *t = invalid;
    *tavg = invalid;
    *tmin = invalid;
    *tmax = invalid;

    if ((tmp117_is_present() == 0U) ||
        (g_gpsdo10.temp_valid == 0U) ||
        (g_gpsdo10.temp_count == 0U)) {
        return;
    }

    *t = app_gpsdo10_float_to_scaled(g_gpsdo10.temp_cur, 1000.0f);
    *tavg = app_gpsdo10_float_to_scaled(g_gpsdo10.temp_sum / (float)g_gpsdo10.temp_count, 1000.0f);
    *tmin = app_gpsdo10_float_to_scaled(g_gpsdo10.temp_min, 1000.0f);
    *tmax = app_gpsdo10_float_to_scaled(g_gpsdo10.temp_max, 1000.0f);

    if ((*tavg < -40000L) || (*tavg > 125000L) ||
        ((*t >= -40000L) && (*t <= 125000L) &&
         ((*tavg > (*t + 10000L)) || (*tavg < (*t - 10000L))))) {
        *tavg = *t;
    }
}

static void gpsdo10_get_vct_snapshot(gpsdo10_vct_snapshot_t *s)
{
    if (s == (gpsdo10_vct_snapshot_t *)0) {
        return;
    }

    s->valid = 0U;
    s->raw = 0U;
    s->vctv = app_gpsdo10_float_to_scaled(-1.0f, GPSDO10_SCALE_4DP);
    s->avg = app_gpsdo10_float_to_scaled(-1.0f, GPSDO10_SCALE_4DP);
    s->min = app_gpsdo10_float_to_scaled(-1.0f, GPSDO10_SCALE_4DP);
    s->max = app_gpsdo10_float_to_scaled(-1.0f, GPSDO10_SCALE_4DP);

    if ((g_gpsdo10.vct_valid == 0U) || (g_gpsdo10.vct_count == 0U)) {
        return;
    }

    s->valid = 1U;
    s->raw = g_gpsdo10.vct_adc_cur;
    s->vctv = app_gpsdo10_float_to_scaled(g_gpsdo10.vct_cur, GPSDO10_SCALE_4DP);
    s->avg = s->vctv;
    s->min = s->vctv;
    s->max = s->vctv;
}

static void app_gpsdo10_print_scaled(int32_t scaled, uint32_t decimals)
{
    uint32_t div = 1UL;
    uint32_t frac;
    uint8_t i;

    for (i = 0U; i < decimals; i++) {
        div *= 10UL;
    }

    if (scaled < 0) {
        app_log_write("-");
        scaled = -scaled;
    }

    frac = (uint32_t)scaled % div;
    app_log_printf("%lu.", (unsigned long)((uint32_t)scaled / div));
    if (decimals == 3U) {
        app_log_printf("%03lu", (unsigned long)frac);
    } else if (decimals == 4U) {
        app_log_printf("%04lu", (unsigned long)frac);
    } else {
        app_log_printf("%lu", (unsigned long)frac);
    }
}

static void app_gpsdo10_print_i64(int64_t value)
{
    uint64_t mag;
    uint32_t high;
    uint32_t low;

    if (value < 0) {
        app_log_write("-");
        mag = (uint64_t)(-value);
    } else {
        mag = (uint64_t)value;
    }

    high = (uint32_t)(mag / 1000000000ULL);
    low = (uint32_t)(mag % 1000000000ULL);

    if (high != 0UL) {
        app_log_printf("%lu%09lu", (unsigned long)high, (unsigned long)low);
    } else {
        app_log_printf("%lu", (unsigned long)low);
    }
}

static uint32_t app_gpsdo10_dacv_x10000(uint16_t dac_code)
{
    return (uint32_t)app_gpsdo10_float_to_scaled(((float)dac_code * DAC8571_VREF_VOLTS) / 65535.0f,
                                                 GPSDO10_SCALE_4DP);
}

static void app_gpsdo10_reset_window(void)
{
    g_gpsdo10.temp_valid = 0U;
    g_gpsdo10.temp_cur = APP_TEMP_INVALID_C;
    g_gpsdo10.temp_sum = 0.0f;
    g_gpsdo10.temp_min = APP_TEMP_INVALID_C;
    g_gpsdo10.temp_max = APP_TEMP_INVALID_C;
    g_gpsdo10.temp_count = 0U;

    g_gpsdo10.vct_valid = 0U;
    g_gpsdo10.vct_adc_cur = 0U;
    g_gpsdo10.vct_cur = -1.0f;
    g_gpsdo10.vct_sum = 0.0f;
    g_gpsdo10.vct_min = -1.0f;
    g_gpsdo10.vct_max = -1.0f;
    g_gpsdo10.vct_count = 0U;
    g_gpsdo10.sec_count = 0U;
}

static void app_gpsdo10_add_temp(float t)
{
    g_gpsdo10.temp_cur = t;
    if (g_gpsdo10.temp_count == 0U) {
        g_gpsdo10.temp_min = t;
        g_gpsdo10.temp_max = t;
    } else {
        if (t < g_gpsdo10.temp_min) {
            g_gpsdo10.temp_min = t;
        }
        if (t > g_gpsdo10.temp_max) {
            g_gpsdo10.temp_max = t;
        }
    }
    g_gpsdo10.temp_sum += t;
    g_gpsdo10.temp_count++;
    g_gpsdo10.temp_valid = 1U;
}

#if GPSDO10_USE_VCT_ADC
static void app_gpsdo10_add_vct(uint16_t raw, float v)
{
    g_gpsdo10.vct_adc_cur = raw;
    g_gpsdo10.vct_cur = v;
    if (g_gpsdo10.vct_count == 0U) {
        g_gpsdo10.vct_min = v;
        g_gpsdo10.vct_max = v;
    } else {
        if (v < g_gpsdo10.vct_min) {
            g_gpsdo10.vct_min = v;
        }
        if (v > g_gpsdo10.vct_max) {
            g_gpsdo10.vct_max = v;
        }
    }
    g_gpsdo10.vct_sum += v;
    g_gpsdo10.vct_count++;
    g_gpsdo10.vct_valid = 1U;
}
#endif

static void app_gpsdo10_print_frame(uint8_t seq10, uint32_t count10,
                                    int32_t err10, const char *state)
{
    int32_t t;
    int32_t tavg;
    int32_t tmin;
    int32_t tmax;
    gpsdo10_vct_snapshot_t vct;
    uint32_t count50 = 0UL;
    int32_t err50 = 0;
    uint16_t dac_code = app_dac8571_get_last_code();
    uint32_t dacv_x10000 = (uint32_t)app_gpsdo10_float_to_scaled(((float)dac_code * DAC8571_VREF_VOLTS) / 65535.0f,
                                                                 GPSDO10_SCALE_4DP);
    const app_rtc_state_t *rtc = app_rtc_get_state();
    app_pps_snapshot_t pps;
    uint8_t gps_valid;
    uint8_t pps_valid;

    app_pps_get_snapshot(&pps);
    gps_valid = ((rtc->rtc_valid != 0U) && (rtc->rtc_synced != 0U) && (rtc->rmc_valid != 0U)) ? 1U : 0U;
    pps_valid = (((HAL_GetTick() - pps.last_tick_ms) <= 2000UL) || (count10 != 0UL)) ? 1U : 0U;

    gpsdo10_get_temp_fields(&t, &tavg, &tmin, &tmax);
    gpsdo10_get_vct_snapshot(&vct);

    (void)app_gpsdo_get_last_disc50(&count50, &err50);

    app_log_printf("GPSDO10,MS=%lu,T=", (unsigned long)HAL_GetTick());
    app_gpsdo10_print_scaled(t, 3U);
    app_log_write(",TAVG=");
    app_gpsdo10_print_scaled(tavg, 3U);
    app_log_write(",TMIN=");
    app_gpsdo10_print_scaled(tmin, 3U);
    app_log_write(",TMAX=");
    app_gpsdo10_print_scaled(tmax, 3U);
    app_log_printf(",VCTADC=%ld,VCTV=", (long)((vct.valid != 0U) ? (int32_t)vct.raw : -1L));
    app_gpsdo10_print_scaled(vct.vctv, 4U);
    app_log_write(",VCTAVG=");
    app_gpsdo10_print_scaled(vct.avg, 4U);
    app_log_write(",VCTMIN=");
    app_gpsdo10_print_scaled(vct.min, 4U);
    app_log_write(",VCTMAX=");
    app_gpsdo10_print_scaled(vct.max, 4U);
    app_log_printf(",DAC=%lu,DACV=%lu.%04lu,SEQ=%lu,STATE=%s,GPS=%lu,PPS=%lu,C10=%lu,E10=%ld,C50=%lu,E50=%ld\r\n",
                   (unsigned long)dac_code,
                   (unsigned long)(dacv_x10000 / 10000UL),
                   (unsigned long)(dacv_x10000 % 10000UL),
                   (unsigned long)seq10,
                   (state != (const char *)0) ? state : app_gpsdo_get_state_string(),
                   (unsigned long)gps_valid,
                   (unsigned long)pps_valid,
                   (unsigned long)count10,
                   (long)err10,
                   (unsigned long)count50,
                   (long)err50);
}

static void app_gpsdo10_print_gpsdo2(uint8_t seq10, uint32_t count10,
                                     int32_t err10, const char *state)
{
    int32_t t;
    int32_t tavg;
    int32_t tmin;
    int32_t tmax;
    uint16_t dac_code = app_dac8571_get_last_code();
    uint32_t dacv_x10000 = app_gpsdo10_dacv_x10000(dac_code);
    const app_rtc_state_t *rtc = app_rtc_get_state();
    app_pps_snapshot_t pps;
    uint8_t gps_valid;
    uint8_t pps_valid;
    uint8_t phase_valid = 0U;
    uint32_t phase_seq = 0UL;
    int32_t phase_tick = 0;
    uint32_t phase_status = 0UL;
    int64_t phase_ns = 0;
    uint8_t phase10_valid = 0U;
    app_phase10_diag_t phase10;
    app_rtc_phase_step_status_t rtc_step;

    if (app_log_gpsdo2_enabled() == 0U) {
        return;
    }

    app_pps_get_snapshot(&pps);
    gps_valid = ((rtc->rtc_valid != 0U) && (rtc->rtc_synced != 0U) && (rtc->rmc_valid != 0U)) ? 1U : 0U;
    pps_valid = (((HAL_GetTick() - pps.last_tick_ms) <= 2000UL) || (count10 != 0UL)) ? 1U : 0U;
    gpsdo10_get_temp_fields(&t, &tavg, &tmin, &tmax);
    if ((g_phase_latest_valid != 0U) &&
        ((HAL_GetTick() - g_phase_latest_ms) <= PHASE_GOOD_TIMEOUT_MS) &&
        (app_phase_status_ok(g_phase_latest.phase_status) != 0U)) {
        phase_seq = g_phase_latest.phase_seq;
        phase_tick = g_phase_latest.phase_ticks;
        phase_status = g_phase_latest.phase_status;
        phase_valid = 1U;
        phase_ns = ((int64_t)phase_tick) * 20LL;
    } else {
        g_phase_latest_valid = 0U;
    }
    phase10_valid = (app_gpsdo10_get_phase10(&phase10) != 0U) ? 1U : 0U;
    if (phase10_valid == 0U) {
        phase10.valid = 0U;
        phase10.freq_err_mhz = 0;
        phase10.dtick = 0;
        phase10.dt_seconds = 0UL;
    } else {
        phase10_valid = (phase10.valid != 0U) ? 1U : 0U;
    }
    app_gpsdo_get_rtc_phase_step_status(&rtc_step);
    {
        int32_t sw_e50 = 0;
        uint8_t sw_e50_ok = app_gpsdo_get_sw_e50(&sw_e50);
        int32_t sw_e50_last = app_gpsdo_get_sw_e50_last();
        uint8_t sw_e50_new = app_gpsdo_get_sw_e50_new();
        uint8_t sw_e50_cnt = app_gpsdo_get_sw_e50_count();
        uint32_t now_ms = HAL_GetTick();

        /* When current phase invalid, fall back to last-valid cache */
        if (phase_valid == 0U && g_last_phase_valid != 0U) {
            phase_seq   = g_last_phase_seq;
            phase_tick  = g_last_phase_tick;
            phase_ns    = g_last_phase_ns;
            phase_status = g_last_phase_status;
        }

        /* GPSDO2 — core telemetry (short, no truncation) */
        app_log_printf("GPSDO2,VER=2,MS=%lu,STATE=%s,GPS=%lu,PPS=%lu,RMC=%lu,RTC_SYNC=%lu,T=",
                       (unsigned long)now_ms,
                       (state != (const char *)0) ? state : app_gpsdo_get_state_string(),
                       (unsigned long)gps_valid,
                       (unsigned long)pps_valid,
                       (unsigned long)rtc->rmc_valid,
                       (unsigned long)rtc->rtc_synced);
        app_gpsdo10_print_scaled(t, 3U);
        app_log_write(",TAVG=");
        app_gpsdo10_print_scaled(tavg, 3U);
        app_log_printf(",DAC=%lu,DACV=%lu.%04lu,SEQ10=%lu,C10=%lu,E10=%ld,SW_E50=%ld,SW_E50_VALID=%lu,SW_E50_NEW=%lu,SW_E50_COUNT=%lu,PHVALID=%lu,PHNS=",
                       (unsigned long)dac_code,
                       (unsigned long)(dacv_x10000 / 10000UL),
                       (unsigned long)(dacv_x10000 % 10000UL),
                       (unsigned long)seq10,
                       (unsigned long)count10,
                       (long)err10,
                       (long)sw_e50_last,
                       (unsigned long)sw_e50_ok,
                        (unsigned long)sw_e50_new,
                        (unsigned long)sw_e50_cnt,
                       (unsigned long)phase_valid);
        app_gpsdo10_print_i64(phase_ns);
        app_log_printf(",PHFERR=%ld,TARGET10=%lu,OFFSET=%ld,DISC_REASON=%s\r\n",
                       (long)((phase10_valid != 0U) ? phase10.freq_err_mhz : 0L),
                       (unsigned long)GPSDO_TARGET_COUNT10,
                       (long)GPSDO_TARGET_COUNT10_OFFSET,
                       app_gpsdo_get_disc_reason());

        /* GPSDO_PHASE — phase meter + RTC step detail (separate row) */
        if (app_log_gpsdo_phase_enabled() != 0U) {
            uint32_t phase_age_ms = (g_last_phase_valid != 0U) ? (now_ms - g_last_phase_ms) : 0UL;
            int64_t ph_corr_ns = phase_ns - ((int64_t)rtc_step.target_ns);
            app_log_printf("GPSDO_PHASE,MS=%lu,PHSEQ=%lu,PHTICK=%ld,PHNS=",
                           (unsigned long)now_ms,
                           (unsigned long)phase_seq,
                           (long)phase_tick);
            app_gpsdo10_print_i64(phase_ns);
            app_log_printf(",PHSTAT=0x%08lX,PHVALID=%lu,LAST_PHSEQ=%lu,LAST_PHTICK=%ld,LAST_PHNS=",
                           (unsigned long)phase_status,
                           (unsigned long)phase_valid,
                           (unsigned long)g_last_phase_seq,
                           (long)g_last_phase_tick);
            app_gpsdo10_print_i64(g_last_phase_ns);
            app_log_printf(",LAST_PHSTAT=0x%08lX,LAST_PHVALID=%lu,LAST_PHAGE_MS=%lu,PH_TARGET_NS=%ld,PH_CORR_NS=",
                           (unsigned long)g_last_phase_status,
                           (unsigned long)g_last_phase_valid,
                           (unsigned long)phase_age_ms,
                           (long)rtc_step.target_ns);
            app_gpsdo10_print_i64(ph_corr_ns);
            app_log_printf(",RTC_STEP_CNT=%ld,RTC_STEP_NS=%ld,RTC_STEP_APPLIED=%lu,RTC_STEP_REASON=%s\r\n",
                           (long)rtc_step.step_cnt,
                           (long)rtc_step.step_ns,
                           (unsigned long)rtc_step.applied,
                           (rtc_step.reason != (const char *)0) ? rtc_step.reason : "?");
        }

        /* GPSDO_AVG — long-window E10 averages */
        if (app_log_gpsdo_avg_enabled() != 0U) {
            int32_t e100  = app_gpsdo_get_sw_e100_last();
            int32_t e200  = app_gpsdo_get_sw_e200_last();
            int32_t e1000 = app_gpsdo_get_sw_e1000_last();
            app_log_printf("GPSDO_AVG,MS=%lu,SEQ10=%lu,E10=%ld,SW_E50=%ld,SW_E50_VALID=%lu,SW_E50_NEW=%lu,SW_E50_COUNT=%lu,DISC10_EN=%lu,DISC10_APPLIED=%lu,DISC10_STEP_CODE=%ld,DISC10_REASON=%s,DISC50_EN=%lu,DISC50_APPLIED=%lu,DISC50_STEP_CODE=%ld,DISC50_REASON=%s,DISC50_CONFIRM_CNT=%lu,DISC50_LAST_MS=%lu,SW_E100=%ld,SW_E100_VALID=%lu,SW_E100_NEW=%lu,SW_E100_COUNT=%lu,SW_E200=%ld,SW_E200_VALID=%lu,SW_E200_NEW=%lu,SW_E200_COUNT=%lu,SW_E1000=%ld,SW_E1000_VALID=%lu,SW_E1000_NEW=%lu,SW_E1000_COUNT=%lu,DF50_MHZ=%ld,DF100_MHZ=%ld,DF200_MHZ=%ld,DF1000_MHZ=%ld\r\n",
                           (unsigned long)now_ms,
                           (unsigned long)seq10,
                           (long)err10,
                           (long)sw_e50_last,
                           (unsigned long)sw_e50_ok,
                           (unsigned long)sw_e50_new,
                           (unsigned long)sw_e50_cnt,
                           (unsigned long)GPSDO_DISC10_CONTROL_ENABLE,
                           (unsigned long)app_gpsdo_get_disc10_applied(),
                           (long)app_gpsdo_get_disc10_step_code(),
                           app_gpsdo_get_disc10_reason(),
                           (unsigned long)GPSDO_DISC50_CONTROL_ENABLE,
                           (unsigned long)app_gpsdo_get_disc50_applied(),
                           (long)app_gpsdo_get_disc50_step_code(),
                           app_gpsdo_get_disc50_reason(),
                           (unsigned long)app_gpsdo_get_disc50_confirm_count(),
                           (unsigned long)app_gpsdo_get_disc50_last_ms(),
                           (long)e100,
                           (unsigned long)app_gpsdo_get_sw_e100_valid(),
                           (unsigned long)app_gpsdo_get_sw_e100_new(),
                           (unsigned long)app_gpsdo_get_sw_e100_count(),
                           (long)e200,
                           (unsigned long)app_gpsdo_get_sw_e200_valid(),
                           (unsigned long)app_gpsdo_get_sw_e200_new(),
                           (unsigned long)app_gpsdo_get_sw_e200_count(),
                           (long)e1000,
                           (unsigned long)app_gpsdo_get_sw_e1000_valid(),
                           (unsigned long)app_gpsdo_get_sw_e1000_new(),
                           (unsigned long)app_gpsdo_get_sw_e1000_count(),
                           (long)((sw_e50_ok != 0U) ? (sw_e50_last * 1000L / 50L) : 0L),
                           (long)((app_gpsdo_get_sw_e100_valid() != 0U) ? (e100 * 1000L / 100L) : 0L),
                           (long)((app_gpsdo_get_sw_e200_valid() != 0U) ? (e200 * 1000L / 200L) : 0L),
                           (long)((app_gpsdo_get_sw_e1000_valid() != 0U) ? (e1000 * 1000L / 1000L) : 0L));
        }
    }
}

/*
 * GPSDO_RESULT -- one fixed-format ASCII row per second holding the current
 * actuator setting and the frequency error already converted to Hz / ppb, so
 * the acceptance figures can be read straight off the terminal.
 *
 * Conversion, following README "上电验证顺序" item 5 (Hz_error = SW_E50 / 50):
 * SW_E50 sums five 10 s windows and one err10 count is 1/10 Hz, so one SW_E50
 * count is 1/50 Hz = 20 mHz of output frequency error.
 *     ERR_MHZ      = SW_E50 * 20               integer mHz, full resolution
 *     ERR_PPB_X100 = SW_E50 * 1.6276 * 100     (20 mHz / 12.288 MHz per count)
 * The README's Hz_error = SW_E50 / 50 is the same quantity in Hz, but with a
 * 0.02 Hz quantum it would truncate the interesting digits away.
 * AVG_MHZ is the 1000 s accumulator: it holds 100 windows, so one count is
 * 20 mHz / 20 = 1 mHz and the raw count is already millihertz.
 *
 * Acceptance: |ERR_MHZ| <= 40 (0.04 Hz absolute) and |AVG_MHZ| <= 20 for the
 * 0.02 Hz average figure.
 *
 * PPS_MS and SEQ10_MS are the clock plausibility check.  Both are the period of
 * a genuinely one-second / ten-second external event measured with
 * HAL_GetTick(), so on a healthy 72 MHz clock they read ~1000 and ~10000.  If
 * they deviate by the same factor then the MCU clock is not what HSE_VALUE
 * claims and the UART baud rate is wrong by that factor -- which is exactly
 * what a terminal showing garbage at 115200 looks like.
 */
static uint8_t g_result_seq10_seen;
static uint8_t g_result_seq10_last;
static uint32_t g_result_seq10_last_ms;
static uint32_t g_result_seq10_ms;

/*
 * ---------------------------------------------------------------------------
 * Report-only simulator (console command SIM).
 *
 * Purpose: the acceptance fields can only carry real data once the antenna has
 * a fix, which needs open sky.  This lets the whole REPORTING path -- field
 * layout, scaling, the 0.04 / 0.02 Hz comparison -- be verified indoors today.
 *
 * Scope is deliberately narrow and is enforced by where the values are used:
 * they substitute into the printed rows only.  They never reach the control
 * loop, never move the DAC, and every affected row carries SIM=1, so a
 * simulated record can never be mistaken for a real measurement.
 * ---------------------------------------------------------------------------
 */
static uint8_t g_sim_on;
static int32_t g_sim_sw_e50;
/*
 * Guarded so that turning GPSDO_LOG_MEAS50_ENABLE off cannot leave statics that
 * are written but never read (this project's warning baseline is zero, and a
 * new warning here would be a false alarm).
 */
#if GPSDO_LOG_MEAS50_ENABLE
static uint8_t g_sim_meas50_sec;
/* Last SW_E50 result counter already reported, and whether a baseline exists. */
static uint32_t g_meas50_seq;
static uint8_t g_meas50_seen;
#endif

void app_gpsdo10_set_sim(int32_t sw_e50_counts)
{
    if (sw_e50_counts == 0) {
        g_sim_on = 0U;
        g_sim_sw_e50 = 0;
    } else {
        g_sim_on = 1U;
        g_sim_sw_e50 = sw_e50_counts;
    }
#if GPSDO_LOG_MEAS50_ENABLE
    g_sim_meas50_sec = 0U;
#endif
}

uint8_t app_gpsdo10_get_sim(int32_t *out)
{
    if (out != (int32_t *)0) {
        *out = g_sim_sw_e50;
    }
    return g_sim_on;
}

#if GPSDO_LOG_MEAS50_ENABLE
/*
 * GPSDO_MEAS50 -- one complete record per completed 50 s window.
 *
 * This is the acceptance trail: the raw 50 s count, the error in both the
 * README's units (Hz via SW_E50/50, here scaled to mHz so the 20 mHz quantum is
 * not truncated) and ppb, the running 1000 s average, and what the controller
 * did about it -- which DAC code it moved from/to, by how much, whether the
 * write succeeded and why.
 *
 * Emitted right after app_gpsdo_poll_1s(), so the DAC fields already reflect
 * any adjustment this window caused.
 */
static void app_gpsdo10_print_meas50(void)
{
    int32_t sw_e50 = 0;
    int32_t err_mhz = 0;
    int32_t err_ppb_x100 = 0;
    int32_t avg_mhz = 0;
    uint8_t sw_ok;
    uint8_t avg_ok;
    uint32_t dac_code;
    uint32_t dacv_x10000;
    const char *reason;

    sw_ok = app_gpsdo_get_sw_e50(&sw_e50);
    avg_ok = app_gpsdo_get_sw_e1000_valid();
    if (avg_ok != 0U) {
        avg_mhz = app_gpsdo_get_sw_e1000_last();
    }

    if (g_sim_on != 0U) {
        sw_e50 = g_sim_sw_e50;
        sw_ok = 1U;
        avg_ok = 1U;
        avg_mhz = g_sim_sw_e50;
    }

    err_mhz = (int32_t)((int64_t)sw_e50 * 20LL);
    err_ppb_x100 = (int32_t)(((int64_t)sw_e50 * 16276LL) / 100LL);

    dac_code = (uint32_t)app_dac8571_get_last_code();
    dacv_x10000 = app_gpsdo10_dacv_x10000((uint16_t)dac_code);
    reason = app_gpsdo_get_disc50_reason();
    if (reason == (const char *)0) {
        reason = "?";
    }

    app_log_printf("GPSDO_MEAS50,N=%lu,MS=%lu,SW_E50=%ld,E50_VALID=%lu,ERR_MHZ=%ld,ERR_PPB_X100=%ld,AVG_MHZ=%ld,AVG_VALID=%lu,DAC=%lu,DACV=%lu.%04luV,STEP_CODE=%ld,APPLIED=%lu,REASON=%s,SIM=%lu\r\n",
                   (unsigned long)app_gpsdo_get_sw_e50_result_seq(),
                   (unsigned long)HAL_GetTick(),
                   (long)sw_e50,
                   (unsigned long)sw_ok,
                   (long)err_mhz,
                   (long)err_ppb_x100,
                   (long)avg_mhz,
                   (unsigned long)avg_ok,
                   (unsigned long)dac_code,
                   (unsigned long)(dacv_x10000 / 10000UL),
                   (unsigned long)(dacv_x10000 % 10000UL),
                   (long)app_gpsdo_get_last_disc_step_code(),
                   (unsigned long)app_gpsdo_get_disc50_applied(),
                   reason,
                   (unsigned long)g_sim_on);
}
#endif /* GPSDO_LOG_MEAS50_ENABLE */

/*
 * EFC? -- how much frequency-correction travel is actually left.
 *
 * Turns an invisible constraint into a number.  The VCTCXO can only be pulled
 * inside APP_DAC_EFC_CODE_MIN..APP_DAC_EFC_CODE_MAX, and the park point is not
 * centred in that window, so the travel available in each direction differs --
 * and it is finite.  Expressed in SW_E50 counts it can be compared directly
 * against the SW_E50 column of GPSDO_RESULT: if |SW_E50| is bigger than the
 * matching budget, the loop saturates and can never reach nominal no matter how
 * the control law is tuned.  That failure looks exactly like "the loop does not
 * work", so it is worth being able to read it off in one command.
 */
void app_gpsdo10_print_efc_budget(void)
{
    uint32_t code = (uint32_t)app_dac8571_get_last_code();
    uint32_t code_min = APP_DAC_EFC_CODE_MIN;
    uint32_t code_max = APP_DAC_EFC_CODE_MAX;
    uint32_t v_code;
    uint32_t v_min;
    uint32_t v_max;
    uint32_t up_lsb;
    uint32_t dn_lsb;
    int64_t uhz_per_lsb;
    int64_t up_mhz;
    int64_t dn_mhz;

    /*
     * uHz per LSB = slope[Hz/V] * (DAC_VREF_UV / 65536)[uV/LSB]
     * (the 1e6 uHz/Hz and the 1e6 uV/V cancel).
     * With the default slope 100 and VREF 3.36e6: 5127 uHz = 5.127 mHz.
     */
    uhz_per_lsb = ((int64_t)APP_VCTCXO_SLOPE_HZ_PER_V * (int64_t)DAC_VREF_UV) / 65536LL;

    if (code > code_max) {
        code = code_max;
    }
    if (code < code_min) {
        code = code_min;
    }
    up_lsb = (uint32_t)(code_max - code);
    dn_lsb = (uint32_t)(code - code_min);
    up_mhz = (uhz_per_lsb * (int64_t)up_lsb) / 1000LL;
    dn_mhz = (uhz_per_lsb * (int64_t)dn_lsb) / 1000LL;

    v_code = app_gpsdo10_dacv_x10000((uint16_t)code);
    v_min = app_gpsdo10_dacv_x10000((uint16_t)code_min);
    v_max = app_gpsdo10_dacv_x10000((uint16_t)code_max);

    app_log_printf("EFC,CODE=%lu,V=%lu.%04luV,CODE_MIN=%lu,V_MIN=%lu.%04luV,CODE_MAX=%lu,V_MAX=%lu.%04luV\r\n",
                   (unsigned long)code,
                   (unsigned long)(v_code / 10000UL),
                   (unsigned long)(v_code % 10000UL),
                   (unsigned long)code_min,
                   (unsigned long)(v_min / 10000UL),
                   (unsigned long)(v_min % 10000UL),
                   (unsigned long)code_max,
                   (unsigned long)(v_max / 10000UL),
                   (unsigned long)(v_max % 10000UL));

    app_log_printf("EFC_PULL,UP_LSB=%lu,UP_MHZ=%ld,UP_E50_MAX=%ld,DOWN_LSB=%lu,DOWN_MHZ=%ld,DOWN_E50_MAX=%ld,UHZ_PER_LSB=%ld\r\n",
                   (unsigned long)up_lsb,
                   (long)up_mhz,
                   (long)(up_mhz / 20LL),
                   (unsigned long)dn_lsb,
                   (long)dn_mhz,
                   (long)(dn_mhz / 20LL),
                   (long)uhz_per_lsb);

    app_log_write("EFC_HINT,UP_E50_MAX/DOWN_E50_MAX are the largest |SW_E50| the loop can still reach; a bigger SW_E50 means it saturates short of nominal. Budgets scale with APP_VCTCXO_SLOPE_HZ_PER_V, which is the ASSUMED 100 Hz/V until measured.\r\n");
}

void app_gpsdo10_print_result(void)
{
    uint8_t seq10 = 0U;
    uint32_t count10 = 0UL;
    int32_t err10 = 0;
    const char *state_snapshot = (const char *)0;
    uint8_t snap_ok;
    uint16_t dac_code;
    uint32_t dacv_x10000;
    int32_t sw_e50_now = 0;
    int32_t sw_e50_last;
    uint8_t sw_e50_ok;
    int32_t err_mhz = 0;
    int32_t err_ppb_x100 = 0;
    int32_t avg_mhz = 0;
    uint8_t avg_valid;
    app_pps_snapshot_t pps;
    uint32_t now_ms;
    uint8_t pps_ok;

    if (app_log_result_enabled() == 0U) {
        return;
    }

    now_ms = HAL_GetTick();
    snap_ok = app_gpsdo_get_last_disc10_snapshot(&seq10, &count10, &err10, &state_snapshot);
    (void)state_snapshot;

    /* Track how often a new 10 s window really arrives, in local milliseconds. */
    if ((snap_ok != 0U) && ((g_result_seq10_seen == 0U) || (seq10 != g_result_seq10_last))) {
        if (g_result_seq10_seen != 0U) {
            g_result_seq10_ms = now_ms - g_result_seq10_last_ms;
        }
        g_result_seq10_last = seq10;
        g_result_seq10_last_ms = now_ms;
        g_result_seq10_seen = 1U;
    }

    dac_code = app_dac8571_get_last_code();
    dacv_x10000 = app_gpsdo10_dacv_x10000(dac_code);
    sw_e50_ok = app_gpsdo_get_sw_e50(&sw_e50_now);
    sw_e50_last = app_gpsdo_get_sw_e50_last();

    if (sw_e50_ok != 0U) {
        /* Purely an overflow guard: SW_E50 is bounded by 5 windows of
         * DAC_ERR10_ABS_MAX_VALID so it can never reach this, and the real
         * pull range is only +-123 Hz anyway. */
        int32_t sw = sw_e50_last;
        if (sw > 100000L) {
            sw = 100000L;
        } else if (sw < -100000L) {
            sw = -100000L;
        }
        err_mhz = sw * 20L;
        err_ppb_x100 = (int32_t)(((int64_t)sw * 16276LL) / 100LL);
    }

    avg_valid = app_gpsdo_get_sw_e1000_valid();
    if (avg_valid != 0U) {
        avg_mhz = app_gpsdo_get_sw_e1000_last();
    }

    /*
     * Report-only substitution; see the block comment on g_sim_on.  Both the
     * 50 s error and the running average take the injected count, so a single
     * command exercises every acceptance field.
     */
    if (g_sim_on != 0U) {
        sw_e50_last = g_sim_sw_e50;
        sw_e50_ok = 1U;
        snap_ok = 1U;
        err_mhz = (int32_t)((int64_t)g_sim_sw_e50 * 20LL);
        err_ppb_x100 = (int32_t)(((int64_t)g_sim_sw_e50 * 16276LL) / 100LL);
        avg_mhz = g_sim_sw_e50;
        avg_valid = 1U;
    }

    app_pps_get_snapshot(&pps);
    pps_ok = (((now_ms - pps.last_tick_ms) <= 2000UL) || (count10 != 0UL)) ? 1U : 0U;

    /*
     * The acceptance row: current actuator setting + measured frequency error.
     * Only acceptance-relevant fields live here.  The clock-check pair
     * (CLK_K_X10000 / CLK_FRAMES) was moved out once the 12.288 MHz / TIM3
     * investigation closed -- it is still available on demand via CLK? -- so
     * this row stays short enough not to be wrapped and misread by a terminal.
     */
    app_log_printf("GPSDO_RESULT,MS=%lu,STATE=%s,MEAS=%lu,DAC=%lu,DACV=%lu.%04luV,C10=%lu,E10=%ld,SW_E50=%ld,E50_VALID=%lu,ERR_MHZ=%ld,ERR_PPB_X100=%ld,AVG_MHZ=%ld,AVG_VALID=%lu,PPS_MS=%lu,PPS_CNT=%lu,PPS_OK=%lu,SEQ10_MS=%lu,DROPPED=%lu,SIM=%lu\r\n",
                   (unsigned long)now_ms,
                   app_gpsdo_get_state_string(),
                   (unsigned long)snap_ok,
                   (unsigned long)dac_code,
                   (unsigned long)(dacv_x10000 / 10000UL),
                   (unsigned long)(dacv_x10000 % 10000UL),
                   (unsigned long)count10,
                   (long)err10,
                   (long)sw_e50_last,
                   (unsigned long)sw_e50_ok,
                   (long)err_mhz,
                   (long)err_ppb_x100,
                   (long)avg_mhz,
                   (unsigned long)avg_valid,
                   (unsigned long)pps.interval_ms,
                   (unsigned long)pps.count,
                   (unsigned long)pps_ok,
                   (unsigned long)g_result_seq10_ms,
                   (unsigned long)app_log_get_tx_drops(),
                   (unsigned long)g_sim_on);
}

/*
 * GPS_STAT -- why the discipline loop is not progressing.
 *
 * The CPLD latches COUNT10 only after ten GPS PPS intervals, so with no PPS
 * there are no windows, COUNT10 stays 0 and the state machine can never leave
 * WARM.  This row separates the possible causes instead of leaving them as a
 * single "GPS=0":
 *
 *   RX_BYTES = 0                     -> the module is not talking at all:
 *                                       power, wiring, or USART2 not at 38400
 *   RX_BYTES > 0 but RMC_LINES = 0   -> bytes arrive but no framed NMEA:
 *                                       wrong baud, inverted levels, noise
 *   RMC_LINES > 0 but RMC_VALID = 0  -> NMEA is healthy, there is simply no
 *                                       position fix yet (sky view / antenna)
 *   PPS_CNT = 0                      -> no TIMEPULSE edge ever reached PA1;
 *                                       u-blox does not pulse before it has a
 *                                       fix, so check PA1 wiring only once
 *                                       RMC_VALID is 1
 */
void app_gpsdo10_print_gps_stat(void)
{
    app_gps_uart_stats_t s;
    app_pps_snapshot_t pps;
    app_gps_sky_stats_t sky;
    const app_rtc_state_t *rtc = app_rtc_get_state();
    uint32_t now_ms = HAL_GetTick();
    uint32_t pps_age_ms = 0xFFFFFFFFUL;
    uint32_t rmc_age_ms = 0xFFFFFFFFUL;
    uint32_t now_t3;
    uint32_t d_ms;
    uint32_t d_t3;
    uint32_t tim3_per_ticksec = 0U;
    const char *fix_str;
    const char *verdict;
    static uint32_t last_ms;
    static uint32_t last_t3;
    /*
     * Stamp both rows of this 5 s group with the same sequence number.  This
     * project leans hard on its own telemetry, so the telemetry has to be able
     * to prove it is not silently losing rows: if GPS_RATE ever goes missing
     * while GPS_SKY carries N=7 and the previous group carried N=5, the loss is
     * visible instead of being taken for "nothing to report".
     */
    static uint32_t gps_stat_seq;

    app_gps_uart_get_stats(&s);
    app_pps_get_snapshot(&pps);

    /*
     * Cross-check the two timebases against each other.  app_timebase counts
     * TIM3 interrupts, HAL_GetTick() counts SysTick; they are independent
     * hardware, so on a correct 100 Hz tick this reads 100.  A large value
     * means TIM3 runs fast (an overflowing prescaler), a small one means the
     * Tick is running slow.  Needs no GPS and no scope.
     */
    now_t3 = app_timebase_get_tick_count();
    d_ms = now_ms - last_ms;
    d_t3 = now_t3 - last_t3;
    if (d_ms != 0U) {
        tim3_per_ticksec = (uint32_t)(((uint64_t)d_t3 * 1000ULL) / (uint64_t)d_ms);
    }
    last_ms = now_ms;
    last_t3 = now_t3;

    if (pps.count != 0UL) {
        pps_age_ms = now_ms - pps.last_tick_ms;
    }
    if (s.last_rmc_tick_ms != 0UL) {
        rmc_age_ms = now_ms - s.last_rmc_tick_ms;
    }

    app_log_printf("GPS_STAT,MS=%lu,RX_BYTES=%lu,NMEA_LINES=%lu,RMC_LINES=%lu,DOLLAR=%lu,OVERFLOW=%lu,RMC_AGE_MS=%lu,CK_OK=%lu,CK_FAIL=%lu,PARSE_FAIL=%lu,RMC_VALID=%lu,RTC_VALID=%lu,RTC_SYNC=%lu,BJ=%02lu:%02lu:%02lu,PPS_CNT=%lu,PPS_INTERVAL_MS=%lu,PPS_INVALID=%lu,PPS_AGE_MS=%lu,TIM3_PER_TICKSEC=%lu,RXERR=%lu\r\n",
                   (unsigned long)now_ms,
                   (unsigned long)s.rx_bytes,
                   (unsigned long)s.nmea_lines,
                   (unsigned long)s.rmc_lines,
                   (unsigned long)s.dollar_count,
                   (unsigned long)s.overflow_lines,
                   (unsigned long)rmc_age_ms,
                   (unsigned long)app_rmc_get_checksum_ok(),
                   (unsigned long)app_rmc_get_checksum_fail(),
                   (unsigned long)app_rmc_get_parse_fail(),
                   (unsigned long)rtc->rmc_valid,
                   (unsigned long)rtc->rtc_valid,
                   (unsigned long)rtc->rtc_synced,
                   (unsigned long)rtc->bj.utc_hour,
                   (unsigned long)rtc->bj.utc_min,
                   (unsigned long)rtc->bj.utc_sec,
                   (unsigned long)pps.count,
                   (unsigned long)pps.interval_ms,
                   (unsigned long)pps.invalid_interval_count,
                   (unsigned long)pps_age_ms,
                   (unsigned long)tim3_per_ticksec,
                   (unsigned long)s.uart_err);

    /*
     * Two short extra rows.  Kept deliberately under ~58 characters each: the
     * terminal wraps around there, and a wrapped row is how "PARSE_FAIL=105"
     * got misread as "0" earlier in this project.
     *
     * VERDICT tokens:
     *   NO_NMEA   no RMC has ever arrived -> wiring / power / wrong UART
     *   FIXED     time solution valid; PPS should follow shortly
     *   HAS_SATS  satellites are tracked but there is no fix yet -> needs open
     *             sky / more time, nothing to repair
     *   NO_GSV    RMC arrives but no GSV was ever seen, so SATS==0 says nothing
     *             about the antenna.  Look at GPS_RATE and NMEA?.
     *   NO_RF     GSV is arriving and reports zero satellites -> the receiver
     *             hears nothing on 1575 MHz (antenna not on the RF connector,
     *             active antenna with no bias feed, or a dead antenna).
     *             Waiting will never fix this.
     *
     * SATS / SNR are maxima folded over all GSV sentences of the last second, so
     * the per-constellation groups of a multi-GNSS receiver cannot overwrite
     * each other.  SNR_BEST is latched for the whole run: it says the RF path
     * demonstrably worked at least once, even if the current second reads zero.
     *
     * GPS_RATE counts by sentence TYPE, not by talker, so a receiver answering
     * with $GBRMC / $BDRMC instead of $GNRMC still shows up (TLK shows which).
     * PER is the navigation epoch period: exactly one RMC per fix, so it is also
     * the rate the TIMEPULSE will come out at.  If PER is not ~1000 the PPS will
     * not be 1 Hz, and both app_pps.c (900..1100 ms) and the CPLD's 10-interval
     * window will reject it.
     */
    app_gps_get_sky_stats(&sky);
    gps_stat_seq++;

    fix_str = "?";
    if (sky.rmc_fix == 'A') {
        fix_str = "A";
    } else if (sky.rmc_fix == 'V') {
        fix_str = "V";
    }

    if (sky.rmc_count == 0UL) {
        verdict = "NO_NMEA";
    } else if (rtc->rmc_valid != 0U) {
        verdict = "FIXED";
    } else if ((sky.ever_had_sats != 0U) || (sky.sats_in_view != 0U)) {
        verdict = "HAS_SATS";
    } else if (sky.gsv_count == 0UL) {
        verdict = "NO_GSV";
    } else {
        verdict = "NO_RF";
    }

    app_log_printf("GPS_SKY,FIX=%s,SATS=%lu,SNR=%lu,BEST=%lu,N=%lu,VERDICT=%s\r\n",
                   fix_str,
                   (unsigned long)sky.sats_in_view,
                   (unsigned long)sky.snr_max,
                   (unsigned long)sky.snr_best,
                   (unsigned long)gps_stat_seq,
                   verdict);

    app_log_printf("GPS_RATE,N=%lu,TLK=%s,RMC=%lu,PER=%lu,GSV=%lu,GGA=%lu\r\n",
                   (unsigned long)gps_stat_seq,
                   sky.rmc_talker,
                   (unsigned long)sky.rmc_count,
                   (unsigned long)sky.rmc_period_ms,
                   (unsigned long)sky.gsv_count,
                   (unsigned long)sky.gga_count);
}

static void app_gpsdo10_phase10_print(const app_phase10_diag_t *d)
{
    if ((d == (const app_phase10_diag_t *)0) || (d->valid == 0U)) {
        app_log_write("PHASE10,VALID=0\r\n");
        return;
    }

    app_log_printf("PHASE10,MS=%lu,SEQ0=%lu,SEQ1=%lu,DT=%lu,TICK0=%ld,TICK1=%ld,DTICK=%ld,SLOPE=%ld,FERR_MHZ=%ld,E10=%ld,C10=%lu,DAC=%lu,DACV=%lu.%04lu\r\n",
                   (unsigned long)d->ms1,
                   (unsigned long)d->seq0,
                   (unsigned long)d->seq1,
                   (unsigned long)d->dt_seconds,
                   (long)d->tick0,
                   (long)d->tick1,
                   (long)d->dtick,
                   (long)d->slope_x1000,
                   (long)d->freq_err_mhz,
                   (long)((d->disc10_valid != 0U) ? d->err10 : 0L),
                   (unsigned long)((d->disc10_valid != 0U) ? d->count10 : 0UL),
                   (unsigned long)d->dac_code,
                   (unsigned long)(d->dacv_x10000 / 10000UL),
                   (unsigned long)(d->dacv_x10000 % 10000UL));
}

static void app_gpsdo10_phase10_update(const app_cpld_phase_t *phase, uint32_t ms)
{
    uint32_t seq_delta;
    uint32_t ms_delta;
    uint32_t dt_seconds;
    int64_t dtick64;
    int64_t slope_x1000;
    int64_t freq_err_mhz;
    uint8_t seq10 = 0U;
    const char *state_snapshot = (const char *)0;

    if (phase == (const app_cpld_phase_t *)0) {
        return;
    }
    if (app_phase_sample_ok(phase) == 0U) {
        app_gpsdo10_reset_phase_history();
        return;
    }

    g_phase_sample_seq = phase->phase_seq;
    g_phase_last_ms = ms;
    g_phase_last_tick = phase->phase_ticks;

    if (g_phase_window_valid == 0U) {
        g_phase_window_valid = 1U;
        g_phase_seq0 = phase->phase_seq;
        g_phase_ms0 = ms;
        g_phase_tick0 = phase->phase_ticks;
        return;
    }

    seq_delta = phase->phase_seq - g_phase_seq0;
    ms_delta = ms - g_phase_ms0;
    if ((seq_delta < 10UL) && (ms_delta < PHASE10_FORCE_MS)) {
        return;
    }

    dt_seconds = (seq_delta != 0UL) ? seq_delta : ((ms_delta + 500UL) / 1000UL);
    if (dt_seconds == 0UL) {
        return;
    }

    dtick64 = (int64_t)phase->phase_ticks - (int64_t)g_phase_tick0;
    slope_x1000 = (dtick64 * 1000LL) / (int64_t)dt_seconds;
    freq_err_mhz = -((dtick64 * 245760LL) / ((int64_t)dt_seconds * 1000LL));

    if ((dt_seconds < 9UL) || (dt_seconds > 11UL) ||
        (dtick64 > 1000LL) || (dtick64 < -1000LL)) {
        app_gpsdo10_reset_phase_history();
        g_phase_window_valid = 1U;
        g_phase_seq0 = phase->phase_seq;
        g_phase_ms0 = ms;
        g_phase_tick0 = phase->phase_ticks;
        g_phase_sample_seq = phase->phase_seq;
        g_phase_last_ms = ms;
        g_phase_last_tick = phase->phase_ticks;
        return;
    }
    g_phase10_last.valid = 1U;
    g_phase10_last.seq0 = g_phase_seq0;
    g_phase10_last.seq1 = phase->phase_seq;
    g_phase10_last.ms0 = g_phase_ms0;
    g_phase10_last.ms1 = ms;
    g_phase10_last.tick0 = g_phase_tick0;
    g_phase10_last.tick1 = phase->phase_ticks;
    g_phase10_last.dt_seconds = dt_seconds;
    g_phase10_last.dtick = (int32_t)dtick64;
    g_phase10_last.slope_x1000 = (int32_t)slope_x1000;
    g_phase10_last.freq_err_mhz = (int32_t)freq_err_mhz;
    g_phase10_last.disc10_valid = app_gpsdo_get_last_disc10_snapshot(&seq10,
                                                                      &g_phase10_last.count10,
                                                                      &g_phase10_last.err10,
                                                                      &state_snapshot);
    (void)seq10;
    (void)state_snapshot;
    g_phase10_last.dac_code = app_dac8571_get_last_code();
    g_phase10_last.dacv_x10000 = app_gpsdo10_dacv_x10000(g_phase10_last.dac_code);
    if (g_phase10_valid_windows < 255U) {
        g_phase10_valid_windows++;
    }
    g_phase10_last.valid_windows = g_phase10_valid_windows;

    if (app_log_phase10_enabled() != 0U) {
        app_gpsdo10_phase10_print(&g_phase10_last);
    }

    g_phase_seq0 = phase->phase_seq;
    g_phase_ms0 = ms;
    g_phase_tick0 = phase->phase_ticks;
}

static void app_gpsdo10_poll_phase_log(void)
{
    app_cpld_phase_t phase;
    HAL_StatusTypeDef st;
    int64_t ns;
    uint32_t ms;
    uint8_t print_now = 0U;

    ms = HAL_GetTick();
    st = app_cpld_read_phase(&phase);
    if (st != HAL_OK) {
        app_gpsdo10_reset_phase_history();
        if (app_log_rtc_phase_enabled() != 0U) {
            app_log_printf("RTC_PHASE,MS=%lu,READ=FAIL\r\n", (unsigned long)ms);
        }
        return;
    }

    if (app_phase_sample_ok(&phase) == 0U) {
        app_gpsdo10_reset_phase_history();
        return;
    }
    g_phase_latest = phase;
    g_phase_latest_ms = ms;
    g_phase_latest_valid = 1U;
    g_last_phase_seq = phase.phase_seq;
    g_last_phase_tick = phase.phase_ticks;
    g_last_phase_ns = ((int64_t)phase.phase_ticks) * 20LL;
    g_last_phase_status = phase.phase_status;
    g_last_phase_ms = ms;
    g_last_phase_valid = 1U;
    app_gpsdo10_phase10_update(&phase, ms);

    if ((g_phase_last_seq_valid == 0U) || (phase.phase_seq != g_phase_last_seq)) {
        print_now = 1U;
    } else {
        g_phase_force_count++;
        if (g_phase_force_count >= 10U) {
            print_now = 1U;
        }
    }

    if (print_now == 0U) {
        return;
    }

    g_phase_last_seq_valid = 1U;
    g_phase_last_seq = phase.phase_seq;
    g_phase_force_count = 0U;
    ns = ((int64_t)phase.phase_ticks) * 20LL;

    if (app_log_rtc_phase_enabled() != 0U) {
        app_log_printf("RTC_PHASE,MS=%lu,SEQ=%lu,TICK=%ld,NS=",
                       (unsigned long)ms,
                       (unsigned long)phase.phase_seq,
                       (long)phase.phase_ticks);
        app_gpsdo10_print_i64(ns);
        app_log_printf(",VALID=%lu,STATUS=0x%08lX,ALIVE=%lu\r\n",
                       (unsigned long)((phase.phase_status & CPLD_RTC_PHASE_VALID_MASK) ? 1UL : 0UL),
                       (unsigned long)phase.phase_status,
                       (unsigned long)((phase.phase_status & CPLD_RTC_PHASE_ALIVE_MASK) ? 1UL : 0UL));
    }
}

void app_gpsdo10_init(void)
{
    g_gpsdo10.last_printed_seq_valid = 0U;
    g_gpsdo10.last_printed_seq10 = 0U;
    g_phase_latest_valid = 0U;
    g_phase_latest_ms = 0UL;
    app_gpsdo10_reset_phase_history();
    app_gpsdo10_reset_window();
}

void app_gpsdo10_poll_1s(void)
{
    HAL_StatusTypeDef st;
    float temp_c;
    uint16_t raw_avg;
    float vct_v;
    uint8_t seq10 = 0U;
    uint32_t count10 = 0UL;
    int32_t err10 = 0;
    const char *state_snapshot = (const char *)0;

    if (tmp117_is_present() != 0U) {
        st = tmp117_read_celsius(&temp_c);
        if (st == HAL_OK) {
            app_gpsdo10_add_temp(temp_c);
            if ((g_gpsdo10.last_printed_seq_valid == 0U) && (g_gpsdo10.sec_count == 0U)) {
                app_log_write("TMP117_READ status=0 T=");
                app_gpsdo10_print_scaled(app_gpsdo10_float_to_scaled(temp_c, 1000.0f), 3U);
                app_log_write("\r\n");
            }
        } else {
            app_log_printf("TMP117_READ_FAIL status=%lu\r\n", (unsigned long)st);
        }
    }

#if GPSDO10_USE_VCT_ADC
    st = vct_adc_read_average(32U, &raw_avg, &vct_v);
    if (st == HAL_OK) {
        app_gpsdo10_add_vct(raw_avg, vct_v);
    } else {
        app_log_printf("VCT_ADC_READ_FAIL status=%lu\r\n", (unsigned long)st);
    }
#else
    (void)st;
    (void)raw_avg;
    (void)vct_v;
#endif

    g_gpsdo10.sec_count++;

    if ((app_gpsdo_get_last_disc10_snapshot(&seq10, &count10, &err10, &state_snapshot) != 0U) &&
        ((g_gpsdo10.last_printed_seq_valid == 0U) || (seq10 != g_gpsdo10.last_printed_seq10))) {
        app_gpsdo10_print_gpsdo2(seq10, count10, err10, state_snapshot);
        if (app_log_legacy_gpsdo10_enabled() != 0U) {
            app_gpsdo10_print_frame(seq10, count10, err10, state_snapshot);
        }
        g_gpsdo10.last_printed_seq_valid = 1U;
        g_gpsdo10.last_printed_seq10 = seq10;
        app_gpsdo10_reset_window();
    } else if (g_gpsdo10.sec_count >= GPSDO10_WINDOW_SEC) {
        g_gpsdo10.sec_count = GPSDO10_WINDOW_SEC;
    }

    /* Steady 1 Hz readout of the current value and the error, independent of
     * whether a new 10 s window has landed yet (the fields then simply repeat
     * the previous measurement; MEAS says whether one exists at all). */
    app_gpsdo10_print_result();

#if GPSDO_LOG_MEAS50_ENABLE
    /*
     * One complete record per completed 50 s window.  Triggered from a
     * monotonic result counter rather than a take/clear flag, and emitted here
     * -- i.e. after app_gpsdo_poll_1s() has run for this second -- so the DAC
     * fields already show any adjustment this window caused.
     */
    {
        uint32_t seq = app_gpsdo_get_sw_e50_result_seq();
        if ((g_meas50_seen == 0U) || (seq != g_meas50_seq)) {
            if (g_meas50_seen != 0U) {
                app_gpsdo10_print_meas50();
            }
            g_meas50_seq = seq;
            g_meas50_seen = 1U;
        }
    }

    /* With the simulator on there is no real window to trigger on, so pace the
     * same record off a plain 50 s divider and let it carry SIM=1. */
    if (g_sim_on != 0U) {
        g_sim_meas50_sec++;
        if (g_sim_meas50_sec >= GPSDO_MEAS50_PERIOD_SEC) {
            g_sim_meas50_sec = 0U;
            app_gpsdo10_print_meas50();
        }
    }
#endif

    /* Every 5 s, say whether the GPS/PPS chain that feeds the discipline loop
     * is actually alive.  Without PPS there are no CPLD windows at all, so
     * this is the first thing to look at when STATE stays WARM. */
    g_gpsdo10.gps_stat_sec++;
    if (g_gpsdo10.gps_stat_sec >= GPSDO10_GPS_STAT_SEC) {
        g_gpsdo10.gps_stat_sec = 0U;
        app_gpsdo10_print_gps_stat();
    }

    app_gpsdo10_poll_phase_log();
}

void app_gpsdo10_print_phase_stat(void)
{
    app_gpsdo10_phase10_print(&g_phase10_last);
    if (g_phase_window_valid != 0U) {
        app_log_printf("PHASESTAT_WINDOW,SEQ0=%lu,TICK0=%ld,MS0=%lu,LAST_SEQ=%lu,LAST_TICK=%ld,LAST_MS=%lu\r\n",
                       (unsigned long)g_phase_seq0,
                       (long)g_phase_tick0,
                       (unsigned long)g_phase_ms0,
                       (unsigned long)g_phase_sample_seq,
                       (long)g_phase_last_tick,
                       (unsigned long)g_phase_last_ms);
    }
}

uint8_t app_gpsdo10_get_phase10(app_phase10_diag_t *out)
{
    if ((out == (app_phase10_diag_t *)0) || (g_phase10_last.valid == 0U)) {
        return 0U;
    }
    if ((g_phase10_last.dt_seconds < 9UL) || (g_phase10_last.dt_seconds > 11UL) ||
        (g_phase10_last.dtick > 1000L) || (g_phase10_last.dtick < -1000L)) {
        return 0U;
    }

    *out = g_phase10_last;
    return 1U;
}

uint8_t app_gpsdo10_phase10_is_valid(void)
{
    if ((g_phase10_last.valid == 0U) ||
        (g_phase10_last.dt_seconds < 9UL) || (g_phase10_last.dt_seconds > 11UL) ||
        (g_phase10_last.dtick > 1000L) || (g_phase10_last.dtick < -1000L)) {
        return 0U;
    }
    return 1U;
}
