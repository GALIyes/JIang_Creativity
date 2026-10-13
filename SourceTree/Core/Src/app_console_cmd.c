#include "app_console_cmd.h"

#include "app_cpld_spi.h"
#include "app_dac8571.h"
#include "app_gpsdo.h"
#include "app_gpsdo10.h"
#include "app_uart_debug.h"
#include "app_vct_adc.h"

#include <string.h>

#define CONSOLE_LINE_LEN 64U
#define DAC_MAX_UV 3300000UL

static char g_console_line[CONSOLE_LINE_LEN];
static uint8_t g_console_len;

static uint8_t app_is_digit(char c)
{
    return ((c >= '0') && (c <= '9')) ? 1U : 0U;
}

static char app_to_upper(char c)
{
    if ((c >= 'a') && (c <= 'z')) {
        return (char)(c - ('a' - 'A'));
    }

    return c;
}

static uint8_t app_starts_with_help(const char *line)
{
    return ((app_to_upper(line[0]) == 'H') &&
            (app_to_upper(line[1]) == 'E') &&
            (app_to_upper(line[2]) == 'L') &&
            (app_to_upper(line[3]) == 'P') &&
            (line[4] == '\0')) ? 1U : 0U;
}

static void app_console_print_help(void)
{
    app_log_write("HELP:\r\n");
    app_log_write("  DACx.xxxxV  - set DAC manual mode\r\n");
    app_log_write("  DACx.xxxxVD - set DAC, clear windows, restart WARM dry-run\r\n");
    app_log_write("  SPIID?      - read SIGNATURE/STATUS/DEBUG_CNT/WINDOW_SEQ\r\n");
    app_log_write("  PHASE?      - Read RTC_PPS phase meter registers\r\n");
    app_log_write("  PHASESTAT?  - print last PHASE10 diagnostic\r\n");
    app_log_write("  LOG?        - print log switches\r\n");
    app_log_write("  LOGPHASE 0/1, LOGRTC 0/1, LOGP10 0/1\r\n");
    app_log_write("  LOGLEGACY 0/1, LOGEVENT 0/1, LOGDAC 0/1\r\n");
    app_log_write("  SPITEST?    - stress read SPI registers\r\n");
    app_log_write("  ADC?        - read PA0/ADC1_IN0 VCT diagnostics\r\n");
}

static uint8_t app_is_spiid_query(const char *line)
{
    return ((app_to_upper(line[0]) == 'S') &&
            (app_to_upper(line[1]) == 'P') &&
            (app_to_upper(line[2]) == 'I') &&
            (app_to_upper(line[3]) == 'I') &&
            (app_to_upper(line[4]) == 'D') &&
            (line[5] == '?') &&
            (line[6] == '\0')) ? 1U : 0U;
}

static uint8_t app_is_spitest_query(const char *line)
{
    return ((app_to_upper(line[0]) == 'S') &&
            (app_to_upper(line[1]) == 'P') &&
            (app_to_upper(line[2]) == 'I') &&
            (app_to_upper(line[3]) == 'T') &&
            (app_to_upper(line[4]) == 'E') &&
            (app_to_upper(line[5]) == 'S') &&
            (app_to_upper(line[6]) == 'T') &&
            (line[7] == '?') &&
            (line[8] == '\0')) ? 1U : 0U;
}

static uint8_t app_is_phase_query(const char *line)
{
    return ((app_to_upper(line[0]) == 'P') &&
            (app_to_upper(line[1]) == 'H') &&
            (app_to_upper(line[2]) == 'A') &&
            (app_to_upper(line[3]) == 'S') &&
            (app_to_upper(line[4]) == 'E') &&
            (line[5] == '?') &&
            (line[6] == '\0')) ? 1U : 0U;
}

static uint8_t app_is_phasestat_query(const char *line)
{
    return ((app_to_upper(line[0]) == 'P') &&
            (app_to_upper(line[1]) == 'H') &&
            (app_to_upper(line[2]) == 'A') &&
            (app_to_upper(line[3]) == 'S') &&
            (app_to_upper(line[4]) == 'E') &&
            (app_to_upper(line[5]) == 'S') &&
            (app_to_upper(line[6]) == 'T') &&
            (app_to_upper(line[7]) == 'A') &&
            (app_to_upper(line[8]) == 'T') &&
            (line[9] == '?') &&
            (line[10] == '\0')) ? 1U : 0U;
}

static uint8_t app_is_adc_query(const char *line)
{
    return ((app_to_upper(line[0]) == 'A') &&
            (app_to_upper(line[1]) == 'D') &&
            (app_to_upper(line[2]) == 'C') &&
            (line[3] == '?') &&
            (line[4] == '\0')) ? 1U : 0U;
}

static uint8_t app_is_log_query(const char *line)
{
    return ((app_to_upper(line[0]) == 'L') &&
            (app_to_upper(line[1]) == 'O') &&
            (app_to_upper(line[2]) == 'G') &&
            (line[3] == '?') &&
            (line[4] == '\0')) ? 1U : 0U;
}

static uint8_t app_console_parse_log_set(const char *line, const char *cmd, uint8_t *enable)
{
    uint8_t i = 0U;

    if ((line == (const char *)0) || (cmd == (const char *)0) || (enable == (uint8_t *)0)) {
        return 0U;
    }

    while (cmd[i] != '\0') {
        if (app_to_upper(line[i]) != cmd[i]) {
            return 0U;
        }
        i++;
    }

    if (line[i] != ' ') {
        return 0U;
    }
    i++;

    if ((line[i] == '0') && (line[i + 1U] == '\0')) {
        *enable = 0U;
        return 1U;
    }
    if ((line[i] == '1') && (line[i + 1U] == '\0')) {
        *enable = 1U;
        return 1U;
    }

    app_log_write("ERR: INVALID_FORMAT\r\n");
    return 1U;
}

static void app_console_print_log_flags(void)
{
    app_log_flags_t f;

    app_log_get_flags(&f);
    app_log_printf("GPSDO_LOG,GPSDO2=%lu,LEGACY10=%lu,RTC_PHASE=%lu,PHASE10=%lu,EVENT=%lu,DAC_ADJUST=%lu\r\n",
                   (unsigned long)f.gpsdo2,
                   (unsigned long)f.legacy_gpsdo10,
                   (unsigned long)f.rtc_phase,
                   (unsigned long)f.phase10,
                   (unsigned long)f.event,
                   (unsigned long)f.dac_adjust);
}

static uint8_t app_console_handle_log_command(const char *line)
{
    uint8_t enable = 0U;

    if (app_is_log_query(line) != 0U) {
        app_console_print_log_flags();
        return 1U;
    }
    if (app_console_parse_log_set(line, "LOGPHASE", &enable) != 0U) {
        app_log_set_phase_detail(enable);
        app_console_print_log_flags();
        return 1U;
    }
    if (app_console_parse_log_set(line, "LOGRTC", &enable) != 0U) {
        app_log_set_rtc_phase(enable);
        app_console_print_log_flags();
        return 1U;
    }
    if (app_console_parse_log_set(line, "LOGP10", &enable) != 0U) {
        app_log_set_phase10(enable);
        app_console_print_log_flags();
        return 1U;
    }
    if (app_console_parse_log_set(line, "LOGLEGACY", &enable) != 0U) {
        app_log_set_legacy_gpsdo10(enable);
        app_console_print_log_flags();
        return 1U;
    }
    if (app_console_parse_log_set(line, "LOGEVENT", &enable) != 0U) {
        app_log_set_event(enable);
        app_console_print_log_flags();
        return 1U;
    }
    if (app_console_parse_log_set(line, "LOGDAC", &enable) != 0U) {
        app_log_set_dac_adjust(enable);
        app_console_print_log_flags();
        return 1U;
    }

    return 0U;
}

static uint32_t app_float_volts_to_x10000(float v)
{
    if (v <= 0.0f) {
        return 0UL;
    }

    return (uint32_t)((v * 10000.0f) + 0.5f);
}

static void app_console_print_i64(int64_t value)
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

static void app_console_print_adc_test(void)
{
    uint16_t raw_min = 0U;
    uint16_t raw_max = 0U;
    uint16_t raw_avg = 0U;
    float v = 0.0f;
    uint32_t v_x10000;
    HAL_StatusTypeDef st;

    st = vct_adc_read_stats(64U, &raw_min, &raw_max, &raw_avg, &v);
    if (st != HAL_OK) {
        app_log_printf("ADC_TEST_FAIL status=%lu channel=ADC1_IN0,rank=1,sample=239.5,samples=64\r\n",
                       (unsigned long)st);
        return;
    }

    v_x10000 = app_float_volts_to_x10000(v);
    app_log_printf("ADC_TEST channel=ADC1_IN0,rank=1,sample=239.5,raw_min=%lu,raw_max=%lu,raw_avg=%lu,vdda=3.3600,v=%lu.%04lu,samples=64,formula=raw*vdda/4095\r\n",
                   (unsigned long)raw_min,
                   (unsigned long)raw_max,
                   (unsigned long)raw_avg,
                   (unsigned long)(v_x10000 / 10000UL),
                   (unsigned long)(v_x10000 % 10000UL));
    app_log_write("ADC_CHECK: disconnect VCT wire then test PA0=GND raw~0, PA0=VDDA raw~4095, PA0=VCT raw~1900\r\n");
}

static void app_console_print_phase(void)
{
    app_cpld_phase_t phase;
    HAL_StatusTypeDef st;
    int64_t ns;
    uint32_t status;

    st = app_cpld_read_phase(&phase);
    if (st != HAL_OK) {
        app_log_printf("RTC_PHASE_NOW,READ=FAIL,STATUS=%lu\r\n", (unsigned long)st);
        return;
    }

    ns = ((int64_t)phase.phase_ticks) * 20LL;
    status = phase.phase_status;

    app_log_printf("RTC_PHASE_NOW,TICK=%ld,NS=", (long)phase.phase_ticks);
    app_console_print_i64(ns);
    app_log_printf(",SEQ=%lu,STATUS=0x%08lX,DEBUG=0x%08lX,VALID=%lu,GPS=%lu,RTC=%lu,MISS=%lu,OVF=%lu,ALIVE=%lu\r\n",
                   (unsigned long)phase.phase_seq,
                   (unsigned long)status,
                   (unsigned long)phase.rtc_debug,
                   (unsigned long)((status & CPLD_RTC_PHASE_VALID_MASK) ? 1UL : 0UL),
                   (unsigned long)((status & CPLD_RTC_PHASE_GPS_SEEN_MASK) ? 1UL : 0UL),
                   (unsigned long)((status & CPLD_RTC_PHASE_RTC_SEEN_MASK) ? 1UL : 0UL),
                   (unsigned long)((status & CPLD_RTC_PHASE_MISSING_RTC_MASK) ? 1UL : 0UL),
                   (unsigned long)((status & CPLD_RTC_PHASE_OVERFLOW_MASK) ? 1UL : 0UL),
                   (unsigned long)((status & CPLD_RTC_PHASE_ALIVE_MASK) ? 1UL : 0UL));
}

static void app_console_print_spiid(void)
{
    uint32_t sig = 0UL;
    uint32_t sig_stable = 0UL;
    uint32_t status = 0UL;
    uint32_t debug_cnt = 0UL;
    uint32_t window_seq = 0UL;
    app_cpld_phase_t phase;
    HAL_StatusTypeDef st_sig;
    HAL_StatusTypeDef st_sig_stable;
    HAL_StatusTypeDef st_status;
    HAL_StatusTypeDef st_debug;
    HAL_StatusTypeDef st_seq;
    HAL_StatusTypeDef st_phase;
    const app_cpld_signature_stable_t *sig_detail;

    st_sig = app_cpld_read_signature(&sig);
    st_sig_stable = app_cpld_read_signature_stable(&sig_stable);
    sig_detail = app_cpld_get_last_signature_stable();
    st_status = app_cpld_read_status(&status);
    st_debug = app_cpld_read_reg32(CPLD_REG_DEBUG_CNT, &debug_cnt);
    st_seq = app_cpld_read_reg32(CPLD_REG_WINDOW_SEQ, &window_seq);
    st_phase = app_cpld_read_phase(&phase);

    app_log_printf("SPIID: SIG=0x%08lX ST_SIG=%lu SIG_STABLE=0x%08lX ST_STABLE=%lu RETRY=%lu SHIFTED_COUNT=%lu STATUS=0x%08lX ST_STATUS=%lu DEBUG_CNT=%lu ST_DEBUG=%lu WINDOW_SEQ=0x%08lX ST_SEQ=%lu RTC_PHASE_TICKS=%ld RTC_PHASE_SEQ=%lu RTC_PHASE_STATUS=0x%08lX RTC_DEBUG=0x%08lX ST_RTC_PHASE=%lu\r\n",
                   (unsigned long)sig,
                   (unsigned long)st_sig,
                   (unsigned long)sig_stable,
                   (unsigned long)st_sig_stable,
                   (unsigned long)sig_detail->retry_count,
                   (unsigned long)sig_detail->shifted_count,
                   (unsigned long)status,
                   (unsigned long)st_status,
                   (unsigned long)debug_cnt,
                   (unsigned long)st_debug,
                   (unsigned long)window_seq,
                   (unsigned long)st_seq,
                   (long)phase.phase_ticks,
                   (unsigned long)phase.phase_seq,
                   (unsigned long)phase.phase_status,
                   (unsigned long)phase.rtc_debug,
                   (unsigned long)st_phase);
}

static void app_console_run_spitest(void)
{
    uint16_t i;
    uint16_t sig_ok = 0U;
    uint16_t sig_shift = 0U;
    uint16_t sig_bad = 0U;
    uint16_t hal_err = 0U;
    uint16_t status_alive_ok = 0U;
    uint32_t sig = 0UL;
    uint32_t status = 0UL;
    uint32_t debug_cnt = 0UL;
    uint32_t window_seq = 0UL;
    HAL_StatusTypeDef st_sig;
    HAL_StatusTypeDef st_status;
    HAL_StatusTypeDef st_debug;
    HAL_StatusTypeDef st_seq;

    for (i = 0U; i < 100U; i++) {
        st_sig = app_cpld_read_signature(&sig);
        if (st_sig == HAL_OK) {
            if (sig == CPLD_SIGNATURE_VALUE) {
                sig_ok++;
            } else if (sig == CPLD_SIGNATURE_SHIFTED_VALUE) {
                sig_shift++;
            } else {
                sig_bad++;
            }
        } else {
            hal_err++;
        }

        st_status = app_cpld_read_status(&status);
        if (st_status == HAL_OK) {
            if ((status & CPLD_STATUS_ALIVE_MASK) != 0UL) {
                status_alive_ok++;
            }
        } else {
            hal_err++;
        }

        st_debug = app_cpld_read_reg32(CPLD_REG_DEBUG_CNT, &debug_cnt);
        if (st_debug != HAL_OK) {
            hal_err++;
        }

        st_seq = app_cpld_read_reg32(CPLD_REG_WINDOW_SEQ, &window_seq);
        if (st_seq != HAL_OK) {
            hal_err++;
        }
    }

    app_log_printf("SPITEST: N=100 SIG_OK=%lu SIG_SHIFT=%lu SIG_BAD=%lu HAL_ERR=%lu STATUS_ALIVE_OK=%lu LAST_DBG=%lu LAST_SEQ=0x%08lX\r\n",
                   (unsigned long)sig_ok,
                   (unsigned long)sig_shift,
                   (unsigned long)sig_bad,
                   (unsigned long)hal_err,
                   (unsigned long)status_alive_ok,
                   (unsigned long)debug_cnt,
                   (unsigned long)window_seq);
}

static void app_print_dac_code_test(void)
{
    app_log_printf("DAC_CODE_TEST: 1.0000V=%lu, 1.5000V=%lu, 1.5580V=%lu\r\n",
                   (unsigned long)app_dac8571_voltage_to_code(1000000UL),
                   (unsigned long)app_dac8571_voltage_to_code(1500000UL),
                   (unsigned long)app_dac8571_voltage_to_code(1558000UL));
}

static void app_print_dac_parse(uint32_t uv, uint8_t restart_auto)
{
    uint32_t volts = uv / 1000000UL;
    uint32_t frac4 = (uv % 1000000UL) / 100UL;

    if (restart_auto != 0U) {
        app_log_printf("DAC_PARSE: %lu.%04luVD\r\n",
                       (unsigned long)volts,
                       (unsigned long)frac4);
    } else {
        app_log_printf("DAC_PARSE: %lu.%04luV\r\n",
                       (unsigned long)volts,
                       (unsigned long)frac4);
    }
}

static void app_try_clear_windows_after_dac(void)
{
    uint32_t sig;
    HAL_StatusTypeDef st;

    st = app_cpld_read_signature(&sig);
    if ((st != HAL_OK) || (sig != CPLD_SIGNATURE_VALUE)) {
        app_log_write("DAC_CLEAR_WINDOWS: WARN:CPLD_OFFLINE_SKIP_CLEAR\r\n");
        return;
    }

    st = app_cpld_clear_measurement_windows();
    if (st == HAL_OK) {
        app_log_write("DAC_CLEAR_WINDOWS: OK\r\n");
    } else {
        app_log_printf("DAC_CLEAR_WINDOWS: WARN:CPLD_CLEAR_FAIL status=%lu\r\n",
                       (unsigned long)st);
    }
}

void app_console_init(void)
{
    g_console_len = 0U;
    app_print_dac_code_test();
}

void app_console_poll(void)
{
    uint8_t ch;

    while (app_uart_debug_get_char(&ch) != 0U) {
        if ((ch == (uint8_t)'\r') || (ch == (uint8_t)'\n')) {
            if (g_console_len != 0U) {
                g_console_line[g_console_len] = '\0';
                app_console_execute_line(g_console_line);
                g_console_len = 0U;
            }
        } else if (g_console_len < (CONSOLE_LINE_LEN - 1U)) {
            g_console_line[g_console_len] = (char)ch;
            g_console_len++;
        } else {
            g_console_len = 0U;
            app_log_write("ERR: CMD_TOO_LONG\r\n");
        }
    }
}

void app_console_execute_line(const char *line)
{
    if ((line == (const char *)0) || (line[0] == '\0')) {
        return;
    }

    app_log_printf("CMD_RX: %s\r\n", line);

    if (app_starts_with_help(line) != 0U) {
        app_console_print_help();
        return;
    }

    if (app_is_spiid_query(line) != 0U) {
        app_console_print_spiid();
        return;
    }

    if (app_is_phase_query(line) != 0U) {
        app_console_print_phase();
        return;
    }

    if (app_is_phasestat_query(line) != 0U) {
        app_gpsdo10_print_phase_stat();
        app_gpsdo_print_phase_fine_status();
        return;
    }

    if (app_console_handle_log_command(line) != 0U) {
        return;
    }

    if (app_is_spitest_query(line) != 0U) {
        app_console_run_spitest();
        return;
    }

    if (app_is_adc_query(line) != 0U) {
        app_console_print_adc_test();
        return;
    }

    if (app_console_handle_dac_command(line) != 0U) {
        return;
    }

    app_log_write("ERR: UNKNOWN_CMD\r\n");
}

uint8_t app_console_handle_dac_command(const char *line)
{
    uint32_t volts = 0UL;
    uint32_t frac4 = 0UL;
    uint8_t frac_digits = 0U;
    uint8_t restart_auto = 0U;
    uint8_t i = 3U;
    uint32_t uv;
    uint16_t mv;
    uint16_t code;
    uint16_t old_code;
    uint32_t old_uv;
    HAL_StatusTypeDef st;

    if ((line == (const char *)0) ||
        (app_to_upper(line[0]) != 'D') ||
        (app_to_upper(line[1]) != 'A') ||
        (app_to_upper(line[2]) != 'C')) {
        return 0U;
    }

    if (line[i] == ' ') {
        i++;
    }

    if (app_is_digit(line[i]) == 0U) {
        app_log_write("ERR: INVALID_FORMAT\r\n");
        return 1U;
    }

    while (app_is_digit(line[i]) != 0U) {
        volts = (volts * 10UL) + (uint32_t)(line[i] - '0');
        i++;
    }

    if (line[i] != '.') {
        app_log_write("ERR: INVALID_FORMAT\r\n");
        return 1U;
    }
    i++;

    if (app_is_digit(line[i]) == 0U) {
        app_log_write("ERR: INVALID_FORMAT\r\n");
        return 1U;
    }

    while (app_is_digit(line[i]) != 0U) {
        if (frac_digits >= 4U) {
            app_log_write("ERR: TOO_MANY_DECIMALS\r\n");
            return 1U;
        }
        frac4 = (frac4 * 10UL) + (uint32_t)(line[i] - '0');
        frac_digits++;
        i++;
    }

    while (frac_digits < 4U) {
        frac4 *= 10UL;
        frac_digits++;
    }

    if (app_to_upper(line[i]) != 'V') {
        app_log_write("ERR: INVALID_FORMAT\r\n");
        return 1U;
    }
    i++;

    if (app_to_upper(line[i]) == 'D') {
        restart_auto = 1U;
        i++;
    }

    if (line[i] != '\0') {
        app_log_write("ERR: INVALID_FORMAT\r\n");
        return 1U;
    }

    uv = (volts * 1000000UL) + (frac4 * 100UL);
    if (uv > DAC_MAX_UV) {
        app_log_write("ERR: range 0.0000~3.3000V\r\n");
        return 1U;
    }

    app_print_dac_parse(uv, restart_auto);
    code = app_dac8571_voltage_to_code(uv);
    old_code = app_dac8571_get_last_code();
    old_uv = app_log_dac_code_to_uv(old_code);
    st = app_dac8571_write_code(code);
    mv = (uint16_t)(uv / 1000UL);

    app_log_printf("DAC_WRITE: code=%lu mv=%lu status=%lu\r\n",
                   (unsigned long)code,
                   (unsigned long)mv,
                   (unsigned long)st);

    if (st != HAL_OK) {
        app_log_printf("ERR: DAC_WRITE status=%lu\r\n", (unsigned long)st);
        app_log_dac_adjust_fail_event("CONSOLE", old_code, code, "MANUAL_CMD", st);
        return 1U;
    }
    app_log_dac_adjust_event("CONSOLE", old_code, code, old_uv, uv, "MANUAL_CMD");

    if (restart_auto != 0U) {
        app_try_clear_windows_after_dac();
    }

    app_gpsdo_on_manual_dac_set(code, uv, restart_auto);

    return 1U;
}
