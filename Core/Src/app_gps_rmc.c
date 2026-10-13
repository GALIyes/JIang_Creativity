#include "app_gps_rmc.h"

#include <string.h>

static app_rtc_state_t g_rtc;
static char g_rmc_pending_buf[RMC_LINE_MAX];
static volatile uint8_t g_rmc_pending_flag;
static uint32_t g_rmc_ck_ok;
static uint32_t g_rmc_ck_fail;
static uint32_t g_rmc_parse_fail;

static uint8_t app_is_leap(uint16_t y)
{
    return ((y % 400U) == 0U) || (((y % 4U) == 0U) && ((y % 100U) != 0U)) ? 1U : 0U;
}

static uint8_t app_days_in_month(uint16_t m, uint16_t y)
{
    static const uint8_t d[12] = {31U,28U,31U,30U,31U,30U,31U,31U,30U,31U,30U,31U};

    if ((m == 2U) && (app_is_leap(y) != 0U)) {
        return 29U;
    }
    if ((m < 1U) || (m > 12U)) {
        return 0U;
    }
    return d[m - 1U];
}

static uint8_t app_digit_val(char c)
{
    if ((c >= '0') && (c <= '9')) {
        return (uint8_t)(c - '0');
    }
    return 0U;
}

static uint8_t app_parse_6digit_hms(const char *s, uint16_t p, uint8_t *h, uint8_t *m, uint8_t *sec)
{
    uint8_t i;

    for (i = 0U; i < 6U; i++) {
        if ((s[p + i] < '0') || (s[p + i] > '9')) {
            return 0U;
        }
    }

    *h = (uint8_t)(app_digit_val(s[p + 0U]) * 10U + app_digit_val(s[p + 1U]));
    *m = (uint8_t)(app_digit_val(s[p + 2U]) * 10U + app_digit_val(s[p + 3U]));
    *sec = (uint8_t)(app_digit_val(s[p + 4U]) * 10U + app_digit_val(s[p + 5U]));
    return 1U;
}

static uint8_t app_parse_6digit_dmy(const char *s, uint16_t p, uint8_t *d, uint8_t *mo, uint8_t *y)
{
    uint8_t i;

    for (i = 0U; i < 6U; i++) {
        if ((s[p + i] < '0') || (s[p + i] > '9')) {
            return 0U;
        }
    }

    *d = (uint8_t)(app_digit_val(s[p + 0U]) * 10U + app_digit_val(s[p + 1U]));
    *mo = (uint8_t)(app_digit_val(s[p + 2U]) * 10U + app_digit_val(s[p + 3U]));
    *y = (uint8_t)(app_digit_val(s[p + 4U]) * 10U + app_digit_val(s[p + 5U]));
    return 1U;
}

static uint8_t app_hex_val(char c, uint8_t *v)
{
    if ((c >= '0') && (c <= '9')) {
        *v = (uint8_t)(c - '0');
        return 1U;
    }
    if ((c >= 'A') && (c <= 'F')) {
        *v = (uint8_t)(c - 'A' + 10);
        return 1U;
    }
    if ((c >= 'a') && (c <= 'f')) {
        *v = (uint8_t)(c - 'a' + 10);
        return 1U;
    }
    return 0U;
}

void app_rmc_init(void)
{
    memset(&g_rtc, 0, sizeof(g_rtc));
    g_rmc_pending_buf[0] = '\0';
    g_rmc_pending_flag = 0U;
    g_rmc_ck_ok = 0UL;
    g_rmc_ck_fail = 0UL;
    g_rmc_parse_fail = 0UL;
}

uint8_t app_rmc_check_checksum(const char *line)
{
    uint16_t i = 1U;
    uint8_t calc = 0U;
    uint8_t hi;
    uint8_t lo;

    if ((line == (const char *)0) || (line[0] != '$')) {
        return 0U;
    }

    while ((line[i] != '\0') && (line[i] != '*')) {
        calc ^= (uint8_t)line[i];
        i++;
    }
    if (line[i] != '*') {
        return 0U;
    }
    i++;

    if ((app_hex_val(line[i], &hi) == 0U) || (app_hex_val(line[i + 1U], &lo) == 0U)) {
        return 0U;
    }

    return (calc == (uint8_t)((hi << 4) | lo)) ? 1U : 0U;
}

uint8_t app_rmc_parse_time(const char *line, app_rmc_time_t *t)
{
    uint16_t field = 0U;
    uint16_t pos = 0U;
    uint8_t status_valid = 0U;
    uint8_t hh = 0U;
    uint8_t mm = 0U;
    uint8_t ss = 0U;
    uint8_t dd = 0U;
    uint8_t mo = 0U;
    uint8_t yy = 0U;
    uint8_t time_ok = 0U;
    uint8_t date_ok = 0U;

    if ((line == (const char *)0) || (t == (app_rmc_time_t *)0)) {
        return 0U;
    }

    while ((line[pos] != '\0') && (field <= 12U)) {
        if (line[pos] == ',') {
            field++;
            pos++;
            continue;
        }
        if (field == 1U) {
            time_ok = app_parse_6digit_hms(line, pos, &hh, &mm, &ss);
            while ((line[pos] != '\0') && (line[pos] != ',')) {
                pos++;
            }
            continue;
        }
        if (field == 2U) {
            if (line[pos] == 'A') {
                status_valid = 1U;
            }
            while ((line[pos] != '\0') && (line[pos] != ',')) {
                pos++;
            }
            continue;
        }
        if (field == 9U) {
            date_ok = app_parse_6digit_dmy(line, pos, &dd, &mo, &yy);
            break;
        }
        pos++;
    }

    if ((status_valid == 0U) || (time_ok == 0U) || (date_ok == 0U)) {
        return 0U;
    }
    if ((hh > 23U) || (mm > 59U) || (ss > 59U) || (dd < 1U) || (dd > 31U) || (mo < 1U) || (mo > 12U)) {
        return 0U;
    }

    t->utc_hour = hh;
    t->utc_min = mm;
    t->utc_sec = ss;
    t->day = dd;
    t->month = mo;
    t->year = (uint16_t)(2000U + yy);

    return 1U;
}

void app_rmc_utc_to_beijing(const app_rmc_time_t *utc, app_rmc_time_t *bj)
{
    if ((utc == (const app_rmc_time_t *)0) || (bj == (app_rmc_time_t *)0)) {
        return;
    }

    *bj = *utc;
    bj->utc_hour = (uint8_t)(bj->utc_hour + 8U);
    if (bj->utc_hour >= 24U) {
        bj->utc_hour = (uint8_t)(bj->utc_hour - 24U);
        bj->day++;
        if (bj->day > app_days_in_month(bj->month, bj->year)) {
            bj->day = 1U;
            bj->month++;
            if (bj->month > 12U) {
                bj->month = 1U;
                bj->year++;
            }
        }
    }
}

void app_rmc_store_line(const char *line)
{
    uint16_t i;

    if (line == (const char *)0) {
        return;
    }

    for (i = 0U; (i < (RMC_LINE_MAX - 1U)) && (line[i] != '\0'); i++) {
        g_rmc_pending_buf[i] = line[i];
    }
    g_rmc_pending_buf[i] = '\0';
    g_rmc_pending_flag = 1U;
}

void app_rmc_process_pending(void)
{
    char local_buf[RMC_LINE_MAX];
    app_rmc_time_t utc;
    app_rmc_time_t bj;

    __disable_irq();
    if (g_rmc_pending_flag == 0U) {
        __enable_irq();
        return;
    }
    memcpy(local_buf, g_rmc_pending_buf, sizeof(local_buf));
    g_rmc_pending_flag = 0U;
    __enable_irq();

    local_buf[RMC_LINE_MAX - 1U] = '\0';

    if (app_rmc_check_checksum(local_buf) == 0U) {
        g_rmc_ck_fail++;
        g_rtc.rmc_valid = 0U;
        return;
    }
    g_rmc_ck_ok++;

    if (app_rmc_parse_time(local_buf, &utc) == 0U) {
        g_rmc_parse_fail++;
        g_rtc.rmc_valid = 0U;
        return;
    }

    app_rmc_utc_to_beijing(&utc, &bj);
    g_rtc.bj = bj;
    g_rtc.rtc_valid = 1U;
    g_rtc.rtc_synced = 1U;
    g_rtc.sec_since_sync = 0U;
    g_rtc.rmc_valid = 1U;
}

void app_rtc_tick_second(void)
{
    if (g_rtc.rtc_valid == 0U) {
        return;
    }

    g_rtc.sec_since_sync++;
    if (g_rtc.sec_since_sync > 5U) {
        g_rtc.rtc_synced = 0U;
    }

    g_rtc.bj.utc_sec++;
    if (g_rtc.bj.utc_sec >= 60U) {
        g_rtc.bj.utc_sec = 0U;
        g_rtc.bj.utc_min++;
        if (g_rtc.bj.utc_min >= 60U) {
            g_rtc.bj.utc_min = 0U;
            g_rtc.bj.utc_hour++;
            if (g_rtc.bj.utc_hour >= 24U) {
                g_rtc.bj.utc_hour = 0U;
                g_rtc.bj.day++;
                if (g_rtc.bj.day > app_days_in_month(g_rtc.bj.month, g_rtc.bj.year)) {
                    g_rtc.bj.day = 1U;
                    g_rtc.bj.month++;
                    if (g_rtc.bj.month > 12U) {
                        g_rtc.bj.month = 1U;
                        g_rtc.bj.year++;
                    }
                }
            }
        }
    }
}

const app_rtc_state_t *app_rtc_get_state(void)
{
    return &g_rtc;
}

uint32_t app_rmc_get_checksum_ok(void)
{
    return g_rmc_ck_ok;
}

uint32_t app_rmc_get_checksum_fail(void)
{
    return g_rmc_ck_fail;
}

uint32_t app_rmc_get_parse_fail(void)
{
    return g_rmc_parse_fail;
}
