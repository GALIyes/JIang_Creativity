#include "app_gps_uart.h"

#include "app_config.h"
#include "app_gps_rmc.h"
#include "app_pps.h"
#include "app_uart_debug.h"
#include "usart.h"

#include <string.h>

#define GPS_DMA_RX_LEN 256U
#define GPS_LINE_LEN   96U

/* USART2_RX is PA3; the module's 1PPS lands on PA1 (GPS_PPS_Pin in main.h). */
#define GPS_RX_GPIO_PORT GPIOA
#define GPS_RX_PIN_MASK  GPIO_PIN_3

static uint8_t g_gps_dma_rx[GPS_DMA_RX_LEN];
static uint16_t g_gps_old_pos;
static char g_gps_line[GPS_LINE_LEN];
static uint16_t g_gps_line_len;
static uint8_t g_gps_started;
static app_gps_uart_stats_t g_gps_stats;

/*
 * Verbatim copies of the three sentences that decide whether the receiver
 * actually has a position fix.
 *
 * "RMC_LINES keeps ticking" only proves the module is talking: a receiver with
 * NO fix still emits every sentence once a second, but with an EMPTY time
 * field and status 'V' instead of 'A'.  Those two field values are exactly
 * what the summary counters cannot show, so keep the sentences themselves and
 * let the terminal decide.
 *
 * Matched on the sentence type (chars 3..5) rather than the talker, so
 * $GNRMC, $GPRMC and $BDRMC all land in the same slot.
 */
#define GPS_NMEA_SLOT_LEN 100U
/* Window over which GSV sentences are folded together; see app_gps_parse_gsv(). */
#define GPS_GSV_WINDOW_MS 1000U
#define GPS_SLOT_RMC 0U
#define GPS_SLOT_GGA 1U
#define GPS_SLOT_GSA 2U
#define GPS_SLOT_GSV 3U
#define GPS_SLOT_N   4U

static char g_nmea_slot[GPS_SLOT_N][GPS_NMEA_SLOT_LEN];
static uint32_t g_nmea_slot_ms[GPS_SLOT_N];
/*
 * Census by sentence type.  Counted here, where the slot is decided purely on
 * characters 3..5, so it is independent of the talker -- which is exactly what
 * makes it able to expose a receiver that never emits $GNRMC / $GPRMC (the only
 * two the RMC path accepts) while still sending valid RMC data under a BeiDou
 * talker, e.g. $GBRMC or $BDRMC.
 */
static uint32_t g_nmea_cnt[GPS_SLOT_N];
static char     g_nmea_talker[GPS_SLOT_N][2];
static uint32_t g_rmc_period_ms;

/*
 * GSV health, extracted because it answers the one question the counters
 * cannot: is the receiver hearing any satellites at all?
 *
 *   sats in view > 0, SNR > 0  -> RF path is fine, just needs time / geometry
 *   sats in view == 0          -> the receiver hears NOTHING on 1575 MHz:
 *                                 antenna not connected, active antenna with no
 *                                 bias voltage, or a dead antenna
 *
 * Without this, "no fix" is indistinguishable from "no antenna", and waiting
 * for a fix that can never come wastes hours.
 */
static uint8_t  g_gsv_sats_in_view;
static uint8_t  g_gsv_max_snr;
static uint8_t  g_gsv_ever_had_sats;
static uint8_t  g_gsv_snr_best;
static uint32_t g_gsv_window_ms;
static uint8_t  g_gsv_win_sats;
static uint8_t  g_gsv_win_snr;

static uint8_t app_gps_sentence_slot(const char *line)
{
    uint8_t i;

    if ((line == (const char *)0) || (line[0] != '$')) {
        return 0xFFU;
    }
    for (i = 0U; i < 6U; i++) {
        if (line[i] == '\0') {
            return 0xFFU;           /* too short to carry a sentence type */
        }
    }

    if (strncmp(&line[3], "RMC", 3U) == 0) { return GPS_SLOT_RMC; }
    if (strncmp(&line[3], "GGA", 3U) == 0) { return GPS_SLOT_GGA; }
    if (strncmp(&line[3], "GSA", 3U) == 0) { return GPS_SLOT_GSA; }
    if (strncmp(&line[3], "GSV", 3U) == 0) { return GPS_SLOT_GSV; }
    return 0xFFU;
}

/*
 * Minimal GSV reader.  Field layout:
 *   field 3               = satellites in view
 *   fields 7, 11, 15, 19  = SNR of each reported satellite   (empty = no signal)
 * Only the first digit-run of each field is read, and the rest of the field is
 * skipped, so empty SNR fields are handled naturally.
 */
static void app_gps_parse_gsv(const char *line, uint32_t now_ms)
{
    uint16_t pos = 0U;
    uint8_t field = 0U;
    uint8_t sats = 0U;
    uint8_t max_snr = 0U;
    uint8_t got_sats = 0U;

    while ((line[pos] != '\0') && (line[pos] != '*') && (field < 20U)) {
        if (line[pos] == ',') {
            field++;
            pos++;
            continue;
        }

        if ((field == 3U) || ((field >= 7U) && (((field - 7U) % 4U) == 0U))) {
            uint16_t v = 0U;
            uint8_t n = 0U;

            while ((line[pos] >= '0') && (line[pos] <= '9') && (n < 3U)) {
                v = (uint16_t)((v * 10U) + (uint16_t)(line[pos] - '0'));
                n++;
                pos++;
            }
            if (field == 3U) {
                if (v <= 64U) {
                    sats = (uint8_t)v;
                }
                got_sats = 1U;
            } else if (v > max_snr) {
                max_snr = (uint8_t)v;
            }
            while ((line[pos] != '\0') && (line[pos] != ',') && (line[pos] != '*')) {
                pos++;              /* skip the rest of this field */
            }
            continue;
        }
        pos++;
    }

    if (got_sats == 0U) {
        return;
    }

    /*
     * A multi-constellation receiver emits GSV once PER CONSTELLATION
     * ($GPGSV, $GLGSV, $GAGSV, $GBGSV), and each sentence carries the count for
     * its own constellation only.  Publishing the newest sentence directly
     * ("last write wins") therefore throws away every constellation but the
     * last one -- and since the last group may legitimately report 0, a healthy
     * antenna reading 8 GPS + 5 BeiDou satellites can be reported as
     * "0 in view".  That is exactly the kind of false "no RF" verdict that
     * sends someone chasing a good antenna.
     *
     * Instead, hold the maximum across a short window: the whole message set of
     * one epoch arrives within a few tens of milliseconds, so a 1 s window
     * spans every constellation exactly once and can never straddle two epochs.
     * A maximum (rather than a sum) is used on purpose -- it answers "is
     * anything being heard at all", which is the question that matters, and it
     * cannot be inflated by the same satellite appearing in two groups.
     */
    if ((g_gsv_window_ms == 0UL) || ((now_ms - g_gsv_window_ms) > GPS_GSV_WINDOW_MS)) {
        g_gsv_window_ms = now_ms;
        g_gsv_win_sats = 0U;
        g_gsv_win_snr = 0U;
    }
    if (sats > g_gsv_win_sats) {
        g_gsv_win_sats = sats;
    }
    if (max_snr > g_gsv_win_snr) {
        g_gsv_win_snr = max_snr;
    }

    g_gsv_sats_in_view = g_gsv_win_sats;
    g_gsv_max_snr = g_gsv_win_snr;

    /* Latched: "was anything ever heard", and the best C/N0 ever achieved.
     * Kept separately from the windowed values so a momentary dropout cannot
     * erase the evidence that the RF path does work. */
    if (sats != 0U) {
        g_gsv_ever_had_sats = 1U;
    }
    if (max_snr > g_gsv_snr_best) {
        g_gsv_snr_best = max_snr;
    }
}

/*
 * Field readers for the stored sentences.  NMEA fields are comma separated and
 * field 0 is the talker+type, so e.g. RMC status is field 2 and the GGA fix
 * quality / satellites-used pair is fields 6 / 7.
 *
 * Both walk to field `idx` and ignore everything past it, which is what makes
 * them tolerate the short sentences a receiver emits before it has a fix
 * ("$GNRMC,,V,,,..."): the requested field is simply empty and reads as 0/'\0'.
 */
static uint16_t app_gps_field_num(const char *s, uint8_t idx)
{
    uint8_t f = 0U;
    uint16_t i;
    uint16_t v = 0U;
    uint8_t n = 0U;

    if ((s == (const char *)0) || (s[0] != '$')) {
        return 0U;
    }
    for (i = 0U; (s[i] != '\0') && (s[i] != '*'); i++) {
        if (s[i] == ',') {
            f++;
            continue;
        }
        if (f == idx) {
            if ((s[i] >= '0') && (s[i] <= '9')) {
                if (n < 5U) {
                    v = (uint16_t)((v * 10U) + (uint16_t)(s[i] - '0'));
                    n++;
                }
            } else if (n != 0U) {
                break;              /* end of that number */
            }
        }
    }
    return v;
}

static char app_gps_field_char(const char *s, uint8_t idx)
{
    uint8_t f = 0U;
    uint16_t i;

    if ((s == (const char *)0) || (s[0] != '$')) {
        return '\0';
    }
    for (i = 0U; (s[i] != '\0') && (s[i] != '*'); i++) {
        if (s[i] == ',') {
            f++;
            continue;
        }
        if (f == idx) {
            return s[i];
        }
    }
    return '\0';
}

void app_gps_get_sky_stats(app_gps_sky_stats_t *out)
{
    if (out == (app_gps_sky_stats_t *)0) {
        return;
    }

    out->rmc_fix       = app_gps_field_char(g_nmea_slot[GPS_SLOT_RMC], 2U);
    out->gga_quality   = (uint8_t)app_gps_field_num(g_nmea_slot[GPS_SLOT_GGA], 6U);
    out->gga_sats_used = (uint8_t)app_gps_field_num(g_nmea_slot[GPS_SLOT_GGA], 7U);
    out->sats_in_view  = g_gsv_sats_in_view;
    out->snr_max       = g_gsv_max_snr;
    out->ever_had_sats = g_gsv_ever_had_sats;
    out->snr_best      = g_gsv_snr_best;

    out->rmc_count     = g_nmea_cnt[GPS_SLOT_RMC];
    out->gga_count     = g_nmea_cnt[GPS_SLOT_GGA];
    out->gsa_count     = g_nmea_cnt[GPS_SLOT_GSA];
    out->gsv_count     = g_nmea_cnt[GPS_SLOT_GSV];
    out->rmc_period_ms = g_rmc_period_ms;

    if (out->rmc_count != 0UL) {
        out->rmc_talker[0] = g_nmea_talker[GPS_SLOT_RMC][0];
        out->rmc_talker[1] = g_nmea_talker[GPS_SLOT_RMC][1];
    } else {
        out->rmc_talker[0] = '-';
        out->rmc_talker[1] = '-';
    }
    out->rmc_talker[2] = '\0';
}

/*
 * Never emit a non-printable byte into the telemetry: the talker characters
 * come from whatever the receiver put on the wire, and a stray byte here would
 * show up as garbage in the log -- the exact symptom this project spent a long
 * time chasing.
 */
static char app_gps_safe_char(char c)
{
    if (((c >= 'A') && (c <= 'Z')) || ((c >= '0') && (c <= '9'))) {
        return c;
    }
    return '?';
}

static void app_gps_keep_sentence(const char *line, uint32_t now_ms)
{
    uint8_t slot = app_gps_sentence_slot(line);
    uint16_t i;

    if (slot >= GPS_SLOT_N) {
        return;
    }

    for (i = 0U; (i < (GPS_NMEA_SLOT_LEN - 1U)) && (line[i] != '\0'); i++) {
        g_nmea_slot[slot][i] = line[i];
    }
    g_nmea_slot[slot][i] = '\0';

    g_nmea_cnt[slot]++;
    /* slot >= 0 implies the line is '$' + 6 characters, so [1] and [2] exist. */
    g_nmea_talker[slot][0] = app_gps_safe_char(line[1]);
    g_nmea_talker[slot][1] = app_gps_safe_char(line[2]);

    /*
     * The gap between consecutive RMCs IS the receiver's navigation epoch
     * period -- one RMC per solution.  Taken against the previous timestamp, so
     * it has to run before that timestamp is refreshed.
     */
    if ((slot == GPS_SLOT_RMC) && (g_nmea_slot_ms[slot] != 0UL)) {
        g_rmc_period_ms = now_ms - g_nmea_slot_ms[slot];
    }

    g_nmea_slot_ms[slot] = now_ms;

    if (slot == GPS_SLOT_GSV) {
        app_gps_parse_gsv(line, now_ms);
    }
}

static void app_gps_handle_line(const char *line)
{
    if ((line == (const char *)0) || (line[0] == '\0')) {
        return;
    }

    app_gps_keep_sentence(line, HAL_GetTick());

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
    uint8_t i;

    memset(&g_gps_stats, 0, sizeof(g_gps_stats));
    g_gps_old_pos = 0U;
    g_gps_line_len = 0U;
    g_gps_started = 0U;
    for (i = 0U; i < GPS_SLOT_N; i++) {
        g_nmea_slot[i][0] = '\0';
        g_nmea_slot_ms[i] = 0UL;
        g_nmea_cnt[i] = 0UL;
        g_nmea_talker[i][0] = '-';
        g_nmea_talker[i][1] = '-';
    }
    g_rmc_period_ms = 0UL;
    g_gsv_sats_in_view = 0U;
    g_gsv_max_snr = 0U;
    g_gsv_ever_had_sats = 0U;
    g_gsv_snr_best = 0U;
    g_gsv_window_ms = 0UL;
    g_gsv_win_sats = 0U;
    g_gsv_win_snr = 0U;

    st = HAL_UART_Receive_DMA(&huart2, g_gps_dma_rx, GPS_DMA_RX_LEN);
    if (st == HAL_OK) {
        g_gps_started = 1U;
        if (huart2.hdmarx != (DMA_HandleTypeDef *)0) {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
    }

    return st;
}

/* Current write position of the circular RX DMA, as an index into rx buffer. */
static uint16_t app_gps_dma_pos(void)
{
    uint16_t pos;

    if (huart2.hdmarx == (DMA_HandleTypeDef *)0) {
        return 0U;
    }

    pos = (uint16_t)(GPS_DMA_RX_LEN - (uint16_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx));
    if (pos >= GPS_DMA_RX_LEN) {
        pos = 0U;
    }
    return pos;
}

/*
 * Restart the GPS receiver.  MAIN LOOP ONLY -- deliberately not called from the
 * USART2 interrupt: HAL_UART_DMAStop() internally busy-waits on the DMA enable
 * bit and HAL_UART_Receive_DMA() takes the HAL lock, neither of which belongs
 * in an ISR.  The main loop runs every 20 ms, so the error callback only needs
 * to let the watchdog below do the work.
 *
 * Draining SR then DR first is required, not cosmetic: on STM32F1 that is what
 * clears a latched ORE/FE/NE/PE, and without it the very next byte would trip
 * the same blocking error again.
 */
void app_gps_uart_recover(void)
{
    g_gps_stats.uart_err++;

    (void)USART2->SR;
    (void)USART2->DR;

    (void)HAL_UART_DMAStop(&huart2);
    huart2.RxState = HAL_UART_STATE_READY;

    if (HAL_UART_Receive_DMA(&huart2, g_gps_dma_rx, GPS_DMA_RX_LEN) == HAL_OK) {
        g_gps_started = 1U;
        /* The DMA counter restarts from GPS_DMA_RX_LEN, so position 0 is the
         * new read cursor -- anything the old buffer still held is stale. */
        g_gps_old_pos = 0U;
        if (huart2.hdmarx != (DMA_HandleTypeDef *)0) {
            __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        }
    } else {
        g_gps_started = 0U;
    }
}

void app_gps_uart_poll(void)
{
    uint16_t pos;

    if (huart2.hdmarx == (DMA_HandleTypeDef *)0) {
        return;
    }

    /*
     * Receiver watchdog.  In DMA mode HAL_UART_IRQHandler() treats ANY receive
     * error as blocking: it aborts the transfer, drops RxState back to READY and
     * calls HAL_UART_ErrorCallback().  Nothing here used to restart it, so ONE
     * overrun or framing error silenced the module for the rest of the run --
     * the pin kept toggling, GPS_STAT kept looking plausible, and RX_BYTES just
     * stopped growing, which reads exactly like "the wire fell off".
     *
     * RxState is the single source of truth for "is the HAL really receiving",
     * and polling it here also covers every other way the receiver can be left
     * down (a failed re-init, the baud scan, a swallowed error).  Gated on
     * hdmarx rather than g_gps_started on purpose: if a recovery attempt fails,
     * the next 20 ms tick must try again instead of giving up.
     */
    if (huart2.RxState != HAL_UART_STATE_BUSY_RX) {
        app_gps_uart_recover();
        return;
    }

    pos = app_gps_dma_pos();

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

/*
 * Print the raw RMC / GGA / GSA sentences plus every counter behind them
 * (console command NMEA?).
 *
 * Every derived value in GPS_STAT comes from these three sentences, so when a
 * parsed field looks wrong this is the honest way to settle it.  In particular
 * it distinguishes the two cases that look identical in the counters:
 *
 *   $GNRMC,123519,A,...        status A  -> the receiver HAS a fix
 *   $GNRMC,,V,,,,,,,,,,N*53    status V  -> NO fix: the time field is empty
 *
 * A module with no fix keeps transmitting every sentence once a second, so a
 * rising RMC_LINES count says nothing about the fix at all.
 */
void app_gps_dump_nmea(void)
{
    app_gps_uart_stats_t s;
    const app_rtc_state_t *rtc = app_rtc_get_state();
    uint32_t now_ms = HAL_GetTick();

    app_gps_uart_get_stats(&s);

    app_log_printf("NMEA_DUMP,RX_BYTES=%lu,LINES=%lu,RMC_LINES=%lu,DOLLAR=%lu,OVERFLOW=%lu,CK_OK=%lu,CK_FAIL=%lu,PARSE_FAIL=%lu,RMC_VALID=%lu,RTC_VALID=%lu\r\n",
                   (unsigned long)s.rx_bytes,
                   (unsigned long)s.nmea_lines,
                   (unsigned long)s.rmc_lines,
                   (unsigned long)s.dollar_count,
                   (unsigned long)s.overflow_lines,
                   (unsigned long)app_rmc_get_checksum_ok(),
                   (unsigned long)app_rmc_get_checksum_fail(),
                   (unsigned long)app_rmc_get_parse_fail(),
                   (unsigned long)rtc->rmc_valid,
                   (unsigned long)rtc->rtc_valid);

    app_log_printf("NMEA_RMC,AGE_MS=%lu,%s\r\n",
                   (unsigned long)((g_nmea_slot[GPS_SLOT_RMC][0] != '\0') ? (now_ms - g_nmea_slot_ms[GPS_SLOT_RMC]) : 0xFFFFFFFFUL),
                   (g_nmea_slot[GPS_SLOT_RMC][0] != '\0') ? g_nmea_slot[GPS_SLOT_RMC] : "(none)");
    app_log_printf("NMEA_GGA,AGE_MS=%lu,%s\r\n",
                   (unsigned long)((g_nmea_slot[GPS_SLOT_GGA][0] != '\0') ? (now_ms - g_nmea_slot_ms[GPS_SLOT_GGA]) : 0xFFFFFFFFUL),
                   (g_nmea_slot[GPS_SLOT_GGA][0] != '\0') ? g_nmea_slot[GPS_SLOT_GGA] : "(none)");
    app_log_printf("NMEA_GSA,AGE_MS=%lu,%s\r\n",
                   (unsigned long)((g_nmea_slot[GPS_SLOT_GSA][0] != '\0') ? (now_ms - g_nmea_slot_ms[GPS_SLOT_GSA]) : 0xFFFFFFFFUL),
                   (g_nmea_slot[GPS_SLOT_GSA][0] != '\0') ? g_nmea_slot[GPS_SLOT_GSA] : "(none)");
    app_log_printf("NMEA_GSV,AGE_MS=%lu,%s\r\n",
                   (unsigned long)((g_nmea_slot[GPS_SLOT_GSV][0] != '\0') ? (now_ms - g_nmea_slot_ms[GPS_SLOT_GSV]) : 0xFFFFFFFFUL),
                   (g_nmea_slot[GPS_SLOT_GSV][0] != '\0') ? g_nmea_slot[GPS_SLOT_GSV] : "(none)");

    app_log_printf("NMEA_RF,SATS_IN_VIEW=%lu,SNR_MAX=%lu,EVER_HAD_SATS=%lu\r\n",
                   (unsigned long)g_gsv_sats_in_view,
                   (unsigned long)g_gsv_max_snr,
                   (unsigned long)g_gsv_ever_had_sats);

    app_log_write("NMEA_HINT,RMC f2 A=fix V=no-fix. GGA f6=quality f7=sats used. NMEA_RF SATS_IN_VIEW=0 means the receiver hears NOTHING on 1575 MHz: antenna not connected, active antenna without bias, or dead antenna. SATS_IN_VIEW>0 with SNR_MAX>0 means RF is fine and only time/geometry is missing.\r\n");
}

/*
 * ---------------------------------------------------------------------------
 * GPS UART baud scan (console command GPSBAUD?).
 *
 * README documents 38400 for this port and usart.c uses it, but a u-blox
 * module's factory default is 9600 while most breakout boards ship pre-set to
 * 38400 -- so the real rate depends on which module is fitted.  A mismatch has
 * a very specific signature: RX_BYTES keeps growing (mis-framed bytes still
 * raise RXNE) while DOLLAR, NMEA_LINES and RMC_LINES all stay at zero.
 *
 * This walks the candidates, dwells at each, and reports the byte and '$'
 * deltas.  The correct rate is the one with a healthy DOLLAR count.
 *
 * DMA state must be unwound explicitly.  HAL_UART_Init() resets huart->RxState
 * but NOT hdma->State, and HAL_DMA_Start_IT() refuses anything but
 * HAL_DMA_STATE_READY -- so re-arming without HAL_UART_DMAStop() first would
 * fail with HAL_BUSY and leave the receiver silently dead.
 * ---------------------------------------------------------------------------
 */
#define GPS_BAUD_CANDIDATES 5U
#define GPS_BAUD_DWELL_MS   800U

static const uint32_t g_gps_baud_list[GPS_BAUD_CANDIDATES] = {
    9600U, 19200U, 38400U, 57600U, 115200U
};

static HAL_StatusTypeDef app_gps_arm_rx(uint32_t baud)
{
    (void)HAL_UART_DMAStop(&huart2);

    huart2.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        return HAL_ERROR;
    }

    g_gps_old_pos = 0U;
    if (HAL_UART_Receive_DMA(&huart2, g_gps_dma_rx, GPS_DMA_RX_LEN) != HAL_OK) {
        return HAL_ERROR;
    }
    if (huart2.hdmarx != (DMA_HandleTypeDef *)0) {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    }
    return HAL_OK;
}

void app_gps_baud_scan(void)
{
    app_gps_uart_stats_t s;
    uint32_t saved = huart2.Init.BaudRate;
    uint32_t best_baud = 0UL;
    uint32_t best_dollar = 0UL;
    uint8_t i;

    if (huart2.hdmarx == (DMA_HandleTypeDef *)0) {
        app_log_write("GPSBAUD_SCAN,ABORT=NO_HDMARX\r\n");
        return;
    }

    app_log_printf("GPSBAUD_SCAN,START=1,CURRENT=%lu\r\n", (unsigned long)saved);

    for (i = 0U; i < GPS_BAUD_CANDIDATES; i++) {
        uint32_t b = g_gps_baud_list[i];
        uint32_t t0;
        uint32_t bytes0;
        uint32_t dollar0;
        uint32_t lines0;
        uint32_t d_bytes;
        uint32_t d_dollar;

        if (app_gps_arm_rx(b) != HAL_OK) {
            app_log_printf("GPSBAUD,%lu,ARM=FAIL\r\n", (unsigned long)b);
            continue;
        }

        app_gps_uart_get_stats(&s);
        bytes0 = s.rx_bytes;
        dollar0 = s.dollar_count;
        lines0 = s.nmea_lines;

        t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < GPS_BAUD_DWELL_MS) {
            app_gps_uart_poll();
        }
        app_gps_uart_poll();

        app_gps_uart_get_stats(&s);
        d_bytes = s.rx_bytes - bytes0;
        d_dollar = s.dollar_count - dollar0;

        app_log_printf("GPSBAUD,%lu,BYTES=%lu,DOLLAR=%lu,LINES=%lu,RMC=%lu\r\n",
                       (unsigned long)b,
                       (unsigned long)d_bytes,
                       (unsigned long)d_dollar,
                       (unsigned long)(s.nmea_lines - lines0),
                       (unsigned long)s.rmc_lines);

        if (d_dollar > best_dollar) {
            best_dollar = d_dollar;
            best_baud = b;
        }
    }

    if (app_gps_arm_rx(saved) != HAL_OK) {
        app_log_write("GPSBAUD_SCAN,RESTORE=FAIL\r\n");
    }

    if (best_baud != 0UL) {
        app_log_printf("GPSBAUD_SCAN,BEST=%lu,DOLLAR=%lu,SET_APP_GPS_UART_BAUD_TO_THIS\r\n",
                       (unsigned long)best_baud,
                       (unsigned long)best_dollar);
    } else {
        app_log_write("GPSBAUD_SCAN,BEST=NONE,NO_DOLLAR_AT_ANY_RATE\r\n");
    }
}

/*
 * ---------------------------------------------------------------------------
 * Electrical probe of the two GPS lines (console command PIN?).
 *
 * WHY THIS EXISTS
 * "GPS not answering" has four very different causes that all look identical in
 * the counters:
 *
 *   1. module unpowered / unwired / dead        -> nothing drives PA3
 *   2. module wired to the wrong pin            -> nothing drives PA3
 *   3. wire open or pin floating                -> PA3 picks up noise
 *   4. wired and transmitting, but framed wrong -> PA3 carries real traffic
 *
 * RX_BYTES=0 cannot separate them, and a scope is not always at hand.  Reading
 * GPIOA->IDR while sampling as fast as the core allows does separate them, and
 * needs no reconfiguration:
 *
 *   - IDR always reflects the physical pin level, even when the pin is in
 *     alternate-function (USART2_RX) or EXTI mode, so this is a pure observer.
 *   - USART2->DR is deliberately never read: on this part that would steal a
 *     byte out of the circular DMA and desynchronise the receiver.
 *   - The DMA counter is polled only to watch it change.  It only decrements
 *     when RXNE fires, i.e. when a byte actually arrived, so DMA_MOVES is a
 *     direct second opinion on "did anything reach the peripheral at all".
 *
 * PA3 (USART2_RX) low-time ratio, in per-mille, over the 1.2 s window:
 *     ~0        line sits idle high -> nothing is driving it (cases 1 and 2)
 *     2 .. 150  an NMEA burst is present -> the electrical path works, so the
 *               problem is baud rate / framing / NMEA parsing, not wiring
 *     ~1000     line held low -> short to ground or inverted drive
 *     scattered across the range -> floating node picking up noise (case 3)
 *
 * PA1 (GPS_PPS_Pin) gets the same treatment, and the EXTI counter is read
 * before and after so a PPS that physically arrives but never reaches the
 * interrupt is visible (PPS_EDGES > 0 with PPS_EXTI_DELTA = 0).
 * ---------------------------------------------------------------------------
 */
static uint32_t app_gps_permille(uint32_t part, uint32_t total)
{
    if (total == 0UL) {
        return 0UL;
    }

    /* 64 bit intermediate: n reaches several million over 1.2 s, so a 32 bit
     * (part * 1000) would wrap. */
    return (uint32_t)(((uint64_t)part * 1000ULL) / (uint64_t)total);
}

/*
 * Pull-down test on PA3 (the second half of the probe).
 *
 * RX_LOW_PM ~0 from the sampling pass means the line sits high -- but that is
 * ambiguous on its own: the node can be actively driven high by the module, or
 * merely FLOATING (open wire, or a module that is not powered) with NOPULL
 * leaving it wherever leakage puts it.
 *
 * Enabling the ~40k internal pull-down separates the two: a real push-pull
 * driver wins easily and the pin still reads high, while an open node is
 * dragged to 0.  So
 *
 *   RX_PULLDN_HIGH_PM ~1000 -> driven high  -> wire and power are fine, the
 *                              module itself is not transmitting
 *   RX_PULLDN_HIGH_PM ~0    -> open node    -> wire open or module unpowered
 *
 * PA3 is restored to exactly what HAL_UART_MspInit() left it in
 * (INPUT / NOPULL) before returning.
 *
 * CRITICAL: the receiver is silenced (USART_CR1_RE cleared) for the duration.
 * Dragging a FLOATING node down with the internal pull creates a genuine
 * falling edge, and USART2 would dutifully frame it as a byte.  That byte is an
 * artefact of the test -- and a nasty one, because seeing it would make the
 * caller conclude "traffic present".  RE=0 stops start-bit detection without
 * touching the DMA, the GPIO configuration, or the HAL state.
 */
static uint32_t app_gps_pull_down_high_permille(void)
{
    GPIO_InitTypeDef gi = {0};
    uint32_t high = 0UL;
    uint32_t i;

    CLEAR_BIT(USART2->CR1, USART_CR1_RE);

    gi.Pin = GPS_RX_PIN_MASK;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPS_RX_GPIO_PORT, &gi);

    for (i = 0UL; i < 20000UL; i++) {
        __NOP();                    /* let the node settle through 40k */
    }
    for (i = 0UL; i < 4096UL; i++) {
        if ((GPS_RX_GPIO_PORT->IDR & GPS_RX_PIN_MASK) != 0UL) {
            high++;
        }
    }

    gi.Pin = GPS_RX_PIN_MASK;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPS_RX_GPIO_PORT, &gi);

    /* Let the line settle back at its natural level before the receiver
     * resumes, so that transition is not framed either. */
    for (i = 0UL; i < 20000UL; i++) {
        __NOP();
    }
    SET_BIT(USART2->CR1, USART_CR1_RE);

    return app_gps_permille(high, 4096UL);
}

/*
 * Second borrowed-pin test, this time pulling PA1 (the PPS line) UP.
 *
 * PA1 reads a solid low with no edges at all, which is ambiguous in the same
 * way PA3 was: the receiver's TIMEPULSE output may be actively holding it low
 * (a powered u-blox with no time fix does exactly that), or the node may
 * simply be floating low.  The ~40k internal pull-up separates them: an
 * actively driven pin stays low, a floating one comes up.
 *
 * The EXTI configuration is restored to exactly what gpio.c set
 * (GPIO_MODE_IT_RISING / NOPULL); PPS_EXTI_DELTA in the same report confirms
 * the interrupt still works afterwards.
 *
 * CRITICAL: the EXTI line is masked for the duration, and this is not
 * optional.  HAL_GPIO_Init() only touches EXTI->IMR when the requested mode is
 * an EXTI mode, so reconfiguring PA1 as a plain input leaves the line ARMED --
 * and pulling up a FLOATING node is a genuine rising edge, which the still
 * armed EXTI counts as a PPS.  That is exactly what happened on the bench:
 * PPS_CNT went 0 -> 1 at the instant of the first probe, which then made the
 * probe look like it had seen a real PPS and silenced itself.
 */
static uint32_t app_gps_pps_pull_up_high_permille(void)
{
    GPIO_InitTypeDef gi = {0};
    uint32_t high = 0UL;
    uint32_t i;

    EXTI->IMR &= ~(uint32_t)GPS_PPS_Pin;        /* mask the edge detector */

    gi.Pin = GPS_PPS_Pin;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPS_PPS_GPIO_Port, &gi);

    for (i = 0UL; i < 20000UL; i++) {
        __NOP();
    }
    for (i = 0UL; i < 4096UL; i++) {
        if ((GPS_PPS_GPIO_Port->IDR & (uint32_t)GPS_PPS_Pin) != 0UL) {
            high++;
        }
    }

    gi.Pin = GPS_PPS_Pin;
    gi.Mode = GPIO_MODE_IT_RISING;
    gi.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPS_PPS_GPIO_Port, &gi);

    /* Drop whatever the borrowed pin latched, then re-arm the line.  Nothing
     * the test itself caused must ever reach the PPS counter. */
    EXTI->PR = (uint32_t)GPS_PPS_Pin;
    EXTI->IMR |= (uint32_t)GPS_PPS_Pin;
    HAL_NVIC_ClearPendingIRQ(GPS_PPS_EXTI_IRQn);

    return app_gps_permille(high, 4096UL);
}

/*
 * Sample EVERY pin of port A and port B, not just the four the design mentions.
 *
 * The user has confirmed the TTL wiring is present and cross-connected, yet
 * PA3 measures as a floating node (the internal 40k pull-down wins), i.e.
 * nothing at all is driving it.  When the wiring "is right" but the pin is
 * dead, the remaining explanations all reduce to "the signal is arriving
 * somewhere else": a different header, a different pin, a swapped pair, or a
 * module that is simply not powered.
 *
 * Reading GPIOx->IDR is a pure observation -- it works whatever the pin is
 * configured as -- so a whole-port sweep finds the traffic wherever it landed.
 * The edge count is the discriminator:
 *
 *   ~0            pin is static (nothing attached, or a steady level)
 *   30 .. 3000    edges only during the once-per-second burst  ->  SERIAL DATA
 *   ~100000+      constant toggling  ->  a FLOATING, noisy node
 *
 * Port A and B cover every UART option on this part (PA2/PA3, PA9/PA10,
 * PB10/PB11) plus SPI1 and I2C1.  PA9 is this log's own TX and will always
 * show edges; PA13/PA14 are SWD.
 */
#define GPS_SCAN_PORTS     2U
#define GPS_SCAN_EDGE_MIN  32UL     /* below this it is not a data burst */
#define GPS_SCAN_MAX_LINES 20U

static GPIO_TypeDef *const g_gps_scan_port[GPS_SCAN_PORTS] = { GPIOA, GPIOB };
/* File scope, not automatic: 16 counts per port per metric would be ~260 bytes
 * of stack in a function that already sits under a printf chain.  The probe is
 * only ever called from the main loop, never reentrantly. */
static uint32_t g_scan_low[GPS_SCAN_PORTS][16];
static uint32_t g_scan_edges[GPS_SCAN_PORTS][16];
static uint16_t g_scan_last[GPS_SCAN_PORTS];

/* Build "PA3" / "PB11" without needing a 32 entry name table. */
static void app_gps_pin_name(uint8_t port_idx, uint8_t bit, char *out)
{
    out[0] = 'P';
    out[1] = (port_idx == 0U) ? 'A' : 'B';
    if (bit < 10U) {
        out[2] = (char)('0' + bit);
        out[3] = '\0';
    } else {
        out[2] = '1';
        out[3] = (char)('0' + (bit - 10U));
        out[4] = '\0';
    }
}

void app_gps_probe_pins(void)
{
    app_pps_snapshot_t pps_before;
    app_pps_snapshot_t pps_after;
    app_gps_uart_stats_t stats_now;
    uint32_t sr;
    uint32_t dma_prev;
    uint32_t dma_moves = 0UL;
    uint32_t dma_final;
    uint32_t t0;
    uint32_t n = 0UL;
    uint32_t pull_high;
    uint32_t pps_pull_high;
    uint32_t active_pins;
    uint32_t shown;
    uint16_t pos_before_pull;
    uint16_t pos_after_pull;
    uint8_t first = 1U;
    uint8_t pi;
    uint8_t b;

    if (huart2.hdmarx == (DMA_HandleTypeDef *)0) {
        app_log_write("GPS_PIN,ABORT=NO_HDMARX\r\n");
        return;
    }

    app_pps_get_snapshot(&pps_before);
    dma_prev = (uint32_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);

    app_log_write("GPS_PIN,START=1,WINDOW_MS=1200\r\n");

    for (pi = 0U; pi < GPS_SCAN_PORTS; pi++) {
        g_scan_last[pi] = 0xFFFFU;
        for (b = 0U; b < 16U; b++) {
            g_scan_low[pi][b] = 0UL;
            g_scan_edges[pi][b] = 0UL;
        }
    }

    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 1200UL) {
        uint32_t k;
        for (k = 0UL; k < 1024UL; k++) {
            for (pi = 0U; pi < GPS_SCAN_PORTS; pi++) {
                uint16_t now_bits = (uint16_t)(g_gps_scan_port[pi]->IDR & 0xFFFFU);
                uint16_t diff = (uint16_t)(now_bits ^ g_scan_last[pi]);

                g_scan_last[pi] = now_bits;
                if (first != 0U) {
                    continue;           /* no previous level to compare with */
                }
                for (b = 0U; b < 16U; b++) {
                    uint16_t m = (uint16_t)(1U << b);
                    if ((now_bits & m) == 0U) {
                        g_scan_low[pi][b]++;
                    }
                    if ((diff & m) != 0U) {
                        g_scan_edges[pi][b]++;
                    }
                }
            }
            first = 0U;
            n++;

            if ((k & 0x3FU) == 0U) {
                uint32_t c = (uint32_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);
                if (c != dma_prev) {
                    dma_moves++;
                    dma_prev = c;
                }
            }
        }
    }

    /* Count whatever really arrived during the sampling window BEFORE the pin
     * is borrowed below, so a genuine burst is never dropped. */
    app_gps_uart_poll();

    app_pps_get_snapshot(&pps_after);
    dma_final = (uint32_t)__HAL_DMA_GET_COUNTER(huart2.hdmarx);
    sr = (uint32_t)USART2->SR;

    /* Borrow PA3 for the pull-down test.  The receiver is RE-gated inside, but
     * still skip anything the transition queued, provided it is only a handful
     * of bytes -- a small artefact must never be counted as NMEA.  If a large
     * delta appeared, real traffic is flowing and gets counted normally. */
    pos_before_pull = app_gps_dma_pos();
    pull_high = app_gps_pull_down_high_permille();
    pos_after_pull = app_gps_dma_pos();
    if ((uint16_t)((uint16_t)(pos_after_pull - pos_before_pull) & 0x00FFU) <= 8U) {
        g_gps_old_pos = pos_after_pull;
    }

    /* Same idea on the PPS line: does anything actively hold PA1 low? */
    pps_pull_high = app_gps_pps_pull_up_high_permille();

    app_gps_uart_get_stats(&stats_now);

    app_log_printf("GPS_PIN,N=%lu,RX_LOW_PM=%lu,PPS_LOW_PM=%lu,RX_EDGES=%lu,PPS_EDGES=%lu,PPS_EXTI_DELTA=%lu,DMA_MOVES=%lu,DMA_C=%lu,UART2_STARTED=%lu,RX_PULLDN_HIGH_PM=%lu,PPS_PULLUP_HIGH_PM=%lu,RX_BYTES_NOW=%lu\r\n",
                   (unsigned long)n,
                   (unsigned long)app_gps_permille(g_scan_low[0][3], n),
                   (unsigned long)app_gps_permille(g_scan_low[0][1], n),
                   (unsigned long)g_scan_edges[0][3],
                   (unsigned long)g_scan_edges[0][1],
                   (unsigned long)(pps_after.count - pps_before.count),
                   (unsigned long)dma_moves,
                   (unsigned long)dma_final,
                   (unsigned long)g_gps_started,
                   (unsigned long)pull_high,
                   (unsigned long)pps_pull_high,
                   (unsigned long)stats_now.rx_bytes);

    /* Candidate-pin activity table: which pin, if any, carries traffic. */
    app_log_printf("GPS_SCAN,PA3_LOW_PM=%lu,PA3_EDGES=%lu,PA10_LOW_PM=%lu,PA10_EDGES=%lu,PB11_LOW_PM=%lu,PB11_EDGES=%lu,PA1_LOW_PM=%lu,PA1_EDGES=%lu\r\n",
                   (unsigned long)app_gps_permille(g_scan_low[0][3], n),
                   (unsigned long)g_scan_edges[0][3],
                   (unsigned long)app_gps_permille(g_scan_low[0][10], n),
                   (unsigned long)g_scan_edges[0][10],
                   (unsigned long)app_gps_permille(g_scan_low[1][11], n),
                   (unsigned long)g_scan_edges[1][11],
                   (unsigned long)app_gps_permille(g_scan_low[0][1], n),
                   (unsigned long)g_scan_edges[0][1]);

    /* Whole-port sweep: report every pin of A and B that actually toggles.
     * This is the one that answers "the wiring is fine, so where did it go?" */
    active_pins = 0UL;
    for (pi = 0U; pi < GPS_SCAN_PORTS; pi++) {
        for (b = 0U; b < 16U; b++) {
            if (g_scan_edges[pi][b] >= GPS_SCAN_EDGE_MIN) {
                active_pins++;
            }
        }
    }
    app_log_printf("GPS_ALLSCAN,N=%lu,EDGE_MIN=%lu,ACTIVE_PINS=%lu\r\n",
                   (unsigned long)n,
                   (unsigned long)GPS_SCAN_EDGE_MIN,
                   (unsigned long)active_pins);

    shown = 0UL;
    for (pi = 0U; pi < GPS_SCAN_PORTS; pi++) {
        for (b = 0U; b < 16U; b++) {
            char nm[5];

            if (g_scan_edges[pi][b] < GPS_SCAN_EDGE_MIN) {
                continue;
            }
            if (shown >= GPS_SCAN_MAX_LINES) {
                continue;
            }
            app_gps_pin_name(pi, b, nm);
            app_log_printf("GPS_ALLSCAN_PIN,%s,LOW_PM=%lu,EDGES=%lu\r\n",
                           nm,
                           (unsigned long)app_gps_permille(g_scan_low[pi][b], n),
                           (unsigned long)g_scan_edges[pi][b]);
            shown++;
        }
    }

    /* SR alone is safe to read; it does not clear RXNE and does not touch DR. */
    app_log_printf("GPS_PIN_SR,SR=0x%04lX,ORE=%lu,NE=%lu,FE=%lu,PE=%lu,RXNE=%lu,IDLE=%lu\r\n",
                   (unsigned long)sr,
                   (unsigned long)((sr & 0x0008UL) ? 1UL : 0UL),
                   (unsigned long)((sr & 0x0004UL) ? 1UL : 0UL),
                   (unsigned long)((sr & 0x0002UL) ? 1UL : 0UL),
                   (unsigned long)((sr & 0x0001UL) ? 1UL : 0UL),
                   (unsigned long)((sr & 0x0020UL) ? 1UL : 0UL),
                   (unsigned long)((sr & 0x0010UL) ? 1UL : 0UL));

    app_log_write("GPS_PIN_HINT,READ AS A PAIR. RX_LOW_PM 2..150=NMEA on PA3; ~0=idle. RX_PULLDN_HIGH_PM~1000=PA3 driven high (wire ok, module silent), ~0=PA3 floating. PPS_PULLUP_HIGH_PM~0=PA1 actively held low (module alive), ~1000=PA1 floating. GPS_ALLSCAN EDGES: 30..3000=serial burst, ~100000+=floating noise, 0=static. TYPE PIN? TO RUN THIS ON DEMAND.\r\n");
}

void app_gps_probe_tick(void)
{
#if APP_GPS_PIN_PROBE_AUTO
    static uint32_t s_last_ms;
    app_gps_uart_stats_t s;
    app_pps_snapshot_t pps;
    uint32_t now_ms = HAL_GetTick();

    app_gps_uart_get_stats(&s);
    app_pps_get_snapshot(&pps);

    /* Self-disabling, but only on evidence of a SUSTAINED signal.  A single
     * stray byte, or a single edge that the probe's own borrowed-pin test
     * latched, is not a signal and must not silence the diagnostic. */
    if ((s.rx_bytes >= APP_GPS_PIN_PROBE_MIN_BYTES) ||
        (pps.count >= APP_GPS_PIN_PROBE_MIN_PPS)) {
        return;
    }

    if ((now_ms - s_last_ms) < APP_GPS_PIN_PROBE_PERIOD_MS) {
        return;
    }
    s_last_ms = now_ms;

    app_gps_probe_pins();
#endif
}
