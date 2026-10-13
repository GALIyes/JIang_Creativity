#ifndef APP_GPS_UART_H
#define APP_GPS_UART_H

#include "main.h"

typedef struct {
    uint32_t rx_bytes;
    uint32_t nmea_lines;
    uint32_t rmc_lines;
    uint32_t overflow_lines;
    uint32_t dollar_count;
    uint32_t last_rmc_tick_ms;
    /*
     * Number of times the receive DMA had to be restarted after a blocking
     * UART error.  Without it a dead receiver is completely invisible: the
     * module keeps talking, the pin keeps toggling, RX_BYTES simply stops
     * growing, and that looks identical to "the wire fell off".
     */
    uint32_t uart_err;
} app_gps_uart_stats_t;

/*
 * Sky / fix health, read out of the sentences that were last received.
 *
 * The counters in app_gps_uart_stats_t cannot separate "the receiver is hearing
 * nothing" from "it hears satellites but has not solved a fix yet", and those
 * two need opposite reactions: the first is an antenna/RF fault that will never
 * resolve by waiting, the second just needs open sky and time.
 *
 *   rmc_fix        'A' fix valid, 'V' no fix yet, '\0' no RMC seen
 *   gga_quality    GGA field 6, 0 = invalid, 1 = GPS, 2 = DGPS
 *   gga_sats_used  GGA field 7, satellites used in the solution
 *   sats_in_view   GSV field 3, satellites the receiver can hear at all
 *   snr_max        best C/N0 in dBHz seen in any GSV SNR field
 *   ever_had_sats  1 once sats_in_view has been non-zero at least once
 *
 * The per-type counters and the RMC period are counted by SENTENCE TYPE, not by
 * a hard-coded talker, so a receiver that answers with $BDRMC / $GBRMC instead
 * of $GNRMC is still visible here.  They answer two things the averages cannot:
 *
 *   - which sentences the module actually emits, and at what rate
 *     (rmc_period_ms is the navigation epoch period: exactly one RMC per fix)
 *   - whether SATS_IN_VIEW==0 is a real "no RF" verdict (GSV is arriving and
 *     reports zero) or merely "no GSV was ever received" (which says nothing
 *     about the antenna)
 */
typedef struct {
    char    rmc_fix;
    uint8_t gga_quality;
    uint8_t gga_sats_used;
    uint8_t sats_in_view;
    uint8_t snr_max;
    uint8_t ever_had_sats;
    uint8_t snr_best;           /* latched best C/N0 ever, dBHz */
    char    rmc_talker[3];      /* "GN" / "GP" / "BD" / "GB", "--" when none */
    uint32_t rmc_count;         /* RMC sentences seen, any talker */
    uint32_t gga_count;
    uint32_t gsa_count;
    uint32_t gsv_count;
    uint32_t rmc_period_ms;     /* gap between the last two RMCs = epoch period */
} app_gps_sky_stats_t;

HAL_StatusTypeDef app_gps_uart_init(void);
void app_gps_uart_poll(void);
void app_gps_uart_get_stats(app_gps_uart_stats_t *stats);
/*
 * Restart the USART2 receive DMA and count the event.  MAIN LOOP ONLY.
 *
 * In DMA mode the HAL treats ANY receive error (overrun, framing, noise,
 * parity) as blocking: it aborts the transfer, drops RxState back to READY and
 * calls HAL_UART_ErrorCallback().  Nothing in the original design restarted it,
 * so one error silenced the GPS receiver for the entire run.
 *
 * Called by the RxState watchdog inside app_gps_uart_poll(), which runs every
 * 20 ms -- not from the error callback, because re-arming needs
 * HAL_UART_DMAStop()/HAL_UART_Receive_DMA() and those are not ISR safe.
 */
void app_gps_uart_recover(void);
/*
 * Electrical probe of PA3 (USART2_RX) and PA1 (PPS), driven by the console
 * command PIN?.  Samples both pins for 1.2 s as fast as the core allows, then
 * reports the low-time ratio, the edge counts, the DMA progress, the raw
 * USART2 status register and a pull-down test on PA3.  Distinguishes "module
 * not driving the line" from "wire floating" from "module driving but silent"
 * from "traffic present, frame wrong", without a scope.
 * Blocks the main loop for the duration; the pin configuration is restored.
 */
void app_gps_probe_pins(void);
/*
 * Call from the main loop.  Re-runs the probe every
 * APP_GPS_PIN_PROBE_PERIOD_MS while BOTH rx_bytes and the PPS count are still
 * zero, so a terminal attached after boot cannot miss it; self-disables as soon
 * as either becomes non-zero, or when APP_GPS_PIN_PROBE_AUTO is 0.
 */
void app_gps_probe_tick(void);
/*
 * Walk the candidate GPS baud rates (9600/19200/38400/57600/115200), dwell at
 * each one, and report how many bytes and how many '$' frame starts actually
 * arrived.  Resolves the "RX_BYTES grows but DOLLAR stays 0" case, which means
 * the module is talking at a different rate than APP_GPS_UART_BAUD.
 * Blocks ~4 s, restores the configured rate afterwards, console command GPSBAUD?.
 */
void app_gps_baud_scan(void);
/*
 * Print the last raw RMC / GGA / GSA sentences verbatim, plus the counters
 * behind them.  Settles "is it no-fix, or is the parser wrong?" -- a receiver
 * without a fix still sends every sentence once a second, only with an empty
 * time field and status 'V'.  Console command NMEA?.
 */
void app_gps_dump_nmea(void);
/*
 * Snapshot the sky / fix health (see app_gps_sky_stats_t).  Parsed on demand out
 * of the stored sentences, so it is cheap enough to call from the periodic
 * telemetry row and needs no extra work in the receive path.
 */
void app_gps_get_sky_stats(app_gps_sky_stats_t *out);

#endif /* APP_GPS_UART_H */
