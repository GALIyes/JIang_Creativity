#include "app_gpsdo_dryrun.h"

#include "app_config.h"
#include "app_cpld_spi.h"
#include "app_gps_uart.h"
#include "app_pps.h"
#include "app_uart_debug.h"

static uint8_t g_have_seq10;
static uint8_t g_last_seq10;

static int32_t app_count_error(uint32_t count, uint32_t target)
{
    if (count >= target) {
        return (int32_t)(count - target);
    }

    return -(int32_t)(target - count);
}

void app_gpsdo_dryrun_init(void)
{
    g_have_seq10 = 0U;
    g_last_seq10 = 0U;

    app_log_printf("GPSDO_DRY_RUN=1 TARGET10=%lu OFFSET=%ld DAC_AUTO_WRITE=0\r\n",
                   (unsigned long)GPSDO_TARGET_COUNT10,
                   (long)GPSDO_TARGET_COUNT10_OFFSET);
}

void app_gpsdo_dryrun_poll_1s(void)
{
    uint32_t sig;
    uint32_t status;
    uint32_t debug_cnt;
    uint32_t pps_debug;
    uint32_t count10;
    uint8_t seq10;
    uint8_t seq50;
    uint8_t clr10;
    uint8_t clr50;
    uint8_t ready;
    uint8_t seq_changed;
    int32_t err10;
    app_pps_snapshot_t pps;
    app_gps_uart_stats_t gps;

    if (app_cpld_read_signature(&sig) != HAL_OK) {
        app_log_write("SPI_ERR: SIGNATURE\r\n");
        return;
    }
    if (app_cpld_read_status(&status) != HAL_OK) {
        app_log_write("SPI_ERR: STATUS\r\n");
        return;
    }
    if (app_cpld_read_reg32(CPLD_REG_DEBUG_CNT, &debug_cnt) != HAL_OK) {
        debug_cnt = 0UL;
    }
    if (app_cpld_read_reg32(CPLD_REG_PPS_DEBUG, &pps_debug) != HAL_OK) {
        pps_debug = 0UL;
    }
    if (app_cpld_read_window_seq(&seq10, &seq50, &clr10, &clr50) != HAL_OK) {
        seq10 = 0U;
        seq50 = 0U;
        clr10 = 0U;
        clr50 = 0U;
    }

    app_pps_get_snapshot(&pps);
    app_gps_uart_get_stats(&gps);

    ready = ((sig == CPLD_SIGNATURE_VALUE) && ((status & CPLD_STATUS_ALIVE_MASK) != 0UL)) ? 1U : 0U;
    app_log_printf("STAT SIG=0x%08lX READY=%lu ST=0x%08lX DBG=%lu PPS=%lu/%lums GPSRX=%lu RMC=%lu SEQ10=%lu\r\n",
                   (unsigned long)sig,
                   (unsigned long)ready,
                   (unsigned long)status,
                   (unsigned long)debug_cnt,
                   (unsigned long)pps.count,
                   (unsigned long)pps.interval_ms,
                   (unsigned long)gps.rx_bytes,
                   (unsigned long)gps.rmc_lines,
                   (unsigned long)seq10);

    seq_changed = ((g_have_seq10 == 0U) || (seq10 != g_last_seq10)) ? 1U : 0U;
    if (((status & CPLD_STATUS_FINISH_10S_MASK) != 0UL) && (seq_changed != 0U)) {
        if (app_cpld_read_count10(&count10) == HAL_OK) {
            err10 = app_count_error(count10, GPSDO_TARGET_COUNT10);
            app_log_printf("DISC_DRY COUNT10=%lu TARGET10=%lu ERR10=%ld SEQ10=%lu SEQ50=%lu CLR=%lu/%lu DAC_WRITE=0\r\n",
                           (unsigned long)count10,
                           (unsigned long)GPSDO_TARGET_COUNT10,
                           (long)err10,
                           (unsigned long)seq10,
                           (unsigned long)seq50,
                           (unsigned long)clr10,
                           (unsigned long)clr50);
        } else {
            app_log_write("SPI_ERR: COUNT10\r\n");
        }

        g_last_seq10 = seq10;
        g_have_seq10 = 1U;
    }
}
