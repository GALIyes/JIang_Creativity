#include "app_uart_debug.h"

#include "app_config.h"
#include "app_gps_uart.h"
#include "app_pps.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/*
 * The GPSDO_AVG diagnostic line needs ~685 characters.  With 512 the line was
 * silently truncated by vsnprintf and the trailing DF50/DF100/DF200/DF1000_MHZ
 * fields -- the ones used to certify the 0.02 Hz average spec -- were lost.
 * The buffer is a file scope static rather than a 512 byte automatic: armlink
 * reported "Maximum Stack Usage = 1152 bytes" against Stack_Size = 0x400, and
 * the deepest chain ran through app_log_printf, so the old automatic buffer
 * overflowed the stack on every DISC50 correction.
 */
#define APP_LOG_BUFFER_LEN   1024U
#define APP_UART_RX_RING_LEN 128U
/*
 * TX is interrupt driven through this ring so app_log_write() never blocks.
 * HAL_UART_Transmit(..., 50 ms) used to stall the main loop for ~60 ms on one
 * GPSDO_AVG line (685 B at 115200 baud), and that call sits directly inside
 * the 1 s discipline tick.  A line that does not fit is dropped whole (never
 * split, never blocking) and counted; the console LOG? command reports the
 * count so silent data loss is impossible.
 */
#define APP_LOG_TX_RING_LEN  2048U

static uint8_t g_log_ready;
/* Set while the repeating banner sweep owns the port: a blocking sweep
 * transmit must not interleave with the interrupt-driven log ring. */
static uint8_t g_log_muted;
static uint8_t g_rx_byte;
static volatile uint16_t g_rx_head;
static volatile uint16_t g_rx_tail;
static uint8_t g_rx_ring[APP_UART_RX_RING_LEN];
static app_log_flags_t g_log_flags;
/* Only ever touched from the main loop; no ISR logs, so a single shared
 * buffer is safe and keeps 512 bytes off the stack. */
static char g_log_buf[APP_LOG_BUFFER_LEN];

static uint8_t g_tx_ring[APP_LOG_TX_RING_LEN];
static volatile uint16_t g_tx_head;
static volatile uint16_t g_tx_tail;
static volatile uint16_t g_tx_len;
static volatile uint8_t g_tx_busy;
/*
 * Set only once the HAL has actually accepted the transfer.  Between the
 * moment app_log_tx_kick() publishes g_tx_busy = 1 and the moment
 * HAL_UART_Transmit_IT() returns, the HAL still reports gState = READY, so a
 * resync running in that window would clear g_tx_len while the transfer it
 * belongs to is about to start -- the completion callback would then advance
 * g_tx_tail by zero and the same chunk would be retransmitted forever.  This
 * flag closes that window: resync refuses to touch a transfer the HAL has not
 * taken ownership of yet.
 */
static volatile uint8_t g_tx_started;
static volatile uint32_t g_tx_drop_count;

static void app_uart_debug_start_rx(void)
{
    (void)HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1U);
}

#if APP_UART1_BAUD_SWEEP_ENABLE
/*
 * Nominal rates to try.  A terminal configured for 115200 reads a banner sent
 * with nominal 115200/k legibly, where k is the real-to-assumed HSE ratio, so
 * this list covers every plausible OSC_IN frequency for an 8 MHz assumption:
 * the 'k =' comment names the OSC_IN value that each entry would detect.
 */
static const uint32_t g_baud_sweep[] = {
    115200U,    /* k = 1.000   OSC_IN = 8 MHz   (clock is as declared) */
    92160U,     /* k = 1.250   OSC_IN = 10 MHz  */
    76800U,     /* k = 1.500   OSC_IN = 12 MHz  */
    75000U,     /* k = 1.536   OSC_IN = 12.288 MHz */
    57600U,     /* k = 2.000   OSC_IN = 16 MHz  */
    46080U,     /* k = 2.500   OSC_IN = 20 MHz  */
    38400U,     /* k = 3.000   OSC_IN = 24 MHz  */
    28800U,     /* k = 4.000   OSC_IN = 32 MHz  */
    230400U,    /* k = 0.500   OSC_IN = 4 MHz   */
    153600U     /* k = 0.750   OSC_IN = 6 MHz   */
};

void app_uart_baud_sweep(void)
{
    char line[128];
    const uint8_t count = (uint8_t)(sizeof(g_baud_sweep) / sizeof(g_baud_sweep[0]));
    app_pps_snapshot_t pps;
    uint8_t i;
    uint8_t rep;
    int n;

    /* PPS_MS is the GPS one-second period measured with HAL_GetTick(), so it
     * reads 1000 * k where k is the real-to-assumed HSE ratio.  Putting it in
     * the banner means the one legible line reports both which nominal rate
     * worked AND the exact clock ratio (1500 -> 12 MHz, 1536 -> 12.288 MHz). */
    app_pps_get_snapshot(&pps);

    for (i = 0U; i < count; i++) {
        huart1.Init.BaudRate = g_baud_sweep[i];
        if (HAL_UART_Init(&huart1) != HAL_OK) {
            continue;
        }
        for (rep = 0U; rep < APP_UART1_BAUD_SWEEP_REPEATS; rep++) {
            n = snprintf(line, sizeof(line),
                         "\r\nBAUD_SWEEP,NOMINAL=%lu,PPS_MS=%lu\r\n",
                         (unsigned long)g_baud_sweep[i],
                         (unsigned long)pps.interval_ms);
            if (n <= 0) {
                break;
            }
            if (n > (int)(sizeof(line) - 1U)) {
                /* snprintf returns the length it would have needed; sending
                 * that many bytes would read past the buffer. */
                n = (int)(sizeof(line) - 1U);
            }
            (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n, 50U);
        }
    }

    /* Always leave the port on the configured rate. */
    huart1.Init.BaudRate = APP_UART1_BAUD;
    (void)HAL_UART_Init(&huart1);

    /* HAL_UART_Init() rewrites the whole peripheral, so the console receiver
     * has to be re-armed or the next byte is never requested. */
    huart1.RxState = HAL_UART_STATE_READY;
    app_uart_debug_start_rx();
}

void app_uart_baud_sweep_tick(void)
{
#if APP_UART1_BAUD_SWEEP_REPEAT_ENABLE
    static uint32_t last_ms;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_ms) >= APP_UART1_BAUD_SWEEP_PERIOD_MS) {
        last_ms = now_ms;
        app_uart_baud_sweep();
    }
#endif
}

#if APP_CLK_MEASURE_ENABLE
/*
 * MCU-clock measurement with the PC as the time reference.
 *
 * A UART link cannot measure its own bit-rate error: the receiver
 * re-synchronises on every start bit, so it tolerates roughly 5 % per frame
 * and the two candidates 76800 / 75000 (2.4 % apart) are equally legible.
 * GPS PPS is the primary reference (PPS_MS), but it needs a position fix,
 * which indoors may never arrive.
 *
 * So: bytes the PC sends back to back leave it every 10 bit times, i.e. every
 * 10/115200 s = 86.806 us, paced by the PC crystal.  DWT_CYCCNT ticks once per
 * HCLK cycle, so
 *
 *     k = HCLK_real / 72e6 = (cycles per frame) / 6250        (PC at 115200)
 *
 * 6250 -> 8 MHz OSC_IN, 9375 -> 12 MHz, 9600 -> 12.288 MHz.  A 2.4 % gap is
 * 225 cycles per frame and averages out to well under 1 % over a burst, so
 * this resolves what the UART itself cannot.
 */
#define APP_CLK_MEAS_GAP_CYCLES   20000UL   /* > 1 frame (6250..9600), < gap between sends */
#define APP_CLK_MEAS_MIN_FRAMES   4U        /* 3 intervals: ~0.5 % over ISR jitter */
#define APP_CLK_MEAS_DIVISOR      6250ULL   /* 10 * 72e6 / 115200 */

static uint8_t g_clkmeas_ready;
static uint32_t g_clkmeas_first;
static uint32_t g_clkmeas_last;
static uint32_t g_clkmeas_frames;
static uint32_t g_clkmeas_k_x10000;

void app_clk_measure_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_clkmeas_ready = 1U;
    g_clkmeas_frames = 0U;
    g_clkmeas_k_x10000 = 0U;
}

uint32_t app_clk_measure_k_x10000(void)
{
    return g_clkmeas_k_x10000;
}

uint32_t app_clk_measure_frames(void)
{
    return g_clkmeas_frames;
}

/* Called from the USART1 receive ISR for every byte, so the timestamps carry
 * only interrupt-entry jitter and not main-loop jitter. */
static void app_clk_measure_byte(void)
{
    uint32_t now;
    uint64_t den;

    if (g_clkmeas_ready == 0U) {
        return;
    }

    now = DWT->CYCCNT;

    /* A gap longer than one frame means a new burst: a UART physically cannot
     * leave a gap below 10 bit times inside one continuous send. */
    if ((g_clkmeas_frames == 0U) || ((now - g_clkmeas_last) > APP_CLK_MEAS_GAP_CYCLES)) {
        g_clkmeas_first = now;
        g_clkmeas_frames = 1U;
    } else {
        g_clkmeas_frames++;
    }
    g_clkmeas_last = now;

    if (g_clkmeas_frames >= APP_CLK_MEAS_MIN_FRAMES) {
        /* the span covers (frames - 1) whole frame periods */
        den = ((uint64_t)(g_clkmeas_frames - 1U)) * APP_CLK_MEAS_DIVISOR;
        if (den != 0ULL) {
            uint32_t k = (uint32_t)(((uint64_t)(g_clkmeas_last - g_clkmeas_first) * 10000ULL) / den);
            /* accept 0.25x .. 4x only; a dead CYCCNT would give 0 and must not
             * overwrite a previous good result */
            if ((k >= 2500U) && (k <= 40000U)) {
                g_clkmeas_k_x10000 = k;
            }
        }
    }
}
#endif /* APP_CLK_MEASURE_ENABLE */
#endif /* APP_UART1_BAUD_SWEEP_ENABLE */

/*
 * If our busy flag says a transfer is running but the HAL reports no TX in
 * progress, the completion callback was lost (or a transmit error left the
 * state machine stuck).  Release the ring so logging recovers by itself
 * instead of going permanently silent.
 *
 * g_tx_started guards the publish/start window: until HAL_UART_Transmit_IT()
 * has accepted the transfer the HAL legitimately reports READY, and clearing
 * the length there would corrupt the ring.
 */
static void app_log_tx_resync_if_stalled(void)
{
    uint32_t primask;

    if ((g_tx_busy == 0U) || (g_tx_started == 0U) ||
        (huart1.gState == HAL_UART_STATE_BUSY_TX)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    g_tx_busy = 0U;
    g_tx_started = 0U;
    g_tx_len = 0U;
    __set_PRIMASK(primask);
}

/* Push the next contiguous chunk of the TX ring out.  Called from the main
 * loop and from HAL_UART_TxCpltCallback. */
static void app_log_tx_kick(void)
{
    uint16_t len;
    uint32_t primask;

    if (g_log_ready == 0U) {
        return;
    }

    app_log_tx_resync_if_stalled();

    if ((g_tx_busy != 0U) || (g_tx_head == g_tx_tail)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((g_tx_busy != 0U) || (g_tx_head == g_tx_tail)) {
        __set_PRIMASK(primask);
        return;
    }
    /* Send up to the end of the ring; the completion callback wraps and
     * re-kicks, so one transfer is at most two ISR round trips. */
    len = (uint16_t)((g_tx_head > g_tx_tail)
                     ? (uint16_t)(g_tx_head - g_tx_tail)
                     : (uint16_t)(APP_LOG_TX_RING_LEN - g_tx_tail));
    g_tx_busy = 1U;
    /* Not yet handed to the HAL, so resync must leave this alone. */
    g_tx_started = 0U;
    g_tx_len = len;
    __set_PRIMASK(primask);

    if (HAL_UART_Transmit_IT(&huart1, &g_tx_ring[g_tx_tail], len) != HAL_OK) {
        primask = __get_PRIMASK();
        __disable_irq();
        g_tx_busy = 0U;
        g_tx_started = 0U;
        g_tx_len = 0U;
        __set_PRIMASK(primask);
        return;
    }

    /* Ownership transferred.  If the callback already ran and finished this
     * chunk (possible for very short transfers) g_tx_busy is back to 0 and
     * there is nothing to mark. */
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_tx_busy != 0U) {
        g_tx_started = 1U;
    }
    __set_PRIMASK(primask);
}

void app_log_init(void)
{
    g_log_ready = 1U;
#if APP_UART1_BAUD_SWEEP_REPEAT_ENABLE
    /* The repeating sweep needs the port to itself.  Set
     * APP_UART1_BAUD_SWEEP_REPEAT_ENABLE to 0U to restore logging. */
    g_log_muted = 1U;
#else
    g_log_muted = 0U;
#endif
    g_rx_head = 0U;
    g_rx_tail = 0U;
    g_tx_head = 0U;
    g_tx_tail = 0U;
    g_tx_len = 0U;
    g_tx_busy = 0U;
    g_tx_started = 0U;
    g_tx_drop_count = 0UL;
    g_log_flags.gpsdo2 = GPSDO_LOG_GPSDO2_ENABLE;
    g_log_flags.legacy_gpsdo10 = GPSDO_LOG_LEGACY_GPSDO10_ENABLE;
    g_log_flags.rtc_phase = GPSDO_LOG_RTC_PHASE_ENABLE;
    g_log_flags.phase10 = GPSDO_LOG_PHASE10_ENABLE;
    g_log_flags.event = GPSDO_LOG_EVENT_ENABLE;
    g_log_flags.dac_adjust = GPSDO_LOG_DAC_ADJUST_ENABLE;
    g_log_flags.gpsdo_phase = GPSDO_LOG_GPSDO_PHASE_ENABLE;
    g_log_flags.gpsdo_avg   = GPSDO_LOG_GPSDO_AVG_ENABLE;
    g_log_flags.result      = GPSDO_LOG_RESULT_ENABLE;
    app_uart_debug_start_rx();
}

uint32_t app_log_get_tx_drops(void)
{
    return g_tx_drop_count;
}

void app_log_write(const char *text)
{
    size_t len;
    uint16_t head;
    uint16_t tail;
    uint16_t used;
    uint16_t first;
    uint16_t n;
    uint32_t primask;

    if ((g_log_ready == 0U) || (g_log_muted != 0U) || (text == (const char *)0)) {
        return;
    }

    len = strlen(text);
    if (len == 0U) {
        return;
    }
    if (len > (size_t)(APP_LOG_TX_RING_LEN - 1U)) {
        len = (size_t)(APP_LOG_TX_RING_LEN - 1U);   /* one write must fit the ring */
    }
    n = (uint16_t)len;

    /* Claim space and copy while interrupts are masked, so the consumer can
     * never observe a half written record: the head is published last. */
    primask = __get_PRIMASK();
    __disable_irq();

    head = g_tx_head;
    tail = g_tx_tail;
    used = (uint16_t)((head >= tail) ? (uint16_t)(head - tail)
                                     : (uint16_t)(APP_LOG_TX_RING_LEN - tail + head));

    if ((uint16_t)(APP_LOG_TX_RING_LEN - 1U - used) < n) {
        g_tx_drop_count++;      /* drop whole line, never block the caller */
        __set_PRIMASK(primask);
        return;
    }

    first = (uint16_t)(APP_LOG_TX_RING_LEN - head);
    if (first > n) {
        first = n;
    }
    memcpy(&g_tx_ring[head], text, (size_t)first);
    if (n > first) {
        memcpy(&g_tx_ring[0], text + first, (size_t)(n - first));
    }

    head = (uint16_t)(head + n);
    if (head >= APP_LOG_TX_RING_LEN) {
        head = (uint16_t)(head - APP_LOG_TX_RING_LEN);
    }
    g_tx_head = head;

    __set_PRIMASK(primask);

    app_log_tx_kick();
}

void app_log_printf(const char *fmt, ...)
{
    va_list ap;
    int n;

    if ((g_log_ready == 0U) || (fmt == (const char *)0)) {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(g_log_buf, sizeof(g_log_buf), fmt, ap);
    va_end(ap);

    if (n <= 0) {
        return;
    }

    g_log_buf[sizeof(g_log_buf) - 1U] = '\0';
    app_log_write(g_log_buf);
}

uint8_t app_uart_debug_get_char(uint8_t *ch)
{
    if (ch == (uint8_t *)0) {
        return 0U;
    }

    __disable_irq();
    if (g_rx_head == g_rx_tail) {
        __enable_irq();
        return 0U;
    }

    *ch = g_rx_ring[g_rx_tail];
    g_rx_tail++;
    if (g_rx_tail >= APP_UART_RX_RING_LEN) {
        g_rx_tail = 0U;
    }
    __enable_irq();

    return 1U;
}

static uint8_t app_log_bool(uint8_t enable)
{
    return (enable != 0U) ? 1U : 0U;
}

void app_log_get_flags(app_log_flags_t *flags)
{
    if (flags != (app_log_flags_t *)0) {
        *flags = g_log_flags;
    }
}

void app_log_set_rtc_phase(uint8_t enable)
{
    g_log_flags.rtc_phase = app_log_bool(enable);
}

void app_log_set_phase10(uint8_t enable)
{
    g_log_flags.phase10 = app_log_bool(enable);
}

void app_log_set_phase_detail(uint8_t enable)
{
    app_log_set_rtc_phase(enable);
    app_log_set_phase10(enable);
}

void app_log_set_legacy_gpsdo10(uint8_t enable)
{
    g_log_flags.legacy_gpsdo10 = app_log_bool(enable);
}

void app_log_set_event(uint8_t enable)
{
    g_log_flags.event = app_log_bool(enable);
}

void app_log_set_dac_adjust(uint8_t enable)
{
    g_log_flags.dac_adjust = app_log_bool(enable);
}

void app_log_set_result(uint8_t enable)
{
    g_log_flags.result = app_log_bool(enable);
}

uint8_t app_log_result_enabled(void)
{
    return g_log_flags.result;
}

uint8_t app_log_gpsdo2_enabled(void)
{
    return g_log_flags.gpsdo2;
}

uint8_t app_log_legacy_gpsdo10_enabled(void)
{
    return g_log_flags.legacy_gpsdo10;
}

uint8_t app_log_rtc_phase_enabled(void)
{
    return g_log_flags.rtc_phase;
}

uint8_t app_log_phase10_enabled(void)
{
    return g_log_flags.phase10;
}

uint8_t app_log_event_enabled(void)
{
    return g_log_flags.event;
}

uint8_t app_log_dac_adjust_enabled(void)
{
    return g_log_flags.dac_adjust;
}

uint8_t app_log_gpsdo_phase_enabled(void)
{
    return g_log_flags.gpsdo_phase;
}

uint8_t app_log_gpsdo_avg_enabled(void)
{
    return g_log_flags.gpsdo_avg;
}

void app_log_print_voltage_uv(uint32_t uv)
{
    app_log_printf("%lu.%04lu",
                   (unsigned long)(uv / 1000000UL),
                   (unsigned long)((uv % 1000000UL) / 100UL));
}

void app_log_print_voltage_uv6(uint32_t uv)
{
    app_log_printf("%lu.%06lu",
                   (unsigned long)(uv / 1000000UL),
                   (unsigned long)(uv % 1000000UL));
}

uint32_t app_log_dac_code_to_uv(uint16_t code)
{
    uint64_t num = ((uint64_t)code * (uint64_t)DAC_VREF_UV) + 32767ULL;
    return (uint32_t)(num / 65535ULL);
}

void app_log_dac_adjust_event(const char *source, uint16_t from_code, uint16_t to_code,
                               uint32_t from_uv, uint32_t to_uv, const char *reason)
{
    int32_t step_uv;

    if ((app_log_event_enabled() == 0U) || (app_log_dac_adjust_enabled() == 0U)) {
        return;
    }

    step_uv = (int32_t)((int64_t)to_uv - (int64_t)from_uv);
    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=DAC_ADJUST,SOURCE=%s,FROM_CODE=%lu,TO_CODE=%lu,FROM_V=",
                   (unsigned long)HAL_GetTick(),
                   (source != (const char *)0) ? source : "?",
                   (unsigned long)from_code,
                   (unsigned long)to_code);
    app_log_print_voltage_uv6(from_uv);
    app_log_write(",TO_V=");
    app_log_print_voltage_uv6(to_uv);
    app_log_printf(",STEP_UV=%ld,REASON=%s\r\n",
                   (long)step_uv,
                   (reason != (const char *)0) ? reason : "?");
}

void app_log_dac_adjust_fail_event(const char *source, uint16_t from_code, uint16_t to_code,
                                   const char *reason, HAL_StatusTypeDef status)
{
    if ((app_log_event_enabled() == 0U) || (app_log_dac_adjust_enabled() == 0U)) {
        return;
    }

    app_log_printf("GPSDO_EVT,MS=%lu,TYPE=DAC_ADJUST_FAIL,SOURCE=%s,FROM_CODE=%lu,TO_CODE=%lu,REASON=%s,STATUS=%lu\r\n",
                   (unsigned long)HAL_GetTick(),
                   (source != (const char *)0) ? source : "?",
                   (unsigned long)from_code,
                   (unsigned long)to_code,
                   (reason != (const char *)0) ? reason : "?",
                   (unsigned long)status);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if ((huart == (UART_HandleTypeDef *)0) || (huart->Instance != USART1)) {
        return;
    }

#if APP_CLK_MEASURE_ENABLE
    /* Timestamp before the ring push: this is the only place that sees every
     * arriving byte at a fixed point in the ISR. */
    app_clk_measure_byte();
#endif

    next = (uint16_t)(g_rx_head + 1U);
    if (next >= APP_UART_RX_RING_LEN) {
        next = 0U;
    }

    if (next != g_rx_tail) {
        g_rx_ring[g_rx_head] = g_rx_byte;
        g_rx_head = next;
    }

    app_uart_debug_start_rx();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t tail;
    uint32_t primask;

    if ((huart == (UART_HandleTypeDef *)0) || (huart->Instance != USART1)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    tail = (uint16_t)(g_tx_tail + g_tx_len);
    if (tail >= APP_LOG_TX_RING_LEN) {
        tail = (uint16_t)(tail - APP_LOG_TX_RING_LEN);
    }
    g_tx_tail = tail;
    g_tx_busy = 0U;
    g_tx_started = 0U;
    g_tx_len = 0U;
    __set_PRIMASK(primask);

    app_log_tx_kick();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == (UART_HandleTypeDef *)0) {
        return;
    }

    if (huart->Instance == USART2) {
        /*
         * The GPS receiver.  HAL_UART_IRQHandler() has already aborted the DMA
         * transfer and set RxState back to READY, so there is nothing useful to
         * do here: re-arming needs HAL_UART_DMAStop()/HAL_UART_Receive_DMA(),
         * which must not run in interrupt context.  The watchdog in
         * app_gps_uart_poll() detects RxState != BUSY_RX within 20 ms and does
         * the restart; see app_gps_uart_recover().
         */
        return;
    }

    if (huart->Instance != USART1) {
        return;
    }

    /* A receive error must not leave the transmit ring stuck; resync only
     * touches the flags when the HAL really has no TX in flight. */
    app_log_tx_resync_if_stalled();

    app_uart_debug_start_rx();
    app_log_tx_kick();
}
