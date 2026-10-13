#include "app_cpld_spi.h"

#include "spi.h"
#include "app_uart_debug.h"

#define CPLD_SPI_TIMEOUT_MS          20U
#define CPLD_SPI_CS_SETUP_US          3U
#define CPLD_SPI_CS_HOLD_US           3U
#define CPLD_SPI_CS_IDLE_US           8U
#define CPLD_SPI_BSY_TIMEOUT_LOOPS 10000UL
#define CPLD_FRAME_MARKER_MASK       0x00000001UL
#define CPLD_FRAME_COUNT_MASK        0x07FFFFFFUL
#define CPLD_STABLE_PAIR_RETRIES      3U

typedef struct {
    uint32_t raw;
    uint32_t count10;
    uint8_t seq4;
    uint8_t seq8;
    uint8_t valid;
} app_cpld_frame_t;

static app_cpld_frame_t g_frame;
static uint8_t g_seq4_last;
static uint8_t g_seq4_valid;
static uint8_t g_clear_count;
static uint32_t g_last_window_seq_raw;
static uint32_t g_dummy_signature;
static uint8_t g_last_stable_retry_count;
static app_cpld_signature_stable_t g_last_sig_stable;

static void app_cpld_delay_us(uint32_t us)
{
    volatile uint32_t i;
    uint32_t loops = (SystemCoreClock / 1000000UL) * us;
    if (loops == 0UL) {
        loops = us;
    }
    for (i = 0U; i < loops; i++) {
        __NOP();
    }
}

static void app_cpld_cs_low(void)
{
    HAL_GPIO_WritePin(CPLD_CS_GPIO_Port, CPLD_CS_Pin, GPIO_PIN_RESET);
}

static void app_cpld_cs_high(void)
{
    HAL_GPIO_WritePin(CPLD_CS_GPIO_Port, CPLD_CS_Pin, GPIO_PIN_SET);
}

static void app_cpld_wait_not_busy(void)
{
    uint32_t guard = CPLD_SPI_BSY_TIMEOUT_LOOPS;
    while ((__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET) && (guard > 0UL)) {
        guard--;
    }
}

static HAL_StatusTypeDef app_cpld_read_raw_frame(uint32_t *raw_out)
{
    uint8_t tx[5] = {0x00U, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    uint8_t rx[5] = {0U, 0U, 0U, 0U, 0U};
    uint32_t raw;
    HAL_StatusTypeDef st;

    if (raw_out == (uint32_t *)0) {
        return HAL_ERROR;
    }

    app_cpld_cs_high();
    app_cpld_delay_us(CPLD_SPI_CS_IDLE_US);
    app_cpld_cs_low();
    app_cpld_delay_us(CPLD_SPI_CS_SETUP_US);
    st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, sizeof(tx), CPLD_SPI_TIMEOUT_MS);
    app_cpld_wait_not_busy();
    app_cpld_delay_us(CPLD_SPI_CS_HOLD_US);
    app_cpld_cs_high();
    app_cpld_delay_us(CPLD_SPI_CS_IDLE_US);
    if (st != HAL_OK) {
        return st;
    }

    raw = ((uint32_t)rx[1] << 24) |
          ((uint32_t)rx[2] << 16) |
          ((uint32_t)rx[3] << 8) |
          ((uint32_t)rx[4]);
    if ((raw & CPLD_FRAME_MARKER_MASK) == 0UL) {
        return HAL_ERROR;
    }
    *raw_out = raw;
    return HAL_OK;
}

static HAL_StatusTypeDef app_cpld_read_frame(app_cpld_frame_t *out)
{
    uint32_t raw1 = 0UL;
    uint32_t raw2 = 0UL;
    uint8_t seq4;
    uint8_t delta;
    uint8_t attempt;
    HAL_StatusTypeDef st1;
    HAL_StatusTypeDef st2;

    if (out == (app_cpld_frame_t *)0) {
        return HAL_ERROR;
    }

    g_last_sig_stable.sig1 = 0UL;
    g_last_sig_stable.sig2 = 0UL;
    g_last_sig_stable.st1 = HAL_ERROR;
    g_last_sig_stable.st2 = HAL_ERROR;
    g_last_sig_stable.retry = 0U;
    g_last_sig_stable.retry_count = 0U;
    g_last_sig_stable.shifted_count = 0U;
    g_last_stable_retry_count = 0U;

    for (attempt = 0U; attempt < CPLD_STABLE_PAIR_RETRIES; attempt++) {
        st1 = app_cpld_read_raw_frame(&raw1);
        st2 = app_cpld_read_raw_frame(&raw2);
        g_last_sig_stable.sig1 = (st1 == HAL_OK) ? CPLD_SIGNATURE_VALUE : 0UL;
        g_last_sig_stable.sig2 = (st2 == HAL_OK) ? CPLD_SIGNATURE_VALUE : 0UL;
        g_last_sig_stable.st1 = st1;
        g_last_sig_stable.st2 = st2;
        g_last_sig_stable.retry = (attempt != 0U) ? 1U : 0U;
        g_last_sig_stable.retry_count = attempt;

        if ((st1 != HAL_OK) || (st2 != HAL_OK) || (raw1 != raw2)) {
            continue;
        }

        seq4 = (uint8_t)((raw1 >> 28) & 0x0FU);
        if (g_seq4_valid == 0U) {
            g_frame.seq8 = seq4;
            g_seq4_valid = 1U;
        } else {
            delta = (uint8_t)((seq4 - g_seq4_last) & 0x0FU);
            g_frame.seq8 = (uint8_t)(g_frame.seq8 + delta);
        }
        g_seq4_last = seq4;

        g_frame.raw = raw1;
        g_frame.seq4 = seq4;
        g_frame.count10 = (raw1 >> 1) & CPLD_FRAME_COUNT_MASK;
        g_frame.valid = 1U;
        g_last_stable_retry_count = attempt;
        *out = g_frame;
        return HAL_OK;
    }

    out->valid = 0U;
    return HAL_ERROR;
}

void app_cpld_spi_init(void)
{
    app_cpld_frame_t frame;
    g_frame.raw = 0UL;
    g_frame.count10 = 0UL;
    g_frame.seq4 = 0U;
    g_frame.seq8 = 0U;
    g_frame.valid = 0U;
    g_seq4_last = 0U;
    g_seq4_valid = 0U;
    g_clear_count = 0U;
    g_last_window_seq_raw = 0UL;
    app_cpld_cs_high();
    g_last_stable_retry_count = 0U;
    g_dummy_signature = (app_cpld_read_frame(&frame) == HAL_OK)
                      ? CPLD_SIGNATURE_VALUE : 0UL;
}

uint32_t app_cpld_get_dummy_signature(void)
{
    return g_dummy_signature;
}

HAL_StatusTypeDef app_cpld_read_measurement(app_cpld_measurement_t *out)
{
    app_cpld_frame_t frame;
    uint32_t status;

    if (out == (app_cpld_measurement_t *)0) {
        return HAL_ERROR;
    }

    out->raw = 0UL;
    out->signature = 0UL;
    out->status = 0UL;
    out->count10 = 0UL;
    out->window_seq_raw = 0UL;
    out->seq10 = 0U;
    out->seq50 = 0U;
    out->clr10_cnt = 0U;
    out->clr50_cnt = 0U;
    out->stable_retry_count = 0U;

    if (app_cpld_read_frame(&frame) != HAL_OK) {
        return HAL_ERROR;
    }

    status = CPLD_STATUS_ALIVE_MASK | CPLD_STATUS_PPS_SYNC_MASK;
    if (frame.count10 != 0UL) {
        status |= CPLD_STATUS_FINISH_10S_MASK;
    }
    if ((frame.seq8 & 1U) != 0U) {
        status |= CPLD_STATUS_PPS_TOGGLE_MASK;
    }

    out->raw = frame.raw;
    out->signature = CPLD_SIGNATURE_VALUE;
    out->status = status;
    out->count10 = frame.count10;
    out->seq10 = frame.seq8;
    out->seq50 = (uint8_t)(frame.seq8 / GPSDO_SW_E50_WINDOWS);
    out->clr10_cnt = g_clear_count;
    out->clr50_cnt = g_clear_count;
    out->window_seq_raw = ((uint32_t)out->clr50_cnt << 24) |
                          ((uint32_t)out->clr10_cnt << 16) |
                          ((uint32_t)out->seq50 << 8) |
                          out->seq10;
    out->stable_retry_count = g_last_stable_retry_count;
    g_last_window_seq_raw = out->window_seq_raw;
    return HAL_OK;
}
HAL_StatusTypeDef app_cpld_read_reg32(uint8_t reg, uint32_t *value)
{
    app_cpld_frame_t frame;
    HAL_StatusTypeDef st;
    uint32_t status;

    if (value == (uint32_t *)0) {
        return HAL_ERROR;
    }
    st = app_cpld_read_frame(&frame);
    if (st != HAL_OK) {
        return st;
    }

    status = CPLD_STATUS_ALIVE_MASK | CPLD_STATUS_PPS_SYNC_MASK;
    if (frame.count10 != 0UL) {
        status |= CPLD_STATUS_FINISH_10S_MASK;
    }
    if ((frame.seq8 & 1U) != 0U) {
        status |= CPLD_STATUS_PPS_TOGGLE_MASK;
    }

    switch (reg) {
    case CPLD_REG_COUNT10:
        *value = frame.count10;
        break;
    case CPLD_REG_COUNT50:
        *value = 0UL;
        break;
    case CPLD_REG_STATUS:
        *value = status;
        break;
    case CPLD_REG_SIGNATURE:
        *value = CPLD_SIGNATURE_VALUE;
        break;
    case CPLD_REG_WINDOW_SEQ:
        *value = ((uint32_t)g_clear_count << 24) |
                 ((uint32_t)g_clear_count << 16) |
                 ((uint32_t)(frame.seq8 / GPSDO_SW_E50_WINDOWS) << 8) |
                 frame.seq8;
        g_last_window_seq_raw = *value;
        break;
    case CPLD_REG_CTRL_STATUS:
        *value = ((uint32_t)g_clear_count << 16) |
                 ((uint32_t)g_clear_count << 8);
        break;
    default:
        *value = 0UL;
        break;
    }
    return HAL_OK;
}

HAL_StatusTypeDef app_cpld_write_reg32(uint8_t reg, uint32_t value)
{
    (void)value;
    if (reg == CPLD_REG_CTRL) {
        g_clear_count++;
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef app_cpld_read_signature(uint32_t *value)
{
    return app_cpld_read_reg32(CPLD_REG_SIGNATURE, value);
}

HAL_StatusTypeDef app_cpld_read_signature_stable(uint32_t *value)
{
    HAL_StatusTypeDef st;
    uint8_t i;

    if (value == (uint32_t *)0) {
        return HAL_ERROR;
    }
    g_last_sig_stable.sig1 = 0UL;
    g_last_sig_stable.sig2 = 0UL;
    g_last_sig_stable.st1 = HAL_ERROR;
    g_last_sig_stable.st2 = HAL_ERROR;
    g_last_sig_stable.retry = 0U;
    g_last_sig_stable.retry_count = 0U;
    g_last_sig_stable.shifted_count = 0U;

    for (i = 0U; i < 5U; i++) {
        st = app_cpld_read_signature(value);
        if (i == 0U) {
            g_last_sig_stable.sig1 = *value;
            g_last_sig_stable.st1 = st;
        }
        g_last_sig_stable.sig2 = *value;
        g_last_sig_stable.st2 = st;
        g_last_sig_stable.retry = (i == 0U) ? 0U : 1U;
        g_last_sig_stable.retry_count = i;
        if ((st == HAL_OK) && (*value == CPLD_SIGNATURE_VALUE)) {
            return HAL_OK;
        }
    }
    return HAL_ERROR;
}

const app_cpld_signature_stable_t *app_cpld_get_last_signature_stable(void)
{
    return &g_last_sig_stable;
}

HAL_StatusTypeDef app_cpld_read_status(uint32_t *value)
{
    return app_cpld_read_reg32(CPLD_REG_STATUS, value);
}

HAL_StatusTypeDef app_cpld_read_count10(uint32_t *value)
{
    return app_cpld_read_reg32(CPLD_REG_COUNT10, value);
}

HAL_StatusTypeDef app_cpld_read_count50(uint32_t *value)
{
    return app_cpld_read_reg32(CPLD_REG_COUNT50, value);
}

HAL_StatusTypeDef app_cpld_read_window_seq(uint8_t *seq10, uint8_t *seq50,
                                           uint8_t *clr10_cnt, uint8_t *clr50_cnt)
{
    uint32_t value;
    HAL_StatusTypeDef st = app_cpld_read_reg32(CPLD_REG_WINDOW_SEQ, &value);
    if (st != HAL_OK) {
        return st;
    }
    if (seq10 != (uint8_t *)0) *seq10 = (uint8_t)value;
    if (seq50 != (uint8_t *)0) *seq50 = (uint8_t)(value >> 8);
    if (clr10_cnt != (uint8_t *)0) *clr10_cnt = (uint8_t)(value >> 16);
    if (clr50_cnt != (uint8_t *)0) *clr50_cnt = (uint8_t)(value >> 24);
    return HAL_OK;
}

uint32_t app_cpld_get_last_window_seq_raw(void)
{
    return g_last_window_seq_raw;
}

HAL_StatusTypeDef app_cpld_clear_measurement_windows(void)
{
    // The compact CPLD has no write path.  Warm-up logic already ignores the
    // current frame and starts on the following sequence, so a software ACK
    // is sufficient and avoids roughly 15 additional MAX V logic cells.
    g_clear_count++;
    return HAL_OK;
}

HAL_StatusTypeDef app_cpld_read_phase_ticks(int32_t *ticks)
{
    if (ticks != (int32_t *)0) *ticks = 0;
    return HAL_ERROR;
}

HAL_StatusTypeDef app_cpld_read_phase_seq(uint32_t *seq)
{
    if (seq != (uint32_t *)0) *seq = 0UL;
    return HAL_ERROR;
}

HAL_StatusTypeDef app_cpld_read_phase_status(uint32_t *status)
{
    if (status != (uint32_t *)0) *status = 0UL;
    return HAL_ERROR;
}

HAL_StatusTypeDef app_cpld_read_rtc_debug(uint32_t *debug)
{
    if (debug != (uint32_t *)0) *debug = 0UL;
    return HAL_ERROR;
}

HAL_StatusTypeDef app_cpld_write_rtc_phase_step_raw(int32_t step_count)
{
    (void)step_count;
    return HAL_ERROR;
}

HAL_StatusTypeDef app_cpld_read_phase(app_cpld_phase_t *out)
{
    if (out != (app_cpld_phase_t *)0) {
        out->phase_ticks = 0;
        out->phase_seq = 0UL;
        out->phase_status = 0UL;
        out->rtc_debug = 0UL;
        out->valid_read = 0U;
    }
    return HAL_ERROR;
}

void app_cpld_spi_diag_raw_read(uint8_t addr)
{
    app_cpld_frame_t frame = {0};
    HAL_StatusTypeDef st = app_cpld_read_frame(&frame);
    app_log_printf("MAXV_FRAME,REQ=0x%02lX,RAW=0x%08lX,SEQ4=%lu,SEQ8=%lu,COUNT10=%lu,HAL=%lu\r\n",
                   (unsigned long)addr,
                   (unsigned long)frame.raw,
                   (unsigned long)frame.seq4,
                   (unsigned long)frame.seq8,
                   (unsigned long)frame.count10,
                   (unsigned long)st);
}




