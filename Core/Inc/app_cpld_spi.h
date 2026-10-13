#ifndef APP_CPLD_SPI_H
#define APP_CPLD_SPI_H

#include "app_config.h"
#include "main.h"

#define CPLD_REG_COUNT1        0x01U
#define CPLD_REG_COUNT10       0x02U
#define CPLD_REG_COUNT50       0x03U
#define CPLD_REG_STATUS        0x04U
#define CPLD_REG_SIGNATURE     0x05U
#define CPLD_REG_DEBUG_CNT     0x06U
#define CPLD_REG_PPS_DEBUG     0x07U
#define CPLD_REG_CTRL          0x0FU
#define CPLD_REG_CTRL_STATUS   0x10U
#define CPLD_REG_WINDOW_SEQ    0x11U
#define CPLD_REG_RTC_PHASE_TICKS  0x12U
#define CPLD_REG_RTC_PHASE_SEQ    0x13U
#define CPLD_REG_RTC_PHASE_STATUS 0x14U
#define CPLD_REG_RTC_DEBUG        0x15U
#define CPLD_REG_RTC_PHASE_STEP   GPSDO_RTC_PHASE_STEP_REG

#define CPLD_CTRL_CLR_10S      0x00000001UL
#define CPLD_CTRL_CLR_50S      0x00000002UL
#define CPLD_CTRL_CLR_MEAS_ALL (CPLD_CTRL_CLR_10S | CPLD_CTRL_CLR_50S)

#define CPLD_SIGNATURE_VALUE   0xA5A55A5AUL
#define CPLD_SIGNATURE_SHIFTED_VALUE 0x4B4AB4B4UL

#define CPLD_STATUS_FINISH_1S_MASK   0x00000001UL
#define CPLD_STATUS_FINISH_10S_MASK  0x00000002UL
#define CPLD_STATUS_FINISH_50S_MASK  0x00000004UL
#define CPLD_STATUS_PPS_SYNC_MASK    0x00000008UL
#define CPLD_STATUS_PPS_TOGGLE_MASK  0x00000010UL
#define CPLD_STATUS_ALIVE_MASK       0x00000020UL

#define CPLD_RTC_PHASE_VALID_MASK       0x00000001UL
#define CPLD_RTC_PHASE_GPS_SEEN_MASK    0x00000002UL
#define CPLD_RTC_PHASE_RTC_SEEN_MASK    0x00000004UL
#define CPLD_RTC_PHASE_MISSING_RTC_MASK 0x00000008UL
#define CPLD_RTC_PHASE_OVERFLOW_MASK    0x00000010UL
#define CPLD_RTC_PHASE_RTC_LEVEL_MASK   0x00000020UL
#define CPLD_RTC_PHASE_GPS_LEVEL_MASK   0x00000040UL
#define CPLD_RTC_PHASE_ALIVE_MASK       0x00000080UL

typedef struct {
    uint32_t sig1;
    uint32_t sig2;
    HAL_StatusTypeDef st1;
    HAL_StatusTypeDef st2;
    uint8_t retry;
    uint8_t retry_count;
    uint8_t shifted_count;
} app_cpld_signature_stable_t;

typedef struct {
    int32_t phase_ticks;
    uint32_t phase_seq;
    uint32_t phase_status;
    uint32_t rtc_debug;
    uint8_t valid_read;
} app_cpld_phase_t;
/* One control-loop sample decoded from one atomically accepted MAX V frame. */
typedef struct {
    uint32_t raw;
    uint32_t signature;
    uint32_t status;
    uint32_t count10;
    uint32_t window_seq_raw;
    uint8_t seq10;
    uint8_t seq50;
    uint8_t clr10_cnt;
    uint8_t clr50_cnt;
    uint8_t stable_retry_count;
} app_cpld_measurement_t;

void app_cpld_spi_init(void);
uint32_t app_cpld_get_dummy_signature(void);
HAL_StatusTypeDef app_cpld_read_measurement(app_cpld_measurement_t *out);
HAL_StatusTypeDef app_cpld_read_reg32(uint8_t reg, uint32_t *value);
HAL_StatusTypeDef app_cpld_write_reg32(uint8_t reg, uint32_t value);
HAL_StatusTypeDef app_cpld_read_signature(uint32_t *value);
HAL_StatusTypeDef app_cpld_read_signature_stable(uint32_t *value);
const app_cpld_signature_stable_t *app_cpld_get_last_signature_stable(void);
HAL_StatusTypeDef app_cpld_read_status(uint32_t *value);
HAL_StatusTypeDef app_cpld_read_count10(uint32_t *value);
HAL_StatusTypeDef app_cpld_read_count50(uint32_t *value);
HAL_StatusTypeDef app_cpld_read_window_seq(uint8_t *seq10, uint8_t *seq50, uint8_t *clr10_cnt, uint8_t *clr50_cnt);
uint32_t app_cpld_get_last_window_seq_raw(void);
HAL_StatusTypeDef app_cpld_clear_measurement_windows(void);
HAL_StatusTypeDef app_cpld_read_phase_ticks(int32_t *ticks);
HAL_StatusTypeDef app_cpld_read_phase_seq(uint32_t *seq);
HAL_StatusTypeDef app_cpld_read_phase_status(uint32_t *status);
HAL_StatusTypeDef app_cpld_read_rtc_debug(uint32_t *debug);
HAL_StatusTypeDef app_cpld_write_rtc_phase_step_raw(int32_t step_count);
HAL_StatusTypeDef app_cpld_read_phase(app_cpld_phase_t *out);

void app_cpld_spi_diag_raw_read(uint8_t addr);

#endif /* APP_CPLD_SPI_H */

