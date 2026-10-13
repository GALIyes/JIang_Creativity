#include "app_tmp117.h"

#include "i2c.h"

#define TMP117_REG_TEMP       0x00U
#define TMP117_I2C_TIMEOUT_MS 20U

static uint8_t g_tmp117_present;

HAL_StatusTypeDef tmp117_init(void)
{
    HAL_StatusTypeDef st;

    st = HAL_I2C_IsDeviceReady(&hi2c2, TMP117_ADDR_HAL, 2U, TMP117_I2C_TIMEOUT_MS);
    g_tmp117_present = (st == HAL_OK) ? 1U : 0U;
    return st;
}

HAL_StatusTypeDef tmp117_read_celsius(float *temp_c)
{
    uint8_t rx[2];
    int16_t raw;
    HAL_StatusTypeDef st;

    if ((temp_c == (float *)0) || (g_tmp117_present == 0U)) {
        return HAL_ERROR;
    }

    st = HAL_I2C_Mem_Read(&hi2c2, TMP117_ADDR_HAL, TMP117_REG_TEMP,
                          I2C_MEMADD_SIZE_8BIT, rx, sizeof(rx),
                          TMP117_I2C_TIMEOUT_MS);
    if (st != HAL_OK) {
        return st;
    }

    raw = (int16_t)(((uint16_t)rx[0] << 8) | (uint16_t)rx[1]);
    *temp_c = (float)raw * 0.0078125f;
    return HAL_OK;
}

uint8_t tmp117_is_present(void)
{
    return g_tmp117_present;
}
