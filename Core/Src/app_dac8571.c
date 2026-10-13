#include "app_dac8571.h"

#include "i2c.h"

static uint16_t g_last_dac_code = DAC_CODE_INIT;

void app_dac8571_init(void)
{
    g_last_dac_code = DAC_CODE_INIT;
}

HAL_StatusTypeDef app_dac8571_write_raw(uint16_t code)
{
    return app_dac8571_write_raw_to_addr(DAC8571_ADDR_7BIT, code);
}

HAL_StatusTypeDef app_dac8571_write_code(uint16_t code)
{
    return app_dac8571_write_raw(code);
}

HAL_StatusTypeDef app_dac8571_write_raw_to_addr(uint8_t addr7, uint16_t code)
{
    uint8_t tx[3];
    HAL_StatusTypeDef st;

    tx[0] = DAC8571_CONTROL_WRITE_UPDATE;
    tx[1] = (uint8_t)(code >> 8);
    tx[2] = (uint8_t)(code & 0xFFU);

    st = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)((uint16_t)addr7 << 1), tx, sizeof(tx), DAC8571_I2C_TIMEOUT_MS);
    if (st == HAL_OK) {
        g_last_dac_code = code;
    }

    return st;
}

uint16_t app_dac8571_uv_to_code(uint32_t uv)
{
    uint64_t num;

    if (uv >= DAC_VREF_UV) {
        return 65535U;
    }

    num = ((uint64_t)uv * 65535ULL) + (uint64_t)(DAC_VREF_UV / 2UL);
    return (uint16_t)(num / (uint64_t)DAC_VREF_UV);
}

uint16_t app_dac8571_voltage_to_code(uint32_t uv)
{
    return app_dac8571_uv_to_code(uv);
}

HAL_StatusTypeDef app_dac8571_write_voltage_mV(uint16_t mv)
{
    uint32_t uv = (uint32_t)mv * 1000UL;
    return app_dac8571_write_raw(app_dac8571_uv_to_code(uv));
}

HAL_StatusTypeDef app_dac8571_set_voltage_uv(uint32_t uv)
{
    return app_dac8571_write_raw(app_dac8571_uv_to_code(uv));
}

HAL_StatusTypeDef app_dac8571_set_initial_near_lock(void)
{
    return app_dac8571_write_raw(DAC_CODE_INIT);
}

uint16_t app_dac8571_get_last_code(void)
{
    return g_last_dac_code;
}
