#ifndef APP_DAC8571_H
#define APP_DAC8571_H

#include "main.h"
#include "app_config.h"

void app_dac8571_init(void);
HAL_StatusTypeDef app_dac8571_write_raw(uint16_t code);
HAL_StatusTypeDef app_dac8571_write_raw_to_addr(uint8_t addr7, uint16_t code);
HAL_StatusTypeDef app_dac8571_write_code(uint16_t code);
HAL_StatusTypeDef app_dac8571_write_voltage_mV(uint16_t mv);
HAL_StatusTypeDef app_dac8571_set_voltage_uv(uint32_t uv);
HAL_StatusTypeDef app_dac8571_set_initial_near_lock(void);
uint16_t app_dac8571_voltage_to_code(uint32_t uv);
uint16_t app_dac8571_uv_to_code(uint32_t uv);
uint16_t app_dac8571_get_last_code(void);

#endif /* APP_DAC8571_H */
