#ifndef APP_TMP117_H
#define APP_TMP117_H

#include "main.h"

#define TMP117_ADDR_7BIT 0x48U
#define TMP117_ADDR_HAL  ((uint16_t)(TMP117_ADDR_7BIT << 1))

HAL_StatusTypeDef tmp117_init(void);
HAL_StatusTypeDef tmp117_read_celsius(float *temp_c);
uint8_t tmp117_is_present(void);

#endif /* APP_TMP117_H */
