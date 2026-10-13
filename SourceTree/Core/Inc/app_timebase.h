#ifndef APP_TIMEBASE_H
#define APP_TIMEBASE_H

#include "main.h"

HAL_StatusTypeDef app_timebase_init(void);
uint8_t app_timebase_take_20ms_flag(void);
uint8_t app_timebase_take_1s_flag(void);

#endif /* APP_TIMEBASE_H */
