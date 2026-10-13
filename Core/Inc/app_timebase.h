#ifndef APP_TIMEBASE_H
#define APP_TIMEBASE_H

#include "main.h"

HAL_StatusTypeDef app_timebase_init(void);
uint8_t app_timebase_take_20ms_flag(void);
uint8_t app_timebase_take_1s_flag(void);
/* Free-running TIM3 interrupt count.  Comparing its growth against
 * HAL_GetTick() tells whether the two timebases agree (should be 100 TIM3
 * interrupts per 1000 ticks). */
uint32_t app_timebase_get_tick_count(void);

#endif /* APP_TIMEBASE_H */
