#ifndef APP_PPS_H
#define APP_PPS_H

#include "main.h"

typedef struct {
    uint32_t count;
    uint32_t last_tick_ms;
    uint32_t interval_ms;
    uint32_t invalid_interval_count;
    uint8_t valid_interval_streak;
} app_pps_snapshot_t;

void app_pps_init(void);
void app_pps_get_snapshot(app_pps_snapshot_t *snapshot);

#endif /* APP_PPS_H */

