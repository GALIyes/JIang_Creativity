#ifndef APP_VCT_ADC_H
#define APP_VCT_ADC_H

#include "main.h"

#define VCT_ADC_VDDA_VOLTS 3.3600f

HAL_StatusTypeDef vct_adc_init(void);
HAL_StatusTypeDef vct_adc_read_raw_once(uint16_t *raw);
HAL_StatusTypeDef vct_adc_read_average(uint16_t samples, uint16_t *raw_avg, float *voltage);
HAL_StatusTypeDef vct_adc_read_stats(uint16_t samples, uint16_t *raw_min, uint16_t *raw_max,
                                     uint16_t *raw_avg, float *voltage);

#endif /* APP_VCT_ADC_H */
