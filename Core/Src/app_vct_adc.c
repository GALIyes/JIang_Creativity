#include "app_vct_adc.h"

#include "adc.h"

#define VCT_ADC_POLL_TIMEOUT_MS 5U
#define VCT_ADC_DISCARD_SAMPLES 4U

HAL_StatusTypeDef vct_adc_init(void)
{
    return HAL_OK;
}

HAL_StatusTypeDef vct_adc_read_raw_once(uint16_t *raw)
{
    HAL_StatusTypeDef st;
    uint32_t value;

    if (raw == (uint16_t *)0) {
        return HAL_ERROR;
    }

    st = HAL_ADC_Start(&hadc1);
    if (st != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return st;
    }

    st = HAL_ADC_PollForConversion(&hadc1, VCT_ADC_POLL_TIMEOUT_MS);
    if (st != HAL_OK) {
        (void)HAL_ADC_Stop(&hadc1);
        return st;
    }

    value = HAL_ADC_GetValue(&hadc1);
    (void)HAL_ADC_Stop(&hadc1);

    if (value > 4095UL) {
        value = 4095UL;
    }
    *raw = (uint16_t)value;
    return HAL_OK;
}

HAL_StatusTypeDef vct_adc_read_average(uint16_t samples, uint16_t *raw_avg, float *voltage)
{
    uint16_t raw_min;
    uint16_t raw_max;

    return vct_adc_read_stats(samples, &raw_min, &raw_max, raw_avg, voltage);
}

HAL_StatusTypeDef vct_adc_read_stats(uint16_t samples, uint16_t *raw_min, uint16_t *raw_max,
                                     uint16_t *raw_avg, float *voltage)
{
    uint16_t i;
    uint16_t raw;
    uint16_t kept = 0U;
    uint32_t sum = 0UL;
    HAL_StatusTypeDef st;

    if ((samples == 0U) ||
        (raw_min == (uint16_t *)0) ||
        (raw_max == (uint16_t *)0) ||
        (raw_avg == (uint16_t *)0) ||
        (voltage == (float *)0)) {
        return HAL_ERROR;
    }

    for (i = 0U; i < (uint16_t)(samples + VCT_ADC_DISCARD_SAMPLES); i++) {
        st = vct_adc_read_raw_once(&raw);
        if (st != HAL_OK) {
            return st;
        }
        if (i < VCT_ADC_DISCARD_SAMPLES) {
            continue;
        }

        if (kept == 0U) {
            *raw_min = raw;
            *raw_max = raw;
        } else {
            if (raw < *raw_min) {
                *raw_min = raw;
            }
            if (raw > *raw_max) {
                *raw_max = raw;
            }
        }
        sum += raw;
        kept++;
    }

    if (kept == 0U) {
        return HAL_ERROR;
    }

    *raw_avg = (uint16_t)((sum + ((uint32_t)kept / 2UL)) / (uint32_t)kept);
    *voltage = ((float)(*raw_avg) * VCT_ADC_VDDA_VOLTS) / 4095.0f;
    return HAL_OK;
}
