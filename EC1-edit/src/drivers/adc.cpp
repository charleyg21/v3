#include "drivers/adc.hpp"

#define VREFINT_CAL_VALUE (*VREFINT_CAL_ADDR)

const float ADC_MAX_VALUE = 4095.0f;
const float VBAT_SCALE = 3.0f;

extern ADC_HandleTypeDef hadc1;

namespace drivers::adc {

/**
 * Read VDDA
 */
float read_vdda() {
    ADC_ChannelConfTypeDef config = {};
    config.Channel = ADC_CHANNEL_VREFINT;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) {
        return -1.0f;
    }

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        return -1.0f;
    }

    uint32_t vrefint_data = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return (float) VREFINT_CAL_VREF / 1000.0f * (float) VREFINT_CAL_VALUE / (float) vrefint_data;
}

/**
 * Read VBAT
 */
float read_vbat() {
    float vdda = read_vdda();

    if (vdda == -1.0f) return -1.0f;

    set_vbaten(&hadc1);

    ADC_ChannelConfTypeDef config = {};
    config.Channel = ADC_CHANNEL_VBAT;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) {
        clear_vbaten(&hadc1);
        return -1.0f;
    }

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {
        HAL_ADC_Stop(&hadc1);
        clear_vbaten(&hadc1);
        return -1.0f;
    }

    uint32_t raw_vbat = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    clear_vbaten(&hadc1);

    return ((float) raw_vbat * vdda / ADC_MAX_VALUE) * VBAT_SCALE;
}

}