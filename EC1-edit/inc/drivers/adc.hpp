/**
 * @file
 *
 * ADC driver functions.
 */

#pragma once

#include <cstdint>

#include "main.h"

namespace drivers::adc {

/**
 * Enable battery voltage channel.
 *
 * @note VBATEN is the name given in the datasheet of the battery voltage ADC channel.
 * @note See section 14.10 of STM reference manual RM0454.
 */
inline void set_vbaten(ADC_HandleTypeDef* hadc)
{
    ADC_Common_TypeDef* adc_common = __LL_ADC_COMMON_INSTANCE(hadc->Instance);
    uint32_t old_path_settings = LL_ADC_GetCommonPathInternalCh(adc_common);
    uint32_t new_path_settings = LL_ADC_PATH_INTERNAL_VBAT | old_path_settings;
    LL_ADC_SetCommonPathInternalCh(adc_common, new_path_settings);
}

/**
 * Disable battery voltage channel.
 *
 * @note VBATEN is the name given in the datasheet of the battery voltage ADC channel.
 * @note See section 14.10 of STM reference manual RM0454.
 */
inline void clear_vbaten(ADC_HandleTypeDef* hadc)
{
    ADC_Common_TypeDef* adc_common = __LL_ADC_COMMON_INSTANCE(hadc->Instance);
    uint32_t old_path_settings = LL_ADC_GetCommonPathInternalCh(adc_common);
    uint32_t new_path_settings = ~LL_ADC_PATH_INTERNAL_VBAT & old_path_settings;
    LL_ADC_SetCommonPathInternalCh(adc_common, new_path_settings);
}

/**
 * Read VBAT
 */
float read_vbat();

} // namespace drivers::adc
