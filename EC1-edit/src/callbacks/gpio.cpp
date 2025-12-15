/**
 * @file
 *
 * Weak-linkage definitions for GPIO callbacks.
 */

#include "main.h"

#include "defines/gpio.hpp"
#include "drivers/gpio.hpp"

namespace drivers::gpio::pins::types {

#define DEFINE_RISING_FALLING_CALLBACKS(NAME)                                          \
    __weak void NAME::on_rising(void) { }                                              \
    __weak void NAME::on_falling(void) { }
FOR_ALL_GPIO_INPUTS(DEFINE_RISING_FALLING_CALLBACKS)
#undef DEFINE_RISING_FALLING_CALLBACKS

} // namespace drivers::gpio::pins::types

/**
 * HAL GPIO EXTI rising callback implementation.
 *
 * Simply delegates to pin-specific callback.
 */
extern "C" void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    using namespace drivers::gpio::pins::types;
#define CALL_CALLBACK(NAME)                                                            \
    if (GPIO_Pin == NAME::PIN) {                                                       \
        NAME::on_rising();                                                             \
    }
    FOR_ALL_GPIO_INPUTS(CALL_CALLBACK)
#undef CALL_CALLBACK
}

/**
 * HAL GPIO EXTI falling callback implementation.
 *
 * Simply delegates to pin-specific callback.
 */
extern "C" void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    using namespace drivers::gpio::pins::types;
#define CALL_CALLBACK(NAME)                                                            \
    if (GPIO_Pin == NAME::PIN) {                                                       \
        NAME::on_falling();                                                            \
    }
    FOR_ALL_GPIO_INPUTS(CALL_CALLBACK)
#undef CALL_CALLBACK
}
