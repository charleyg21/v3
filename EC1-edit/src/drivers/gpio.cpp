/**
 * @file
 *
 * Driver functions for GPIO pins.
 */

#include "defines/gpio.hpp"
#include "drivers/gpio.hpp"

namespace drivers::gpio::pins {

#define DECLARE_PIN_OBJECT(NAME) types::NAME NAME;
FOR_ALL_GPIOS(DECLARE_PIN_OBJECT)
#undef DECLARE_PIN_OBJECT

} // namespace drivers::gpio::pins
