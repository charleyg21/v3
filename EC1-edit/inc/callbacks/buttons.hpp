/**
 * @file
 *
 * Declarations for USER button press callbacks.
 */

#pragma once

#include "main.h"

#include "defines/gpio.hpp"

namespace callbacks {

#define DEFINE_BUTTON_CALLBACKS(BUTTON)                                                \
    namespace BUTTON {                                                                 \
        void pressed(void);                                                            \
        void released(void);                                                           \
    }
FOR_ALL_BUTTONS(DEFINE_BUTTON_CALLBACKS)
#undef DEFINE_BUTTON_CALLBACKS

} // namespace callbacks
