/**
 * @file
 *
 * Weak-linkage definitions for button press callbacks.
 */

#include "callbacks/buttons.hpp"

namespace callbacks {

#define DECLARE_BUTTON_CALLBACKS(BUTTON)                                               \
    namespace BUTTON {                                                                 \
        __weak void pressed(void) { }                                                  \
        __weak void released(void) { }                                                 \
    }
FOR_ALL_BUTTONS(DECLARE_BUTTON_CALLBACKS)
#undef DECLARE_BUTTON_CALLBACKS

} // callbacks
