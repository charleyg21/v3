/*
 * Copyright © 2021 Jonathan Starr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/**
 * @file
 *
 * @author Jonathan Starr <jonstarr@utexas.edu>
 * @date April 2nd, 2021
 *
 * @brief Button filter helper class definition.
 */

 /**
  * Added button holding
  * 
  * @author Hunter Gensler <hunter.gensler@gmail.com>
  * @date May 5th, 2025
  */

#pragma once

#include "main.h"

#include "cmsis_os.h"

#include "stream_buffer.h"
#include "timers.h"

#include "helpers/cmsis_os.hpp"

namespace helpers {

/**
 * @brief Button filter class declaration.
 *
 * The button filter is simply a thread, timer, and state machine that
 * debounces buttons. It does this by listening to listening to events
 * registered through signal_change_event(). If a change occurs with no
 * subsequent change within debounce_ticks, the pin state is read and the
 * appropriate _on_press() or _on_release() callback is called.
 */
class ButtonFilter {
public:
    /**
     * @brief Instantiate the button filter helper object.
     * @param priority       CMSIS-OS priority to use for debounce thread.
     * @param debounce_ticks Number of delay ticks for debouncing.
     */
    explicit ButtonFilter(
        osPriority_t priority = osPriorityNormal, uint32_t debounce_ticks = 50)
        : _debounce_ticks(debounce_ticks)
        , _debounce_timer(_debounce_timer_callback, this)
    {
    }

    /**
     * @brief Initialize the button filter helper object.
     *
     * This initializes the encapsulated CMSIS-OS objects.
     */
    void init(void) { _debounce_timer.init(osTimerOnce); }

    /**
     * @brief Register a rising / falling event with the filter.
     */
    void signal_change_event_from_isr(void)
    {
        auto* const handle = static_cast<TimerHandle_t>(_debounce_timer.get_handle());

        if (xTimerChangePeriodFromISR(handle, _debounce_ticks, 0) != pdPASS) {
            return;
        }

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xTimerStartFromISR(handle, &xHigherPriorityTaskWoken) != pdPASS) {
            return;
        }

        switch (_state) {
        case PRESSED:
            _state = RELEASING;
            break;
        case RELEASED:
            _state = PRESSING;
            break;
        default:
            break;
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

private:
    /**
     * @brief Callback to call when the button has been pressed.
     */
    virtual void _on_press(void) = 0;

    /**
     * @brief Callback to call when the button has been released.
     */
    virtual void _on_release(void) = 0;

    /**
     * @brief Callback to call when the button is being held.
     */
    virtual void _on_hold(void) = 0;

    /**
     * @brief Definitions for button holding logic 
     * @def _hold_ticks: the number of timer ticks after pressing the button without
     *      releasing before the button is considered "held"
     * @def _hold_timer_tick: the length of a timer tick (in ms)
     * @def _held: the number of ticks elapsed since pressing the button without releasing
     * @def _hold_timer: the timer for detecting when this button is held
     * @note "this" is passed into the pvTimerGetTimerID parameter of xTimerCreate
     */
    int _hold_ticks = 4;
    int _hold_timer_tick = 125;
    int _held = 0;
    TimerHandle_t _hold_timer = xTimerCreate("hold_timer", _hold_timer_tick, pdTRUE, (void*) this, _holding_timer_callback);

    /**
     * @brief Helper method to determine if the button is in a "pressed" state.
     */
    virtual bool _button_pressed(void) = 0;

    enum ThreadFlag { BUTTON_CHANGED, UNKNOWN };
    static constexpr auto FLAGS = { ThreadFlag::BUTTON_CHANGED };

    // Number of ticks before the debounce timeout event.
    const uint32_t _debounce_ticks;

    // Encapsulate a static debounce timer object.
    cmsis_os::StaticTimer _debounce_timer;

    /// @brief Enumeration of the button state.
    enum State {
        PRESSED,
        PRESSING,
        RELEASED,
        RELEASING,
    } _state
        = RELEASED;

    /// @brief Callback called by helper thread on button change.
    void _on_change(void) { }

    /// @brief Helper method to determine if the button is in the "released" state.
    bool _button_released(void) { return not _button_pressed(); }

    /// @brief Callback called by timer when debounce ticks has occurred.
    static void _debounce_timer_callback(void* parameters)
    {
        auto button_filter = static_cast<ButtonFilter*>(parameters);
        button_filter->_on_debounce_timeout();
    }

    /// @brief Callback called by timer when debounce ticks has occurred.
    void _on_debounce_timeout(void)
    {
        const bool pressed = _button_pressed();
        if (pressed and _state == PRESSING) {
            _on_press();
            if (_hold_timer != NULL) xTimerStart(_hold_timer, 0);
        }

        const bool released = not pressed;
        if (released and _state == RELEASING) {
            _on_release();
            if (_hold_timer != NULL) xTimerStop(_hold_timer, 0);
            _held = 0;
        }

        _state = pressed ? PRESSED : RELEASED;
    }

    /// @brief Callback called by holding timer when it ends
    static void _holding_timer_callback(TimerHandle_t timer) {
        ButtonFilter* instance = static_cast<ButtonFilter*>(pvTimerGetTimerID(timer));
        if (instance->_held >= instance->_hold_ticks) {
            instance->_on_hold();
        }

        instance->_held++;
    }
};

} // namespace helpers
