/**
 * @file
 *
 * Models and declarations for accessing ADC buffer and values.
 */

#pragma once

#include <cstdint>

#include "cmsis_os.h"

#include "commands/base.hpp"
#include "drivers/thermocouple.hpp"

#include "helpers/cmsis_os.hpp"

#define FOR_ALL_THERMOCOUPLE_COMMANDS(FUNC) FUNC(READ)

namespace commands {
namespace thermocouple {

    enum Command {
#define DEFINE_ENUM(COMMAND) COMMAND,
        FOR_ALL_THERMOCOUPLE_COMMANDS(DEFINE_ENUM)
#undef DEFINE_ENUM
    };

    Status read(int16_t& heater, int16_t& liquid, bool& overheated);
} // namespace adc

template <> struct Signature<thermocouple::READ> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status, int16_t, int16_t>;
};

template <> void set_thread_flag<thermocouple::READ>(void);

#define DECLARE_CALLBACKS(COMMAND)                                                     \
    template <> void set_thread_flag<thermocouple::COMMAND>(void);                     \
    osStatus_t queue_put(                                                              \
        const QueueItem<thermocouple::COMMAND>& item, uint32_t timeout);
FOR_ALL_THERMOCOUPLE_COMMANDS(DECLARE_CALLBACKS)
#undef DECLARE_CALLBACKS

} // namespace commands
