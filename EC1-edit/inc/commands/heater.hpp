/**
 * @file
 *
 * Declarations for heater control commands.
 */

#pragma once

#include <cstdint>
#include <type_traits>

#include "cmsis_os.h"

#include "main.h"

#include "commands/base.hpp"
#include "defines/status.hpp"
#include "helpers/cmsis_os.hpp"

#define FOR_ALL_HEATER_COMMANDS(FUNC)                                                  \
    FUNC(STATUS)                                                                       \
    FUNC(TURN_OFF)                                                                     \
    FUNC(TURN_ON)

namespace commands {
namespace heater {
    enum Command {
#define DEFINE_ENUM(COMMAND) COMMAND,
        FOR_ALL_HEATER_COMMANDS(DEFINE_ENUM)
#undef DEFINE_ENUM
    };

    Status turn_on(int16_t target);
    Status turn_off(void);
    Status status(bool& heater_state);
} // namespace heater

template <> struct Signature<heater::TURN_ON> {
    using Parameters = std::tuple<int16_t>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<heater::TURN_OFF> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<heater::STATUS> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<bool>;
};

#define DECLARE_CALLBACKS(COMMAND)                                                     \
    template <> void set_thread_flag<heater::COMMAND>(void);                           \
    osStatus_t queue_put(const QueueItem<heater::COMMAND>& item, uint32_t timeout);
FOR_ALL_HEATER_COMMANDS(DECLARE_CALLBACKS)
#undef DECLARE_CALLBACKS

} // namespace commands
