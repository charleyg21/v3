/**
 * @file
 *
 * Declarations for valve control commands.
 */

#pragma once

#include <cstdint>

#include "cmsis_os.h"

#include "main.h"

#include "commands/base.hpp"
#include "defines/status.hpp"
#include "helpers/cmsis_os.hpp"

#define FOR_ALL_VALVE_COMMANDS(FUNC)                                                   \
    FUNC(REHOME)                                                                       \
    FUNC(MOVE_TO)

namespace commands {
namespace valve {
    enum class Position : uint8_t {
        HOME,
        WATER_IN,
        WATER_OUT,
        SUPERCHARGER,
        SPORES,
    };

    enum Command {
#define DEFINE_ENUM(COMMAND) COMMAND,
        FOR_ALL_VALVE_COMMANDS(DEFINE_ENUM)
#undef DEFINE_ENUM
    };

    Status rehome(void);
    Status move_to(Position position);
} // namespace heater

template <> struct Signature<valve::REHOME> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<valve::MOVE_TO> {
    using Parameters = std::tuple<valve::Position>;
    using ReturnValue = std::tuple<Status>;
};

#define DECLARE_CALLBACKS(COMMAND)                                                     \
    template <> void set_thread_flag<valve::COMMAND>(void);                            \
    osStatus_t queue_put(const QueueItem<valve::COMMAND>& item, uint32_t timeout);
FOR_ALL_VALVE_COMMANDS(DECLARE_CALLBACKS)
#undef DECLARE_CALLBACKS

} // namespace commands
