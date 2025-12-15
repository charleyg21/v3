/**
 * @file
 *
 * Declarations for cycler control commands.
 */

#pragma once

#include <cstdint>

#include "cmsis_os.h"

#include "main.h"

#include "commands/base.hpp"
#include "defines/status.hpp"
#include "helpers/cmsis_os.hpp"

#define FOR_ALL_CYCLER_COMMANDS(FUNC)                                                  \
    FUNC(REHOME)                                                                       \
    FUNC(PRIME)                                                                        \
    FUNC(CYCLE)                                                                        \
    FUNC(RINSE)                                                                        \
    FUNC(MANUAL_RINSE)                                                                 \
    FUNC(ROT_SYRINGE_TEST)                                                             \
    FUNC(ERROR_STATE)

namespace commands {
namespace cycler {
    enum Command {
#define DEFINE_ENUM(COMMAND) COMMAND,
        FOR_ALL_CYCLER_COMMANDS(DEFINE_ENUM)
#undef DEFINE_ENUM
    };

    Status rehome(void);
    Status prime(void);
    Status cycle(void);
    Status rinse(void);
    Status manual_rinse(void);
    Status rot_syringe_test(void);
    Status error_state(void);
} // namespace cycler

template <> struct Signature<cycler::REHOME> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<cycler::PRIME> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<cycler::CYCLE> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<cycler::RINSE> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<cycler::MANUAL_RINSE> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<cycler::ROT_SYRINGE_TEST> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<cycler::ERROR_STATE> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

#define DECLARE_CALLBACKS(COMMAND)                                                     \
    template <> void set_thread_flag<cycler::COMMAND>(void);                           \
    osStatus_t queue_put(const QueueItem<cycler::COMMAND>& item, uint32_t timeout);
FOR_ALL_CYCLER_COMMANDS(DECLARE_CALLBACKS)
#undef DECLARE_CALLBACKS

} // namespace commands
