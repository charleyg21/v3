/**
 * @file
 *
 * Declarations for syringe control commands.
 */

#pragma once

#include <cstdint>
#include <type_traits>

#include "cmsis_os.h"

#include "main.h"

#include "commands/base.hpp"
#include "defines/status.hpp"
#include "helpers/cmsis_os.hpp"

#define FOR_ALL_SYRINGE_COMMANDS(FUNC)                                                 \
    FUNC(REHOME)                                                                       \
    FUNC(PUSH)                                                                         \
    FUNC(PULL)                                                                         \
    FUNC(FILL)

namespace commands {
namespace syringe {
    enum Command {
#define DEFINE_ENUM(COMMAND) COMMAND,
        FOR_ALL_SYRINGE_COMMANDS(DEFINE_ENUM)
#undef DEFINE_ENUM
    };

    Status rehome(void);
    Status push(uint32_t steps);
    Status pull(uint32_t steps);
    Status fill(void);
};

template <> struct Signature<syringe::REHOME> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<syringe::PUSH> {
    using Parameters = std::tuple<uint32_t>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<syringe::PULL> {
    using Parameters = std::tuple<uint32_t>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<syringe::FILL> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status>;
};

#define DECLARE_CALLBACKS(COMMAND)                                                     \
    template <> void set_thread_flag<syringe::COMMAND>(void);                          \
    osStatus_t queue_put(const QueueItem<syringe::COMMAND>& item, uint32_t timeout);
FOR_ALL_SYRINGE_COMMANDS(DECLARE_CALLBACKS)
#undef DECLARE_CALLBACKS

} // namespace commands
