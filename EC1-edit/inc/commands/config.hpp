/**
 * @file
 *
 * Declarations for config control commands.
 */

#pragma once

#include <cstdint>
#include <tuple>

#include "cmsis_os.h"

#include "main.h"

#include "commands/base.hpp"
#include "defines/status.hpp"
#include "defines/warnings.hpp"
#include "helpers/cmsis_os.hpp"

#define FOR_ALL_CONFIG_COMMANDS(FUNC)                                                  \
    FUNC(SET_TIME)                                                                     \
    FUNC(GET_TIME)                                                                     \
    FUNC(SET_TIMER)                                                                    \
    FUNC(GET_TIMER)                                                                    \
    FUNC(SET_NUM_RINSES)                                                               \
    FUNC(GET_NUM_RINSES)                                                               \
    FUNC(SET_NUM_CYCLES)                                                               \
    FUNC(GET_NUM_CYCLES)                                                               \
    FUNC(SET_DISPENSE_TIME)                                                            \
    FUNC(GET_DISPENSE_TIME)                                                            \
    FUNC(SET_CONFIGURED)                                                               \
    FUNC(GET_CONFIGURED)                                                               \
    FUNC(UPDATE_VBAT)                                                                  \
    FUNC(GET_VBAT)                                                                     \
    FUNC(SET_HOMED)                                                                    \
    FUNC(GET_HOMED)                                                                    \
    FUNC(SET_CYCLING)                                                                  \
    FUNC(GET_CYCLING)                                                                  \
    FUNC(SET_CYCLES_RUN)                                                               \
    FUNC(GET_CYCLES_RUN)                                                               \
    FUNC(CYCLE_MODE)                                                                   \
    FUNC(CYCLE_COUNT)                                                                  \
    FUNC(SET_WARNING)                                                                  \
    FUNC(GET_WARNINGS)                                                                 \
    FUNC(GET_ACTIVE_WARNING)                                                           \
    FUNC(GET_WARNING_LEVEL)                                                            \
    FUNC(RESET_WARNINGS)                                                               \
    FUNC(RESET_WARNING)

namespace commands {
namespace config {
    enum Command {
#define DEFINE_ENUM(COMMAND) COMMAND,
        FOR_ALL_CONFIG_COMMANDS(DEFINE_ENUM)
#undef DEFINE_ENUM
    };

    enum CycleMode {
        DEFAULT,
        WASTEWATER,
        ANAEROBIC
    };

    Status set_time(uint8_t hours, uint8_t minutes);
    std::tuple<Status, uint8_t, uint8_t> get_time(void);

    Status set_timer(uint8_t hours, uint8_t minutes);
    std::tuple<Status, uint8_t, uint8_t> get_timer(void);

    Status set_num_rinses(uint8_t num_rinses);
    std::tuple<Status, uint8_t> get_num_rinses(void);

    Status set_num_cycles(uint32_t num_cycles);
    std::tuple<Status, uint32_t> get_num_cycles(void);

    Status set_dispense_time(uint8_t index, uint16_t dispense_time);
    std::tuple<Status, uint16_t> get_dispense_time(uint8_t index);

    Status set_configured(bool configured);
    std::tuple<Status, bool> get_configured(void);

    Status set_cycles_run(uint8_t cycles_run);
    std::tuple<Status, uint8_t> get_cycles_run(void);

    Status set_cycle_mode(uint8_t mode);
    std::tuple<Status, uint8_t> get_cycle_mode(void);

    Status set_cycle_count(uint8_t count);
    std::tuple<Status, uint8_t> get_cycle_count(void);
    
    void update_vbat(void);
    float get_vbat(void);

    void set_homed(bool homed);
    bool get_homed(void);

    void set_cycling(bool cycling);
    bool get_cycling(void);
} // namespace config

template <> struct Signature<config::SET_TIME> {
    using Parameters = std::tuple<uint8_t, uint8_t>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<config::GET_TIME> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status, uint8_t, uint8_t>;
};

template <> struct Signature<config::SET_TIMER> {
    using Parameters = std::tuple<uint8_t, uint8_t>;
    using ReturnValue = std::tuple<Status>;
};

template <> struct Signature<config::GET_TIMER> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<Status, uint8_t, uint8_t>;
};

template <> struct Signature<config::SET_NUM_RINSES> {
    using Parameters = std::tuple<uint8_t>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_NUM_RINSES> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<uint8_t>;
};

template <> struct Signature<config::SET_NUM_CYCLES> {
    using Parameters = std::tuple<uint8_t>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_NUM_CYCLES> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<uint8_t>;
};

template <> struct Signature<config::SET_DISPENSE_TIME> {
    using Parameters = std::tuple<uint8_t, uint16_t>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_DISPENSE_TIME> {
    using Parameters = std::tuple<uint8_t>;
    using ReturnValue = std::tuple<uint16_t>;
};

template <> struct Signature<config::SET_CONFIGURED> {
    using Parameters = std::tuple<bool>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_CONFIGURED> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<bool>;
};

template <> struct Signature<config::UPDATE_VBAT> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_VBAT> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<float>;
};

template <> struct Signature<config::SET_HOMED> {
    using Parameters = std::tuple<bool>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_HOMED> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<bool>;
};

template <> struct Signature<config::SET_CYCLING> {
    using Parameters = std::tuple<bool>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_CYCLING> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<bool>;
};

template <> struct Signature<config::SET_CYCLES_RUN> {
    using Parameters = std::tuple<uint8_t>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_CYCLES_RUN> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<uint8_t>;
};

template <> struct Signature<config::CYCLE_MODE> {
    using Parameters = std::tuple<uint8_t>;
    using ReturnValue = std::tuple<uint8_t>;
};

template <> struct Signature<config::CYCLE_COUNT> {
    using Parameters = std::tuple<uint8_t>;
    using ReturnValue = std::tuple<uint8_t>;
};

template <> struct Signature<config::SET_WARNING> {
    using Parameters = std::tuple<warnings::WarningLevel, warnings::Warning>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::GET_WARNINGS> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<uint8_t>;
};

template <> struct Signature<config::GET_ACTIVE_WARNING> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<warnings::Warning>;
};

template <> struct Signature<config::GET_WARNING_LEVEL> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<warnings::WarningLevel>;
};

template <> struct Signature<config::RESET_WARNINGS> {
    using Parameters = std::tuple<>;
    using ReturnValue = std::tuple<>;
};

template <> struct Signature<config::RESET_WARNING> {
    using Parameters = std::tuple<warnings::Warning>;
    using ReturnValue = std::tuple<>;
};

#define DECLARE_CALLBACKS(COMMAND)                                                     \
    template <> void set_thread_flag<config::COMMAND>(void);                           \
    osStatus_t queue_put(const QueueItem<config::COMMAND>& item, uint32_t timeout);
FOR_ALL_CONFIG_COMMANDS(DECLARE_CALLBACKS)
#undef DECLARE_CALLBACKS

} // namespace commands