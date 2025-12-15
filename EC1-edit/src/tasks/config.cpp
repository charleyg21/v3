/**
 * @file
 *
 * Task for handling configuration.
 */

#include <cstdint>
#include <tuple>

#include "cmsis_os.h"

#include "main.h"

#include "timers.h"

#include "defines/gpio.hpp"

#include "commands/config.hpp"
#include "commands/cycler.hpp"
#include "drivers/rtc.hpp"
#include "drivers/gpio.hpp"
#include "drivers/adc.hpp"
#include "helpers/cmsis_os.hpp"
#include "commands/warnings.hpp"

/////////////////////////
// Thread event flags. //
/////////////////////////

using Status = defines::Status;

namespace {

#define FOR_ALL_THREAD_FLAGS(FUNC)                                                     \
    FOR_ALL_CONFIG_COMMANDS(FUNC)                                                      \
    FUNC(PRESET)

enum class ThreadFlag : uint32_t {
#define DECLARE_ENUM(FLAG) FLAG,
    FOR_ALL_THREAD_FLAGS(DECLARE_ENUM) UNKNOWN
#undef DECLARE_ENUM
};

constexpr auto FLAGS = {
#define DECLARE_ENUM(FLAG) ThreadFlag::FLAG,
    FOR_ALL_THREAD_FLAGS(DECLARE_ENUM)
#undef DECLARE_ENUM
};

class Thread : public cmsis_os::Thread, public cmsis_os::Flaggable<Thread, ThreadFlag> {
public:
    Thread(void)
        : cmsis_os::Thread()
        , cmsis_os::Flaggable<Thread, ThreadFlag>(FLAGS)
    {
    }

    void target(RTC_HandleTypeDef* hrtc);
} THREAD;

} // anonyous namespace

//////////////////
// Config Model //
//////////////////

struct Config {
    bool configured;
    uint8_t num_rinses;
    uint8_t mode_settings;
    uint32_t num_cycles;
    uint16_t dispense_time[5];
    uint8_t warnings;
    uint8_t cycles_run;
};

static_assert(
    sizeof(Config) == (5 * sizeof(uint32_t)), "Invalid size of configuration model.");

union config_union {
    Config as_struct;
    uint32_t as_words[5];
};

///////////////////////
// Controller Queues //
///////////////////////

namespace commands::config {

constexpr std::size_t REQUEST_QUEUE_SIZE = 1;

#define DECLARE_QUEUE(COMMAND)                                                         \
    static Queue<COMMAND, REQUEST_QUEUE_SIZE> COMMAND##_QUEUE;
FOR_ALL_CONFIG_COMMANDS(DECLARE_QUEUE)
#undef DECLARE_QUEUE

} // namespace commands::config

namespace commands {

#define DEFINE_QUEUE_PUT(COMMAND)                                                      \
    template <>                                                                        \
    osStatus_t queue_put<config::COMMAND>(                                             \
        const QueueItem<config::COMMAND>& item, uint32_t timeout)                      \
    {                                                                                  \
        return config::COMMAND##_QUEUE.put(item, timeout);                             \
    }
FOR_ALL_CONFIG_COMMANDS(DEFINE_QUEUE_PUT)
#undef DEFINE_QUEUE_PUT

#define DEFINE_SET_THREAD_FLAG(COMMAND)                                                \
    template <> void set_thread_flag<config::COMMAND>(void)                            \
    {                                                                                  \
        THREAD.set_flag(ThreadFlag::COMMAND);                                          \
    }
FOR_ALL_CONFIG_COMMANDS(DEFINE_SET_THREAD_FLAG)
#undef DEFINE_SET_THREAD_FLAG

} // namespace commands

namespace commands::config {

bool homed = false;
bool cycling = false;
TimerHandle_t timer = nullptr;
int timer_repeats = 0;
int test = -1;

Status set_time(uint8_t hours, uint8_t minutes)
{
    const auto params = std::make_tuple(hours, minutes);
    ReturnValue<SET_TIME> return_value {};
    const auto send_status = send_and_block<SET_TIME>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

std::tuple<Status, uint8_t, uint8_t> get_time(void)
{
    const auto params = std::make_tuple();
    ReturnValue<GET_TIME> return_value {};
    const auto send_status = send_and_block<GET_TIME>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        const auto status = defines::status::map_from(send_status);
        return std::make_tuple(status, uint8_t { 0 }, uint8_t { 0 });
    }

    return return_value;
}

Status set_timer(uint8_t hours, uint8_t minutes)
{
    const auto params = std::make_tuple(hours, minutes);
    ReturnValue<SET_TIMER> return_value {};
    const auto send_status = send_and_block<SET_TIMER>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

std::tuple<Status, uint8_t, uint8_t> get_timer(void)
{
    const auto params = std::make_tuple();
    ReturnValue<GET_TIMER> return_value {};
    const auto send_status = send_and_block<GET_TIMER>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        const auto status = defines::status::map_from(send_status);
        return std::make_tuple(status, uint8_t { 0 }, uint8_t { 0 });
    }

    return return_value;
}

Status set_dispense_time(uint8_t index, uint16_t dispense_time)
{
    const auto params = std::make_tuple(index, dispense_time);
    ReturnValue<SET_DISPENSE_TIME> return_value {};
    const auto send_status = send_and_block<SET_DISPENSE_TIME>(params, return_value);
    return defines::status::map_from(send_status);
}

std::tuple<Status, uint16_t> get_dispense_time(uint8_t index)
{
    const auto params = std::make_tuple(index);
    ReturnValue<GET_DISPENSE_TIME> return_value {};
    const auto send_status = send_and_block<GET_DISPENSE_TIME>(params, return_value);
    const auto status = defines::status::map_from(send_status);
    const auto [dispense_time] = return_value;
    return std::make_tuple(status, dispense_time);
}

Status set_num_rinses(uint8_t num_rinses)
{
    const auto params = std::make_tuple(num_rinses);
    ReturnValue<SET_NUM_RINSES> return_value {};
    const auto send_status = send_and_block<SET_NUM_RINSES>(params, return_value);
    return defines::status::map_from(send_status);
}

std::tuple<Status, uint8_t> get_num_rinses(void)
{
    const auto params = std::make_tuple();
    ReturnValue<GET_NUM_RINSES> return_value {};
    const auto send_status = send_and_block<GET_NUM_RINSES>(params, return_value);
    const auto status = defines::status::map_from(send_status);
    const auto [num_rinses] = return_value;
    return std::make_tuple(status, num_rinses);
}

Status set_num_cycles(uint32_t num_cycles) {
    const auto params = std::make_tuple(num_cycles);
    ReturnValue<SET_NUM_CYCLES> return_value {};
    const auto send_status = send_and_block<SET_NUM_CYCLES>(params, return_value);
    return defines::status::map_from(send_status);
}

std::tuple<Status, uint32_t> get_num_cycles(void)
{
    const auto params = std::make_tuple();
    ReturnValue<GET_NUM_CYCLES> return_value {};
    const auto send_status = send_and_block<GET_NUM_CYCLES>(params, return_value);
    const auto status = defines::status::map_from(send_status);
    const auto [num_cycles] = return_value;
    return std::make_tuple(status, num_cycles);
}

Status set_configured(bool configured)
{
    const auto params = std::make_tuple(configured);
    ReturnValue<SET_CONFIGURED> return_value {};
    const auto send_status = send_and_block<SET_CONFIGURED>(params, return_value);
    return defines::status::map_from(send_status);
}

std::tuple<Status, bool> get_configured(void)
{
    const auto params = std::make_tuple();
    ReturnValue<GET_CONFIGURED> return_value {};
    const auto send_status = send_and_block<GET_CONFIGURED>(params, return_value);
    const auto status = defines::status::map_from(send_status);
    const auto [configured] = return_value;
    return std::make_tuple(status, configured);
}

Status set_cycles_run(uint8_t cycles_run)
{
    const auto params = std::make_tuple(cycles_run);
    ReturnValue<SET_CYCLES_RUN> return_value {};
    const auto send_status = send_and_block<SET_CYCLES_RUN>(params, return_value);
    return defines::status::map_from(send_status);
}

std::tuple<Status, uint8_t> get_cycles_run(void)
{
    const auto params = std::make_tuple();
    ReturnValue<GET_CYCLES_RUN> return_value {};
    const auto send_status = send_and_block<GET_CYCLES_RUN>(params, return_value);
    const auto status = defines::status::map_from(send_status);
    const auto [cycles_run] = return_value;
    return std::make_tuple(status, cycles_run);
}

Status set_cycle_mode(uint8_t mode)
{
    const auto params = std::make_tuple(mode);
    ReturnValue<CYCLE_MODE> return_value {};
    const auto send_status = send_and_block<CYCLE_MODE>(params, return_value);
    return defines::status::map_from(send_status);
}

std::tuple<Status, uint8_t> get_cycle_mode(void)
{
    const auto params = std::make_tuple(255);
    ReturnValue<CYCLE_MODE> return_value {};
    const auto send_status = send_and_block<CYCLE_MODE>(params, return_value);
    const auto status = defines::status::map_from(send_status);
    const auto [mode] = return_value;
    return std::make_tuple(status, mode);
}

Status set_cycle_count(uint8_t count)
{
    const auto params = std::make_tuple(count);
    ReturnValue<CYCLE_COUNT> return_value {};
    const auto send_status = send_and_block<CYCLE_COUNT>(params, return_value);
    return defines::status::map_from(send_status);
}

std::tuple<Status, uint8_t> get_cycle_count(void)
{
    const auto params = std::make_tuple(255);
    ReturnValue<CYCLE_COUNT> return_value {};
    const auto send_status = send_and_block<CYCLE_COUNT>(params, return_value);
    const auto status = defines::status::map_from(send_status);
    const auto [count] = return_value;
    return std::make_tuple(status, count);
}

void update_vbat(void)
{
    const auto params = std::make_tuple();
    ReturnValue<UPDATE_VBAT> return_value {};
    const auto send_status = send_and_block<UPDATE_VBAT>(params, return_value);
}

float get_vbat(void)
{
    const auto params = std::make_tuple();
    ReturnValue<GET_VBAT> return_value {};
    const auto send_status = send_and_block<GET_VBAT>(params, return_value);
    const auto [vbat] = return_value;
    return vbat;
}

void set_homed(bool homed) {
    const auto params = std::make_tuple(homed);
    ReturnValue<SET_HOMED> return_value {};
    const auto send_status = send_and_block<SET_HOMED>(params, return_value);
}

bool get_homed(void) {
    const auto params = std::make_tuple();
    ReturnValue<GET_HOMED> return_value {};
    const auto send_status = send_and_block<GET_HOMED>(params, return_value);
    const auto [homed] = return_value;
    return homed;
}

void set_cycling(bool cycling) {
    const auto params = std::make_tuple(cycling);
    ReturnValue<SET_CYCLING> return_value {};
    const auto send_status = send_and_block<SET_CYCLING>(params, return_value);
}

bool get_cycling(void) {
    const auto params = std::make_tuple();
    ReturnValue<GET_CYCLING> return_value {};
    const auto send_status = send_and_block<GET_CYCLING>(params, return_value);
    const auto [cycling] = return_value;
    return cycling;
}

void _preset_callback(TimerHandle_t timer) {
    using namespace warnings;

    if (commands::config::timer_repeats == 0) return;
    commands::config::timer_repeats--;

    WarningLevel level = get_warning_level();
    uint8_t warnings = get_warnings();
    WarningLevel logged_level = _warning_level_from_warnings(warnings);
    if (level == WarningLevel::Critical 
            || logged_level == WarningLevel::Critical) return;
    
    if (commands::config::get_cycling()) return;

    THREAD.set_flag(ThreadFlag::PRESET); 
}

} // namespace commands::config

namespace warnings {
    using namespace commands;
    using namespace commands::config;

    WarningLevel warning_level = WarningLevel::None;
    Warning active_warning = Warning::Overflow;

    #define LEAK_TIME 5
    uint8_t leaked = 0;

    #define LOW_VBAT_TIME 60
    const float LOW_VBAT = 2.2f;
    uint8_t low_battery = 0;

    void set_warning(WarningLevel new_warning_level, Warning new_warning) {
        const auto params = std::make_tuple(new_warning_level, new_warning);
        ReturnValue<SET_WARNING> return_value {};
        const auto send_status = send_and_block<SET_WARNING>(params, return_value);
    }

    uint8_t get_warnings(void) {
        const auto params = std::make_tuple();
        ReturnValue<GET_WARNINGS> return_value {};
        const auto send_status = send_and_block<GET_WARNINGS>(params, return_value);
        const auto [warnings] = return_value;
        return warnings;
    }

    Warning get_active_warning(void) {
        const auto params = std::make_tuple();
        ReturnValue<GET_ACTIVE_WARNING> return_value {};
        const auto send_status = send_and_block<GET_ACTIVE_WARNING>(params, return_value);
        const auto [warning] = return_value;
        return warning;
    }

    WarningLevel get_warning_level(void) {
        const auto params = std::make_tuple();
        ReturnValue<GET_WARNING_LEVEL> return_value {};
        const auto send_status = send_and_block<GET_WARNING_LEVEL>(params, return_value);
        const auto [warning_level] = return_value;
        return warning_level;
    }

    void reset_warnings(void) {
        const auto params = std::make_tuple();
        ReturnValue<RESET_WARNINGS> return_value {};
        const auto send_status = send_and_block<RESET_WARNINGS>(params, return_value);
    }

    void reset_warning(Warning warning) {
        const auto params = std::make_tuple(warning);
        ReturnValue<RESET_WARNING> return_value {};
        const auto send_status = send_and_block<RESET_WARNING>(params, return_value);
    }

    bool _is_warning(uint8_t warnings, Warning warning) {
        return (warnings >> warning) & 1;
    }

    bool _is_any_warning(uint8_t warnings) {
        return (warnings & 0x3F) != 0;
    }

    WarningLevel _warning_level_from_warnings(uint8_t warnings) {
        if (_is_warning(warnings, Overflow) || _is_warning(warnings, Syringe_Homing)) {
            return Critical;
        }
        else if (_is_warning(warnings, Rotary_Homing) || _is_warning(warnings, High_Temp)) {
            return High;
        }
        else if (_is_warning(warnings, Low_Temp) || _is_warning(warnings, Low_Battery)) {
            return Normal;
        }

        return None;
    }

    void _error_timer_callback(TimerHandle_t timer) {
        bool overflow = drivers::gpio::pins::LEAK_DETECT.read() == GPIO_PIN_RESET;

        if (overflow) {
            if (leaked++ >= LEAK_TIME) {
                set_warning(Critical, Overflow);
            }
        }
        else {
            leaked = 0;
        }

        float battery = commands::config::get_vbat();
        bool warn_low_battery = false;

        if (battery < LOW_VBAT) {
            if (low_battery++ >= LOW_VBAT_TIME) {
                set_warning(Normal, Low_Battery);
                low_battery = 0;
                warn_low_battery = true;
            }
        }
        else {
            low_battery = 0;
        }

        WarningLevel current_level = get_warning_level();
        
        switch (current_level) {
            case Critical:
                commands::cycler::error_state();
                ui_warning(Critical);
                break;
            case Normal:
                if (warn_low_battery) ui_warning(Normal);
                break;

        }
    }
} // namespace warnings

namespace drivers::adc {
    float last_vbat = 3.0f;
}

/////////////////////////
// Timer thread target //
/////////////////////////

void Thread::target(RTC_HandleTypeDef* hrtc)
{
    using RealTimeClock = drivers::RealTimeClock;
    RealTimeClock rtc(hrtc);
    constexpr auto num_registers = RealTimeClock::NUM_RTC_BACKUP_REGISTERS;

    // Load the configuration.
    union config_union config = { .as_struct = {} };
    for (auto index = decltype(num_registers) { 0 }; index < num_registers; index++) {
        config.as_words[index] = rtc.read_backup(index);
    }

    const auto write_config = [&rtc](uint32_t (&config)[num_registers]) {
        const auto size = num_registers;
        for (auto index = decltype(size) { 0 }; index < size; index++) {
            rtc.write_backup(index, config[index]);
        }
    };

    #ifdef PROD
        hrtc->AlarmAEventCallback = [](RTC_HandleTypeDef* hrtc) {
            #define ONE_HOUR 3600000

            using RealTimeClock = drivers::RealTimeClock;
            RealTimeClock rtc(hrtc);
            constexpr auto num_registers = RealTimeClock::NUM_RTC_BACKUP_REGISTERS;

            // Load the configuration.
            union config_union config = { .as_struct = {} };
            for (auto index = decltype(num_registers) { 0 }; index < num_registers; index++) {
                config.as_words[index] = rtc.read_backup(index);
            }

            int cycle_count = config.as_struct.mode_settings >> 2;

            if (commands::config::timer != nullptr) {
                xTimerStop(commands::config::timer, 0);
                xTimerDelete(commands::config::timer, 0);
            }

            switch (cycle_count) {
                case 0:
                    commands::config::timer_repeats = 1;
                    break;
                case 1:
                    commands::config::timer_repeats = 2;
                    commands::config::timer = xTimerCreate("preset", ONE_HOUR * 12, 
                        pdTRUE, (void*) 0, commands::config::_preset_callback);
                    xTimerStart(commands::config::timer, 0);
                    break;
                case 2:
                    commands::config::timer_repeats = 3;
                    commands::config::timer = xTimerCreate("preset", ONE_HOUR * 8, 
                        pdTRUE, (void*) 0, commands::config::_preset_callback);
                    xTimerStart(commands::config::timer, 0);
                    break;
                case 3:
                    commands::config::timer_repeats = 4;
                    commands::config::timer = xTimerCreate("preset", ONE_HOUR * 6, 
                        pdTRUE, (void*) 0, commands::config::_preset_callback);
                    xTimerStart(commands::config::timer, 0);
                    break;
                case 4:
                    commands::config::timer_repeats = 6;
                    commands::config::timer = xTimerCreate("preset", ONE_HOUR * 4, 
                        pdTRUE, (void*) 0, commands::config::_preset_callback);
                    xTimerStart(commands::config::timer, 0);
                    break;
                case 5:
                    commands::config::timer_repeats = 8;
                    commands::config::timer = xTimerCreate("preset", ONE_HOUR * 3, 
                        pdTRUE, (void*) 0, commands::config::_preset_callback);
                    xTimerStart(commands::config::timer, 0);
                    break;
                case 6:
                    commands::config::timer_repeats = 12;
                    commands::config::timer = xTimerCreate("preset", ONE_HOUR * 2, 
                        pdTRUE, (void*) 0, commands::config::_preset_callback);
                    xTimerStart(commands::config::timer, 0);
                    break;
            }

            commands::config::_preset_callback(nullptr);
        };
    #endif

    using namespace commands;
    using namespace commands::config;

    const auto handle_set_time = [&rtc](Signature<SET_TIME>::Parameters params) {
        const auto [hours, minutes] = params;
        const auto status = rtc.set_time(hours, minutes);
        // Must do a dummy `set_date()` here to synchronize shadow registers.
        rtc.set_date(0, 0, 0);
        return Signature<SET_TIMER>::ReturnValue { status };
    };

    const auto handle_get_time = [&rtc](Signature<GET_TIME>::Parameters params) {
        const auto [time_status, hours, minutes] = rtc.get_time();
        // Must do a dummy `get_date()` here to synchronize shadow registers.
        rtc.get_date();
        return Signature<GET_TIMER>::ReturnValue { time_status, hours, minutes };
    };

    const auto handle_set_timer = [&](Signature<SET_TIMER>::Parameters params) {
        const auto [hours, minutes] = params;
        const auto status = rtc.set_alarm_a(hours, minutes);
        return Signature<SET_TIMER>::ReturnValue { status };
    };

    const auto handle_get_timer = [&rtc](Signature<GET_TIMER>::Parameters params) {
        const auto [status, hours, minutes] = rtc.get_alarm_a();
        return Signature<GET_TIMER>::ReturnValue { status, hours, minutes };
    };

    const auto handle_set_dispense_time
        = [&config, &write_config](Signature<SET_DISPENSE_TIME>::Parameters params) {
              const auto [index, dispense_time] = params;
              config.as_struct.dispense_time[index] = dispense_time;
              write_config(config.as_words);
              return Signature<SET_DISPENSE_TIME>::ReturnValue {};
          };

    const auto handle_get_dispense_time
        = [&config](Signature<GET_DISPENSE_TIME>::Parameters params) {
              const auto [index] = params;
              const auto dispense_time = config.as_struct.dispense_time[index];
              return Signature<GET_DISPENSE_TIME>::ReturnValue { dispense_time };
          };

    const auto handle_set_num_rinses
        = [&config, &write_config](Signature<SET_NUM_RINSES>::Parameters params) {
              const auto [num_rinses] = params;
              config.as_struct.num_rinses = num_rinses;
              write_config(config.as_words);
              return Signature<SET_NUM_RINSES>::ReturnValue {};
          };

    const auto handle_get_num_rinses
        = [&config](Signature<GET_NUM_RINSES>::Parameters params) {
              const auto num_rinses = config.as_struct.num_rinses;
              return Signature<GET_NUM_RINSES>::ReturnValue { num_rinses };
          };

    const auto handle_set_num_cycles
        = [&config, &write_config](Signature<SET_NUM_CYCLES>::Parameters params) {
              const auto [num_cycles] = params;
              config.as_struct.num_cycles = num_cycles;
              write_config(config.as_words);
              return Signature<SET_NUM_CYCLES>::ReturnValue {};
          };

    const auto handle_get_num_cycles
        = [&config](Signature<GET_NUM_CYCLES>::Parameters params) {
              const auto num_cycles = config.as_struct.num_cycles;
              return Signature<GET_NUM_CYCLES>::ReturnValue { num_cycles };
          };

    const auto handle_set_configured
        = [&config, &write_config](Signature<SET_CONFIGURED>::Parameters params) {
              const auto [configured] = params;
              config.as_struct.configured = configured;
              write_config(config.as_words);
              return Signature<SET_CONFIGURED>::ReturnValue {};
          };

    const auto handle_get_configured
        = [&config](Signature<GET_CONFIGURED>::Parameters params) {
              const auto configured = config.as_struct.configured;
              return Signature<GET_CONFIGURED>::ReturnValue { configured };
          };
    
    const auto handle_update_vbat
            = [](Signature<UPDATE_VBAT>::Parameters params) {
        drivers::adc::last_vbat = drivers::adc::read_vbat();
        return Signature<UPDATE_VBAT>::ReturnValue {};
    };

    const auto handle_get_vbat
            = [](Signature<GET_VBAT>::Parameters params) {
        return Signature<GET_VBAT>::ReturnValue { drivers::adc::last_vbat };
    };

    const auto handle_set_homed
            = [](Signature<SET_HOMED>::Parameters params) {
        const auto [homed] = params;
        commands::config::homed = homed;
        return Signature<SET_HOMED>::ReturnValue {};
    };

    const auto handle_get_homed
            = [](Signature<GET_HOMED>::Parameters params) {
        return Signature<GET_HOMED>::ReturnValue { commands::config::homed };
    };

    const auto handle_set_cycling
            = [](Signature<SET_CYCLING>::Parameters params) {
        const auto [cycling] = params;
        commands::config::cycling = cycling;
        return Signature<SET_CYCLING>::ReturnValue {};
    };

    const auto handle_get_cycling
            = [](Signature<GET_CYCLING>::Parameters params) {
        return Signature<GET_CYCLING>::ReturnValue { commands::config::cycling };
    };

    const auto handle_set_cycles_run
            = [&config, &write_config](Signature<SET_CYCLES_RUN>::Parameters params) {
        const auto [cycles_run] = params;
        config.as_struct.cycles_run = cycles_run;
        write_config(config.as_words);
        return Signature<SET_CYCLES_RUN>::ReturnValue {};
    };

    const auto handle_get_cycles_run
            = [&config](Signature<GET_CYCLES_RUN>::Parameters params) {
        const auto cycles_run = config.as_struct.cycles_run;
        return Signature<GET_CYCLES_RUN>::ReturnValue { cycles_run };
    };

    const auto handle_cycle_mode
            = [&config, &write_config](Signature<CYCLE_MODE>::Parameters params) {
        const auto [new_mode] = params;

        if (new_mode != 255) {
            uint8_t new_mode_settings = config.as_struct.mode_settings;
            new_mode_settings = (new_mode_settings & ~0x3) | (new_mode & 0x3);
            config.as_struct.mode_settings = new_mode_settings;
            write_config(config.as_words);
        }

        const auto mode_settings = config.as_struct.mode_settings;
        uint8_t cycle_mode = mode_settings & 0x3;

        return Signature<CYCLE_MODE>::ReturnValue { cycle_mode };
    };

    const auto handle_cycle_count
            = [&config, &write_config](Signature<CYCLE_COUNT>::Parameters params) {
        const auto [new_count] = params;

        if (new_count != 255) {
            uint8_t new_mode_settings = config.as_struct.mode_settings;
            new_mode_settings = (new_count << 2) | (new_mode_settings & 0x3);
            config.as_struct.mode_settings = new_mode_settings;
            write_config(config.as_words);
        }

        const auto mode_settings = config.as_struct.mode_settings;
        uint8_t cycle_count = mode_settings >> 2;

        return Signature<CYCLE_COUNT>::ReturnValue { cycle_count };
    };

    const auto handle_preset = []() {
        constexpr auto CYCLE = commands::cycler::CYCLE;
        using Signature = commands::Signature<CYCLE>;
        Signature::Parameters params {};
        commands::send<CYCLE>(params);
    };

    const auto handle_set_warning 
            = [&config, &write_config](Signature<SET_WARNING>::Parameters params) {
        const auto [warning_level, warning] = params;
        uint8_t new_warnings = config.as_struct.warnings;
        new_warnings = new_warnings | (1 << warning);
        config.as_struct.warnings = new_warnings;
        write_config(config.as_words);
        WarningLevel new_warning_level = warnings::_warning_level_from_warnings(1 << warning);
        if (new_warning_level >= warnings::warning_level) {
            warnings::warning_level = new_warning_level;
            warnings::active_warning = warning;
        }

        return Signature<SET_WARNING>::ReturnValue {};
    };

    const auto handle_get_warnings
            = [&config](Signature<GET_WARNINGS>::Parameters params) {
        uint8_t warnings = config.as_struct.warnings;
        return Signature<GET_WARNINGS>::ReturnValue { warnings };
    };

    const auto handle_get_active_warning
            = [](Signature<GET_ACTIVE_WARNING>::Parameters params) {
        return Signature<GET_ACTIVE_WARNING>::ReturnValue { warnings::active_warning };
    };

    const auto handle_get_warning_level
            = [](Signature<GET_WARNING_LEVEL>::Parameters params) {
        return Signature<GET_WARNING_LEVEL>::ReturnValue { warnings::warning_level };
    };

    const auto handle_reset_warnings
            = [&config, &write_config](Signature<RESET_WARNINGS>::Parameters params) {
        config.as_struct.warnings = 0;
        write_config(config.as_words);
        warnings::warning_level = WarningLevel::None;

        return Signature<RESET_WARNINGS>::ReturnValue {}; 
    };

    const auto handle_reset_warning 
            = [&config, &write_config](Signature<RESET_WARNING>::Parameters params) {
        const auto [warning] = params;
        uint8_t new_warnings = config.as_struct.warnings;
        new_warnings = new_warnings & ~(1 << warning);
        config.as_struct.warnings = new_warnings;
        write_config(config.as_words);
        WarningLevel new_warning_level = warnings::_warning_level_from_warnings(new_warnings);
        if (new_warning_level < warnings::warning_level) warnings::warning_level = new_warning_level;

        return Signature<RESET_WARNING>::ReturnValue {};
    };

    #define ERROR_TIME 1000
    TimerHandle_t _error_timer = xTimerCreate("error_timer", ERROR_TIME, pdTRUE, (void*) 0, warnings::_error_timer_callback);
    xTimerStart(_error_timer, 0);

    for (;;) {
        const uint32_t flags = _wait_flags(osFlagsWaitAny, osWaitForever);
        for (const auto& flag : _iterate_flags(flags)) {
            switch (flag) {
            case ThreadFlag::SET_TIME: {
                handle_queue<SET_TIME>(SET_TIME_QUEUE, handle_set_time);
                break;
            }
            case ThreadFlag::GET_TIME: {
                handle_queue<GET_TIME>(GET_TIME_QUEUE, handle_get_time);
                break;
            }
            case ThreadFlag::SET_TIMER: {
                handle_queue<SET_TIMER>(SET_TIMER_QUEUE, handle_set_timer);
                break;
            }
            case ThreadFlag::GET_TIMER: {
                handle_queue<GET_TIMER>(GET_TIMER_QUEUE, handle_get_timer);
                break;
            }
            case ThreadFlag::SET_DISPENSE_TIME: {
                handle_queue<SET_DISPENSE_TIME>(
                    SET_DISPENSE_TIME_QUEUE, handle_set_dispense_time);
                break;
            }
            case ThreadFlag::GET_DISPENSE_TIME: {
                handle_queue<GET_DISPENSE_TIME>(
                    GET_DISPENSE_TIME_QUEUE, handle_get_dispense_time);
                break;
            }
            case ThreadFlag::SET_NUM_RINSES: {
                handle_queue<SET_NUM_RINSES>(
                    SET_NUM_RINSES_QUEUE, handle_set_num_rinses);
                break;
            }
            case ThreadFlag::GET_NUM_RINSES: {
                handle_queue<GET_NUM_RINSES>(
                    GET_NUM_RINSES_QUEUE, handle_get_num_rinses);
                break;
            }
            case ThreadFlag::SET_NUM_CYCLES: {
                handle_queue<SET_NUM_CYCLES>(
                    SET_NUM_CYCLES_QUEUE, handle_set_num_cycles);
                break;
            }
            case ThreadFlag::GET_NUM_CYCLES: {
                handle_queue<GET_NUM_CYCLES>(
                    GET_NUM_CYCLES_QUEUE, handle_get_num_cycles);
                break;
            }
            case ThreadFlag::SET_CONFIGURED: {
                handle_queue<SET_CONFIGURED>(
                    SET_CONFIGURED_QUEUE, handle_set_configured);
                break;
            }
            case ThreadFlag::GET_CONFIGURED: {
                handle_queue<GET_CONFIGURED>(
                    GET_CONFIGURED_QUEUE, handle_get_configured);
                break;
            }
            case ThreadFlag::UPDATE_VBAT: {
                handle_queue<UPDATE_VBAT>(
                    UPDATE_VBAT_QUEUE, handle_update_vbat);
                break;
            }
            case ThreadFlag::GET_VBAT: {
                handle_queue<GET_VBAT>(
                    GET_VBAT_QUEUE, handle_get_vbat);
                break;
            }
            case ThreadFlag::SET_HOMED: {
                handle_queue<SET_HOMED>(
                    SET_HOMED_QUEUE, handle_set_homed);
                break;
            }
            case ThreadFlag::GET_HOMED: {
                handle_queue<GET_HOMED>(
                    GET_HOMED_QUEUE, handle_get_homed);
                break;
            }
            case ThreadFlag::SET_CYCLING: {
                handle_queue<SET_CYCLING>(
                    SET_CYCLING_QUEUE, handle_set_cycling);
                break;
            }
            case ThreadFlag::GET_CYCLING: {
                handle_queue<GET_CYCLING>(
                    GET_CYCLING_QUEUE, handle_get_cycling);
                break;
            }
            case ThreadFlag::SET_CYCLES_RUN: {
                handle_queue<SET_CYCLES_RUN>(
                    SET_CYCLES_RUN_QUEUE, handle_set_cycles_run);
                break;
            }
            case ThreadFlag::GET_CYCLES_RUN: {
                handle_queue<GET_CYCLES_RUN>(
                    GET_CYCLES_RUN_QUEUE, handle_get_cycles_run);
                break;
            }
            case ThreadFlag::CYCLE_MODE: {
                handle_queue<CYCLE_MODE>(
                    CYCLE_MODE_QUEUE, handle_cycle_mode);
                break;
            }
            case ThreadFlag::CYCLE_COUNT: {
                handle_queue<CYCLE_COUNT>(
                    CYCLE_COUNT_QUEUE, handle_cycle_count);
                break;
            }
            case ThreadFlag::SET_WARNING: {
                handle_queue<SET_WARNING>(
                    SET_WARNING_QUEUE, handle_set_warning);
                break;
            }
            case ThreadFlag::GET_WARNINGS: {
                handle_queue<GET_WARNINGS>(
                    GET_WARNINGS_QUEUE, handle_get_warnings);
                break;
            }
            case ThreadFlag::GET_ACTIVE_WARNING: {
                handle_queue<GET_ACTIVE_WARNING>(
                    GET_ACTIVE_WARNING_QUEUE, handle_get_active_warning);
                break;
            }
            case ThreadFlag::GET_WARNING_LEVEL: {
                handle_queue<GET_WARNING_LEVEL>(
                    GET_WARNING_LEVEL_QUEUE, handle_get_warning_level);
                break;
            }
            case ThreadFlag::RESET_WARNINGS: {
                handle_queue<RESET_WARNINGS>(
                    RESET_WARNINGS_QUEUE, handle_reset_warnings);
                break;
            }
            case ThreadFlag::RESET_WARNING: {
                handle_queue<RESET_WARNING>(
                    RESET_WARNING_QUEUE, handle_reset_warning);
                break;
            }
            case ThreadFlag::PRESET: {
                handle_preset();
                break;
            }
            case ThreadFlag::UNKNOWN: {
                break;
            }
            }
        }
    }
}

extern "C" void init_config_task_globals(osThreadId_t handle)
{
    THREAD.set_handle(handle);
#define INIT_QUEUE(COMMAND) commands::config::COMMAND##_QUEUE.init();
    FOR_ALL_CONFIG_COMMANDS(INIT_QUEUE)
#undef INIT_QUEUE
}

extern "C" void start_config_task(void* parameters)
{
    auto params_array = static_cast<void**>(parameters);
    auto hrtc = static_cast<RTC_HandleTypeDef*>(params_array[0]);
    THREAD.target(hrtc);
}
