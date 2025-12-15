/**
 * @file
 *
 * Task for cycling.
 */

#include <cstdint>

#include "cmsis_os.h"

#include "main.h"

#include "commands/base.hpp"
#include "commands/config.hpp"
#include "commands/cycler.hpp"
#include "commands/heater.hpp"
#include "commands/syringe.hpp"
#include "commands/thermocouple.hpp"
#include "commands/valve.hpp"
#include "commands/warnings.hpp"

#include "ui/menu.hpp"

#include "drivers/display.hpp"
#include "helpers/cmsis_os.hpp"

/////////////////////////
// Thread event flags. //
/////////////////////////

using Status = defines::Status;

namespace {

#define FOR_ALL_THREAD_FLAGS(FUNC) FOR_ALL_CYCLER_COMMANDS(FUNC)

enum class ThreadFlag : std::uint32_t {
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

    void target(I2C_HandleTypeDef* hi2c, osMutexId_t i2c_mutex_handle,
        osMutexId_t display_mutex_handle);
} THREAD;

} // anonyous namespace

///////////////////////
// Controller Queues //
///////////////////////

namespace commands::cycler {

constexpr std::size_t REQUEST_QUEUE_SIZE = 1;

#define DECLARE_QUEUE(COMMAND)                                                         \
    static Queue<COMMAND, REQUEST_QUEUE_SIZE> COMMAND##_QUEUE;
FOR_ALL_CYCLER_COMMANDS(DECLARE_QUEUE)
#undef DECLARE_QUEUE

} // namespace commands::cycler

namespace commands {

#define DEFINE_QUEUE_PUT(COMMAND)                                                      \
    template <>                                                                        \
    osStatus_t queue_put<cycler::COMMAND>(                                             \
        const QueueItem<cycler::COMMAND>& item, uint32_t timeout)                      \
    {                                                                                  \
        return cycler::COMMAND##_QUEUE.put(item, timeout);                             \
    }
FOR_ALL_CYCLER_COMMANDS(DEFINE_QUEUE_PUT)
#undef DEFINE_QUEUE_PUT

#define DEFINE_SET_THREAD_FLAG(COMMAND)                                                \
    template <> void set_thread_flag<cycler::COMMAND>(void)                            \
    {                                                                                  \
        THREAD.set_flag(ThreadFlag::COMMAND);                                          \
    }
FOR_ALL_CYCLER_COMMANDS(DEFINE_SET_THREAD_FLAG)
#undef DEFINE_SET_THREAD_FLAG

} // namespace commands

namespace commands::cycler {

Status rehome(void)
{
    const auto params = std::make_tuple();
    ReturnValue<REHOME> return_value {};

    const auto send_status = send_and_block<REHOME>(params, return_value);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status prime(void)
{
    const auto params = std::make_tuple();
    ReturnValue<PRIME> return_value {};

    const auto send_status = send_and_block<PRIME>(params, return_value);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status cycle(void)
{
    const auto params = std::make_tuple();
    ReturnValue<CYCLE> return_value {};

    const auto send_status = send_and_block<CYCLE>(params, return_value);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status rinse(void)
{
    const auto params = std::make_tuple();
    ReturnValue<RINSE> return_value {};

    const auto send_status = send_and_block<RINSE>(params, return_value);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status manual_rinse(void)
{
    const auto params = std::make_tuple();
    ReturnValue<MANUAL_RINSE> return_value {};

    const auto send_status = send_and_block<MANUAL_RINSE>(params, return_value);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status rot_syringe_test(void)
{
    const auto params = std::make_tuple();
    ReturnValue<ROT_SYRINGE_TEST> return_value {};

    const auto send_status = send_and_block<ROT_SYRINGE_TEST>(params, return_value);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status error_state(void)
{
    const auto params = std::make_tuple();
    ReturnValue<ERROR_STATE> return_value {};

    const auto send_status = send_and_block<ERROR_STATE>(params, return_value);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

} // namespace commands::cycler

/////////////////////////
// Timer thread target //
/////////////////////////
void Thread::target(I2C_HandleTypeDef* hi2c, osMutexId_t i2c_mutex_handle,
    osMutexId_t display_mutex_handle)
{
    using Mutex = cmsis_os::Mutex;
    Mutex i2c_mutex(i2c_mutex_handle);
    drivers::I2C i2c(hi2c, i2c_mutex);

    Mutex display_mutex(display_mutex_handle);

    using Display = drivers::Display;
    Display display(i2c);

    display_mutex.acquire(osWaitForever);
    display.init();
    display_mutex.release();

    using namespace warnings;

    /////////////
    // Helpers //
    /////////////

    const auto enable_gpio_interrupts = []() {
        HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
        HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
        HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    };

    const auto disable_gpio_interrupts = []() {
        HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
        HAL_NVIC_DisableIRQ(EXTI2_3_IRQn);
        HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
    };

    const auto rehome_system = [&display](void) {
        using namespace commands;

        display.line1_puts("      REHOMING      ");
        display.line2_puts("        ...         ");

        const auto rehome_status = [&display]() {
            auto status = valve::rehome();
            osDelay(1'000);
            if (not defines::status::is_ok(status)) {
                set_warning(WarningLevel::High, Warning::Rotary_Homing);
                ui_warning(WarningLevel::High);
                return defines::status::map_from(status);
            }

            status = valve::move_to(valve::Position::WATER_OUT);
            osDelay(1'000);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            // status = valve::move_to(valve::Position::LIGHTNING);
            // osDelay(1'000);
            // if (not defines::status::is_ok(status)) {
            //     return defines::status::map_from(status);
            // }

            // status = valve::move_to(valve::Position::SPORES);
            // osDelay(1'000);
            // if (not defines::status::is_ok(status)) {
            //     return defines::status::map_from(status);
            // }

            status = syringe::rehome();
            if (not defines::status::is_ok(status)) {
                set_warning(WarningLevel::Critical, Warning::Syringe_Homing);
                return defines::status::map_from(status);
            }

            status = valve::move_to(valve::Position::HOME);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }
            return Status::OK;
        }();

        if (defines::status::is_ok(rehome_status)) {
            display.line1_puts("      REHOMING      ");
            display.line2_puts("      COMPLETE      ");
            config::set_homed(true);
        } else {
            display.line1_puts("      REHOMING      ");
            display.line2_puts("       FAILED       ");
        }
        osDelay(3'000);

        return rehome_status;
    };

    const auto rinse = [&display, &rehome_system](void) {
        using namespace commands;

        bool homed = config::get_homed();
        if (!homed) {
            auto status = rehome_system();
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }
        }

        const auto [get_status, num_rinses] = commands::config::get_num_rinses();
        if (not defines::status::is_ok(get_status)) {
            return defines::status::map_from(get_status);
        }

        // const auto num_pulls_per_rinse = decltype(num_rinses) { 9 };
        // const auto num_pulls = num_pulls_per_rinse * num_rinses;
        for (auto rinse = decltype(num_rinses) { 0 }; rinse < num_rinses; rinse++) {
            display.line1_puts("      RINSING       ");
            display.line2_printf("      %02d / %02d      ", (rinse + 1), num_rinses);

            auto status = valve::move_to(valve::Position::WATER_IN);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            status = syringe::fill();
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            status = valve::move_to(valve::Position::WATER_OUT);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            status = syringe::rehome();
            if (not defines::status::is_ok(status)) {
                set_warning(WarningLevel::Critical, Warning::Syringe_Homing);
                return defines::status::map_from(status);
            }
        }

        const auto move_status = valve::move_to(valve::Position::HOME);
        return defines::status::map_from(move_status);
    };

    const auto manual_rinse = [&display](void) {
        using namespace commands;

        auto status = valve::move_to(valve::Position::HOME);
        if (not defines::status::is_ok(status)) {
            return defines::status::map_from(status);
        }

        display.line1_puts("      RINSING       ");
        display.line2_puts("                    ");

        drivers::gpio::pins::SOL_VALVE.set();

        osDelay(10'000);

        drivers::gpio::pins::SOL_VALVE.reset();

        return Status::OK;
    };

    const auto rot_syringe_test = [&display, &rehome_system](void) {
        using namespace commands;

        bool homed = config::get_homed();
        if (!homed) {
            auto status = rehome_system();
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }
        }

        const auto pull_status = [&display]() {
            display.line1_puts("       DOSING       ");
            display.line2_puts("       WATER        ");

            auto status = valve::move_to(valve::Position::WATER_IN);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            const uint32_t dosing_water_ml = 30;
            status = syringe::pull(dosing_water_ml);
            if (not defines::status::is_ok(status)) {
                return status;
            }

            return Status::OK;
        }();

        if (defines::status::is_ok(pull_status)) {
            display.line1_puts("       DOSING       ");
            display.line2_puts("     COMPLETED      ");
        } else {
            display.line1_puts("       DOSING       ");
            display.line2_puts("       FAILED       ");
        }
        osDelay(3'000);

        display.line1_puts("     DISPENSING     ");
        display.line2_puts("        ...         ");

        const auto dispense_status = []() {
            auto status = valve::move_to(valve::Position::WATER_OUT);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            status = syringe::rehome();
            if (not defines::status::is_ok(status)) {
                set_warning(WarningLevel::Critical, Warning::Syringe_Homing);
                return defines::status::map_from(status);
            }

            status = valve::move_to(valve::Position::HOME);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            return Status::OK;
        }();

        if (defines::status::is_ok(dispense_status)) {
            display.line1_puts("      DISPENSE      ");
            display.line2_puts("      COMPLETED     ");
        } else {
            display.line1_puts("      DISPENSE      ");
            display.line2_puts("       FAILED       ");
        }
        osDelay(3'000);

        return Status::OK;
    };

    const auto prime = [&display, &rinse, &rehome_system](void) {
        using namespace commands;

        bool homed = config::get_homed();
        if (!homed) {
            auto status = rehome_system();
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }
        }

        const auto pull_status = [&display]() {
            display.line1_puts("       DOSING       ");
            display.line2_puts("       WATER        ");

            auto status = valve::move_to(valve::Position::WATER_IN);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            const uint32_t dosing_water_ml = 30;
            status = syringe::pull(dosing_water_ml);
            if (not defines::status::is_ok(status)) {
                return status;
            }

            display.line1_puts("       DOSING       ");
            display.line2_puts("       SPORES       ");

            status = valve::move_to(valve::Position::SPORES);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            const uint32_t dosing_spores_ml = 20;
            status = syringe::pull(dosing_spores_ml);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            display.line1_puts("       DOSING       ");
            display.line2_puts("    SUPERCHARGER    ");

            status = valve::move_to(valve::Position::SUPERCHARGER);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            const uint32_t dosing_supercharger_ml = 10;
            status = syringe::pull(dosing_supercharger_ml);
            if (not defines::status::is_ok(status)) {
                return status;
            }

            return Status::OK;
        }();

        if (defines::status::is_ok(pull_status)) {
            display.line1_puts("       DOSING       ");
            display.line2_puts("     COMPLETED      ");
        } else {
            display.line1_puts("       DOSING       ");
            display.line2_puts("       FAILED       ");
        }
        osDelay(3'000);

        display.line1_puts("     DISPENSING     ");
        display.line2_puts("        ...         ");

        const auto dispense_status = []() {
            auto status = valve::move_to(valve::Position::WATER_OUT);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            status = syringe::rehome();
            if (not defines::status::is_ok(status)) {
                set_warning(WarningLevel::Critical, Warning::Syringe_Homing);
                return defines::status::map_from(status);
            }

            status = valve::move_to(valve::Position::HOME);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            return Status::OK;
        }();

        if (defines::status::is_ok(dispense_status)) {
            display.line1_puts("      DISPENSE      ");
            display.line2_puts("     COMPLETED      ");
        } else {
            display.line1_puts("      DISPENSE      ");
            display.line2_puts("       FAILED       ");
        }
        osDelay(3'000);

        return Status::OK;
    };

    const auto cycle = [&display, &rinse, &rehome_system](void) {
        using namespace commands;

        const auto [_, cycles_run] = config::get_cycles_run();
        config::set_cycles_run(cycles_run + 1);

        /*display.line1_puts("      TESTING       ");
        display.line2_puts("                    ");
        osDelay(750);
        return Status::OK;*/

        bool homed = config::get_homed();
        if (!homed) {
            auto status = rehome_system();
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }
        }
        
        const auto pull_status = [&display]() {
            enum Dose {
                WATER,
                SPORES,
                BUFFER
            };

            auto [_, cycle_mode] = config::get_cycle_mode();
            uint32_t doses[3] = { 35, 20, 10 };

            {
                using namespace config;

                switch (cycle_mode) {
                    case CycleMode::WASTEWATER:
                        doses[Dose::WATER] = 20;
                        doses[Dose::SPORES] = 10;
                        doses[Dose::BUFFER] = 4;
                        break;
                    case CycleMode::ANAEROBIC:
                        doses[Dose::WATER] = 100;
                        doses[Dose::SPORES] = 50;
                        doses[Dose::BUFFER] = 20;
                        break;
                }
            }

            display.line1_puts("       DOSING       ");
            display.line2_puts("       WATER        ");

            auto status = valve::move_to(valve::Position::WATER_IN);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            const uint32_t dosing_water_ml = doses[Dose::WATER];
            status = syringe::pull(dosing_water_ml);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            display.line1_puts("       DOSING       ");
            display.line2_puts("       SPORES       ");

            status = valve::move_to(valve::Position::SPORES);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            const uint32_t dosing_spores_ml = doses[Dose::SPORES];
            status = syringe::pull(dosing_spores_ml);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            display.line1_puts("       DOSING       ");
            display.line2_puts("    SUPERCHARGER    ");

            status = valve::move_to(valve::Position::SUPERCHARGER);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            const uint32_t dosing_supercharger_ml = doses[Dose::BUFFER];
            status = syringe::pull(dosing_supercharger_ml);
            if (not defines::status::is_ok(status)) {
                return status;
            }

            return Status::OK;
        }();

        if (defines::status::is_ok(pull_status)) {
            display.line1_puts("       DOSING       ");
            display.line2_puts("     COMPLETED      ");
        } else {
            display.line1_puts("       DOSING       ");
            display.line2_puts("       FAILED       ");
        }
        osDelay(3'000);

        constexpr uint32_t mixing_volume_ml = 5;

        display.line1_puts("     ACTIVATING     ");
        display.line2_puts("        ...         ");

        const auto activating_status = [&display](void) {
            constexpr uint32_t activating_time = 45 * 60 * 1'000;
            uint32_t total_time = 0;
            uint32_t pushpull_time = 0;
            uint32_t pushpull_timeout = 300'000;
            constexpr uint32_t loop_delay = 1'000;
            constexpr uint32_t pushpull_delay = 5'000;

            const uint32_t overheat_timeout = 5 * 60 * 1000;
            const int16_t overheat_temp = 110;
            uint32_t overheat_time = 0;

            const uint32_t underheat_timeout = 5 * 60 * 1000;
            const int16_t underheat_temp = 80;
            uint32_t underheat_time = 0;

            constexpr int16_t target = 36;
            commands::heater::turn_on(target);

            while (true) {
                const auto start = osKernelGetTickCount();

                int16_t heater, liquid;
                bool overheated = false;

                // Overheat causes instant fail
                // Timer is still implemented but unused
                auto status = commands::thermocouple::read(heater, liquid, overheated);
                if (not defines::status::is_ok(status)) {
                    if (overheated) {
                        set_warning(WarningLevel::High, Warning::High_Temp);
                        ui_warning(WarningLevel::High);
                    }
                    
                    return defines::status::map_from(status);
                }

                const uint32_t minutes = total_time / (60 * 1'000);
                display.line1_puts("     ACTIVATING     ");
                display.line2_printf("H: %3dC        M: %2d", heater, minutes);

                // If heater on then timeout to push/pull is 30 sec, ekse 5 min
                if (pushpull_time >= pushpull_timeout) {
                    status = commands::syringe::push(mixing_volume_ml);
                    if (not defines::status::is_ok(status)) {
                        return defines::status::map_from(status);
                    }

                    osDelay(pushpull_delay);

                    status = commands::syringe::pull(mixing_volume_ml);
                    if (not defines::status::is_ok(status)) {
                        return defines::status::map_from(status);
                    }

                    pushpull_time = 0;
                }

                osDelay(loop_delay);
                total_time += osKernelGetTickCount() - start;
                pushpull_time += osKernelGetTickCount() - start;
                
                if (heater >= overheat_temp) {
                    overheat_time += osKernelGetTickCount() - start;
                }
                else {
                    overheat_time = 0;
                }

                if (heater < underheat_temp) {
                    underheat_time += osKernelGetTickCount() - start;
                }
                else {
                    underheat_time = 0;
                }

                if (overheat_time >= overheat_timeout) {
                    set_warning(WarningLevel::High, Warning::High_Temp);
                    ui_warning(WarningLevel::High);
                    return Status::ERROR;
                }

                if (underheat_time >= underheat_timeout) {
                    set_warning(WarningLevel::Normal, Warning::Low_Temp);
                }

                if (total_time >= activating_time) {
                    return Status::OK;
                }
            }

            return Status::OK;
        }();

        if (defines::status::is_ok(activating_status)) {
            display.line1_puts("     ACTIVATING     ");
            display.line2_puts("     COMPLETED      ");
        } else {
            display.line1_puts("     ACTIVATING     ");
            display.line2_puts("      FAILED        ");
        }
        osDelay(3'000);

        commands::heater::turn_off();

        display.line1_puts("     DISPENSING     ");
        display.line2_puts("        ...         ");

        const auto dispense_status = []() {
            auto status = valve::move_to(valve::Position::WATER_OUT);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            status = syringe::rehome();
            if (not defines::status::is_ok(status)) {
                set_warning(WarningLevel::Critical, Warning::Syringe_Homing);
                return defines::status::map_from(status);
            }

            status = valve::move_to(valve::Position::HOME);
            if (not defines::status::is_ok(status)) {
                return defines::status::map_from(status);
            }

            return Status::OK;
        }();

        if (defines::status::is_ok(dispense_status)) {
            display.line1_puts("      DISPENSE      ");
            display.line2_puts("      COMPLETED     ");
        } else {
            display.line1_puts("      DISPENSE      ");
            display.line2_puts("       FAILED       ");
        }
        osDelay(3'000);

        // For Ecostorm XL, open the valve for set amount of time
        // based on number of rinses requested
        {
            const auto [get_status, num_rinses] = commands::config::get_num_rinses();
            if (not defines::status::is_ok(get_status)) {
                return defines::status::map_from(get_status);
            }

            display.line1_puts("   FLUSHING SYSTEM   ");
            display.line2_puts("                     ");

            if (num_rinses > 0) {
                // Enable flush solenoid
                drivers::gpio::pins::SOL_VALVE.set();

                // Delay 108 sec for every rinse specified
                for (int index = 0; index < num_rinses; index++) {
                    display.line2_printf(
                        "         %1d/%1d         ", index + 1, num_rinses);
                    osDelay(108'000);
                }

                // Disable flush solenoid
                drivers::gpio::pins::SOL_VALVE.reset();
            }
            else {
                display.line2_puts("    (No solenoid)    ");
                
                const auto flush_status = []() {
                    for (int i = 0; i < 2; i++) {
                        auto status = valve::move_to(valve::Position::WATER_IN);
                        if (not defines::status::is_ok(status)) {
                            return defines::status::map_from(status);
                        }

                        const uint32_t dosing_water_ml = 200;
                        status = syringe::pull(dosing_water_ml);
                        if (not defines::status::is_ok(status)) {
                            return defines::status::map_from(status);
                        }

                        status = valve::move_to(valve::Position::WATER_OUT);
                        if (not defines::status::is_ok(status)) {
                            return defines::status::map_from(status);
                        }

                        status = syringe::rehome();
                        if (not defines::status::is_ok(status)) {
                            set_warning(WarningLevel::Critical, Warning::Syringe_Homing);
                            return defines::status::map_from(status);
                        }
                    }

                    auto status = valve::move_to(valve::Position::HOME);
                    if (not defines::status::is_ok(status)) {
                        return defines::status::map_from(status);
                    }

                    return Status::OK;
                }();

                if (defines::status::is_ok(flush_status)) {
                    display.line1_puts("        FLUSH       ");
                    display.line2_puts("      COMPLETED     ");
                } else {
                    display.line1_puts("        FLUSH       ");
                    display.line2_puts("       FAILED       ");
                }
                osDelay(3'000);
            }
        }

        // Update cycle count
        {
            auto [cycle_status, num_cycles] = commands::config::get_num_cycles();
            if (defines::status::is_ok(cycle_status)) {
                commands::config::set_num_cycles(++num_cycles);
            }
        }

        return Status::OK;
    };

    const auto error_state = [&]() {
        using namespace commands;

        drivers::gpio::pins::SOL_VALVE.reset();
        valve::move_to(valve::Position::HOME);
        
        return Status::OK;
    };

    //////////////
    // Handlers //
    //////////////

    using namespace commands;
    using namespace commands::cycler;

    const auto handle_rehome = [&rehome_system](Signature<REHOME>::Parameters params) {
        ui::start_holding_awake();
        const auto status = rehome_system();
        ui::end_holding_awake();
        return Signature<REHOME>::ReturnValue { status };
    };

    const auto handle_prime = [&prime](Signature<PRIME>::Parameters params) {
        ui::start_holding_awake();
        const auto status = prime();
        ui::end_holding_awake();
        return Signature<PRIME>::ReturnValue { status };
    };

    const auto handle_cycle = [&cycle](Signature<CYCLE>::Parameters params) {
        commands::config::set_cycling(true);
        ui::start_holding_awake();
        const auto status = cycle();
        ui::end_holding_awake();
        commands::config::update_vbat();
        ui::resume_menu();
        commands::config::set_cycling(false);
        return Signature<CYCLE>::ReturnValue { status };
    };

    const auto handle_rinse = [&rinse](Signature<RINSE>::Parameters params) {
        ui::start_holding_awake();
        const auto status = rinse();
        ui::end_holding_awake();
        return Signature<CYCLE>::ReturnValue { status };
    };

    const auto handle_manual_rinse = [&manual_rinse](Signature<MANUAL_RINSE>::Parameters params) {
        ui::start_holding_awake();
        const auto status = manual_rinse();
        ui::end_holding_awake();
        return Signature<MANUAL_RINSE>::ReturnValue { status };
    };

    const auto handle_rot_syringe_test = [&rot_syringe_test](Signature<ROT_SYRINGE_TEST>::Parameters params) {
        ui::start_holding_awake();
        const auto status = rot_syringe_test();
        ui::end_holding_awake();
        return Signature<MANUAL_RINSE>::ReturnValue { status };
    };

    const auto handle_error_state = [&](Signature<ERROR_STATE>::Parameters params) {
        const auto status = error_state();
        return Signature<ERROR_STATE>::ReturnValue { status };
    };

    while (true) {
        const uint32_t flags = _wait_flags(osFlagsWaitAny, osWaitForever);
        disable_gpio_interrupts();
        display_mutex.acquire(osWaitForever);
        for (const auto& flag : _iterate_flags(flags)) {
            switch (flag) {
            case ThreadFlag::REHOME: {
                handle_queue<REHOME>(REHOME_QUEUE, handle_rehome);
                break;
            }
            case ThreadFlag::PRIME: {
                handle_queue<PRIME>(PRIME_QUEUE, handle_prime);
                break;
            }
            case ThreadFlag::CYCLE: {
                handle_queue<CYCLE>(CYCLE_QUEUE, handle_cycle);
                break;
            }
            case ThreadFlag::RINSE: {
                handle_queue<RINSE>(RINSE_QUEUE, handle_rinse);
                break;
            }
            case ThreadFlag::MANUAL_RINSE: {
                handle_queue<MANUAL_RINSE>(MANUAL_RINSE_QUEUE, handle_manual_rinse);
                break;
            }
            case ThreadFlag::ROT_SYRINGE_TEST: {
                handle_queue<MANUAL_RINSE>(ROT_SYRINGE_TEST_QUEUE, handle_rot_syringe_test);
                break;
            }
            case ThreadFlag::ERROR_STATE: {
                handle_queue<ERROR_STATE>(ERROR_STATE_QUEUE, handle_error_state);
                break;
            }
            case ThreadFlag::UNKNOWN: {
                break;
            }
            }
        }
        display_mutex.release();
        enable_gpio_interrupts();
    }
}

extern "C" void init_cycler_task_globals(osThreadId_t handle)
{
    THREAD.set_handle(handle);
#define INIT_QUEUE(COMMAND) commands::cycler::COMMAND##_QUEUE.init();
    FOR_ALL_CYCLER_COMMANDS(INIT_QUEUE)
#undef INIT_QUEUE
}

extern "C" void start_cycler_task(void* parameters)
{
    auto params_array = static_cast<void**>(parameters);
    auto hi2c = static_cast<I2C_HandleTypeDef*>(params_array[0]);
    auto i2c_mutex_handle = static_cast<osMutexId_t>(params_array[1]);
    auto display_mutex_handle = static_cast<osMutexId_t>(params_array[2]);
    THREAD.target(hi2c, i2c_mutex_handle, display_mutex_handle);
}
