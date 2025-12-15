/**
 * @file
 *
 * Task for handling display and button in order to implement user interface.
 */

#include <cstdint>

#include "cmsis_os.h"

#include "main.h"

#include "timers.h"

#include "commands/config.hpp"
#include "commands/cycler.hpp"
#include "commands/warnings.hpp"

#include "defines/gpio.hpp"

#include "drivers/display.hpp"
#include "drivers/gpio.hpp"
#include "drivers/adc.hpp"

#include "helpers/button_filter.hpp"
#include "helpers/cmsis_os.hpp"

#include "ui/menu.hpp"
#include "ui/hidden_code.hpp"

/////////////////////////
// Thread event flags. //
/////////////////////////

using Status = defines::Status;

namespace {

#define FOR_ALL_THREAD_FLAGS(FUNC)                                                     \
    FUNC(MOVE_UP)                                                                      \
    FUNC(MOVE_DOWN)                                                                    \
    FUNC(MOVE_UP_HOLDING)                                                              \
    FUNC(MOVE_DOWN_HOLDING)                                                            \
    FUNC(CRITICAL_WARNING)                                                             \
    FUNC(HIGH_WARNING)                                                                 \
    FUNC(BATTERY_WARNING)                                                              \
    FUNC(SELECT)                                                                       \
    FUNC(DEBUG)                                                                        \
    FUNC(TIMEOUT)                                                                                                                              

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

    void target(I2C_HandleTypeDef* hi2c, osMutexId_t i2c_mutex_handle,
        osMutexId_t display_mutex_handle);

} THREAD;

} // anonyous namespace

/////////////////////////////
// Button Debounce Filters //
/////////////////////////////

namespace filters {

class : public helpers::ButtonFilter {
private:
    void _on_press(void) override { 
        THREAD.set_flag(::ThreadFlag::SELECT); 
    }
    void _on_release(void) override { }
    void _on_hold(void) override {  }
    bool _button_pressed(void) override
    {
        return drivers::gpio::pins::USER_1.read() == GPIO_PIN_RESET;
    }
} USER_1;

class : public helpers::ButtonFilter {
private:
    void _on_press(void) override { 
        THREAD.set_flag(::ThreadFlag::MOVE_DOWN);
    }
    void _on_release(void) override { }
    void _on_hold(void) override { THREAD.set_flag(::ThreadFlag::MOVE_DOWN_HOLDING); }
    bool _button_pressed(void) override
    {
        return drivers::gpio::pins::USER_2.read() == GPIO_PIN_RESET;
    }
} USER_2;

class : public helpers::ButtonFilter {
private:
    void _on_press(void) override { 
        THREAD.set_flag(::ThreadFlag::MOVE_UP); 
    }
    void _on_release(void) override { }
    void _on_hold(void) override { THREAD.set_flag(::ThreadFlag::MOVE_UP_HOLDING); }
    bool _button_pressed(void) override
    {
        return drivers::gpio::pins::USER_3.read() == GPIO_PIN_RESET;
    }
} USER_3;

void init_all(void)
{
#define INIT_BUTTON_FILTER(BUTTON) filters::BUTTON.init();
    FOR_ALL_BUTTONS(INIT_BUTTON_FILTER)
#undef INIT_BUTTON_FILTER
}

} // namespace filters

namespace ui {
    #define SLEEP_TIME 300'000
    bool awake = true;
    bool hold_awake = false;
    TimerHandle_t _sleep_timer = NULL;

    void start_holding_awake() {
        if (_sleep_timer) xTimerStop(_sleep_timer, 0);
        auto& LCD_BL = drivers::gpio::pins::LCD_BL;
        LCD_BL.set();
        hold_awake = true;
    }

    void end_holding_awake() {
        if (_sleep_timer) xTimerReset(_sleep_timer, 0);
        awake = true;
        hold_awake = false;
    }

    void resume_menu() {
        THREAD.set_flag(::ThreadFlag::TIMEOUT);
    }
}

namespace warnings {
    void ui_warning(WarningLevel level) {
        switch (level) {
            case WarningLevel::Critical:
                THREAD.set_flag(::ThreadFlag::CRITICAL_WARNING);
                break;
            case WarningLevel::High:
                THREAD.set_flag(::ThreadFlag::HIGH_WARNING);
                break;
            case WarningLevel::Normal:
                THREAD.set_flag(::ThreadFlag::BATTERY_WARNING);
                break;
        }
    }

    void _test_callback(TimerHandle_t timer) {
        set_warning(WarningLevel::Normal, Warning::Low_Temp);
    }
} // namespace warnings 

////////////////////////////
// GPIO button callbacks. //
////////////////////////////

namespace drivers::gpio::pins::types {

#define DEFINE_BUTTON_CALLBACKS(BUTTON)                                                \
    void BUTTON::on_rising(void) { filters::BUTTON.signal_change_event_from_isr(); }   \
    void BUTTON::on_falling(void) { filters::BUTTON.signal_change_event_from_isr(); }
FOR_ALL_BUTTONS(DEFINE_BUTTON_CALLBACKS)
#undef DEFINE_BUTTON_CALLBACKS

} // namespace drivers::gpio::pins::types


void _sleeping_timer_callback(TimerHandle_t timer){
    using Display = drivers::Display;

    Display* display = static_cast<Display*>(pvTimerGetTimerID(timer));

    display->line1_puts("    Ecostorm  XL    ");
    display->line2_puts("--------------------");
    
    auto& LCD_BL = drivers::gpio::pins::LCD_BL;
    LCD_BL.reset();
    ui::awake = false;
}

//////////////////////
// UI thread target //
//////////////////////

void Thread::target(I2C_HandleTypeDef* hi2c, osMutexId_t i2c_mutex_handle,
    osMutexId_t display_mutex_handle)
{
    filters::init_all();
    using namespace ui;

    drivers::gpio::pins::LED1.set();
    drivers::gpio::pins::LED2.set();
    drivers::gpio::pins::LED3.set();
    drivers::gpio::pins::LED4.set();

    // Disable flush solenoid
    drivers::gpio::pins::SOL_VALVE.reset();

    auto& LCD_RESET = drivers::gpio::pins::LCD_RESET;
    LCD_RESET.reset();
    osDelay(100);

    auto& LCD_BL = drivers::gpio::pins::LCD_BL;
    LCD_BL.set();
    osDelay(100);

    using Mutex = cmsis_os::Mutex;
    Mutex i2c_mutex(i2c_mutex_handle);
    drivers::I2C i2c(hi2c, i2c_mutex);

    Mutex display_mutex(display_mutex_handle);

    using Display = drivers::Display;
    Display display(i2c);

    LCD_RESET.set();
    osDelay(100);

    display_mutex.acquire(osWaitForever);
    display.init();
    display.clear();
    display_mutex.release();

    display.line1_puts("    Ecostorm  XL    ");
    display.line2_puts("--------------------");
    osDelay(2'000);
    display.line2_puts(VERSION_STRING);
    osDelay(5'000);

    const auto warning_flash = [&]() {
        LCD_BL.set();
        osDelay(125);
        LCD_BL.reset();
        osDelay(125);
        LCD_BL.set();
        osDelay(125);
        LCD_BL.reset();
        osDelay(125);
        LCD_BL.set();
    };

    {
        using namespace warnings;

        uint8_t warnings = get_warnings();
        
        if (_is_any_warning(warnings)) {
            display.line1_puts("!  PENDING ALARMS  !");
            
            for (int i = 0; i < 2; i++) {
                display.line2_puts("                    ");
                warning_flash();
                osDelay(500);

                if (_is_warning(warnings, Warning::Overflow)) {
                    display.line2_puts("      Overflow      ");
                    osDelay(1'000);
                }
                if (_is_warning(warnings, Warning::Rotary_Homing)) {
                    display.line2_puts("   Rotary Homing    ");
                    osDelay(1'000);
                }
                if (_is_warning(warnings, Warning::Syringe_Homing)) {
                    display.line2_puts("   Syringe Homing   ");
                    osDelay(1'000);
                }
                if (_is_warning(warnings, Warning::High_Temp)) {
                    display.line2_puts("  High Temperature  ");
                    osDelay(1'000);
                }
                if (_is_warning(warnings, Warning::Low_Temp)) {
                    display.line2_puts("  Low Temperature   ");
                    osDelay(1'000);
                }
                if (_is_warning(warnings, Warning::Low_Battery)) {
                    display.line2_puts("    Low Battery     ");
                    osDelay(1'000);
                }
            }
        }
    }

#ifdef DEBUG
    display.line1_puts("    Ecostorm  XL    ");
    display.line2_puts("----FAKE HOMING-----");
    osDelay(2'000);
#else
    commands::cycler::rehome();
#endif

    //TimerHandle_t _test_timer = xTimerCreate("test_timer", 3000, pdFALSE, (void*) 0, warnings::_test_callback);
    //xTimerStart(_test_timer, 0);

    ///////////////////
    // Menu commands //
    ///////////////////

    const auto wait_for_select = [this, &display, &display_mutex](const char* line1) {
        display_mutex.acquire(osWaitForever);
        display.line1_puts(line1);
        display.line2_puts("   [PRESS SELECT]   ");
        display_mutex.release();

        bool finished = false;
        while (not finished) {
            const auto flags = _wait_flags(osFlagsWaitAny, osWaitForever);
            auto iterable = _iterate_flags(flags);
            if (iterable.begin() == iterable.end()) {
                continue;
            }

            const auto flag = *iterable.begin();
            switch (flag) {
            case ThreadFlag::MOVE_UP:
                break;
            case ThreadFlag::MOVE_DOWN:
                break;
            case ThreadFlag::SELECT:
                finished = true;
                break;
            case ThreadFlag::TIMEOUT:
            case ThreadFlag::UNKNOWN:
                break;
            }
        }
    };

    const auto not_implemented = [&display, &display_mutex](const Menu* menu) {
        display_mutex.acquire(osWaitForever);
        display.line1_puts("        NOT         ");
        display.line2_puts("    IMPLEMENTED     ");
        display_mutex.release();
        osDelay(1'000);
        return nullptr;
    };

    const auto exit_menu = [](const Menu* menu) { return menu->parent; };

    const auto display_count = [this, &display, &display_mutex](const Menu* menu) {
        const auto [status, num_cycles] = commands::config::get_num_cycles();
        if (defines::status::is_ok(status)) {
            display.line2_printf("   Cycle #: % 5d   ", num_cycles);
        }
        return nullptr;
    };

    const auto prime = [&display](const Menu* menu) {
        using namespace warnings;

        uint8_t warnings = get_warnings();

        if (_is_any_warning(warnings)) {
            display.line1_puts("    ALARMS MUST     ");
            display.line2_puts("  BE CLEARED FIRST  ");
            osDelay(1'000);
            return nullptr;
        }

        commands::cycler::prime();
        return nullptr;
    };

    const auto cycle = [&display](const Menu* menu) {
        using namespace warnings;

        uint8_t warnings = get_warnings();

        if (_is_any_warning(warnings)) {
            display.line1_puts("    ALARMS MUST     ");
            display.line2_puts("  BE CLEARED FIRST  ");
            osDelay(1'000);
            return nullptr;
        }

        commands::cycler::cycle();
        return nullptr;
    };

    const auto rehome = [](const Menu* menu) {
        commands::cycler::rehome();
        return nullptr;
    };

    const auto cycle_continuously = [&display](const Menu* menu) {
        using namespace warnings;

        uint8_t warnings = get_warnings();

        if (_is_any_warning(warnings)) {
            display.line1_puts("    ALARMS MUST     ");
            display.line2_puts("  BE CLEARED FIRST  ");
            osDelay(1'000);
            return nullptr;
        }

        while (true) {
            commands::cycler::cycle();
        }
        return nullptr;
    };

    const auto manual_rinse = [](const Menu* menu) {
        commands::cycler::manual_rinse();
        return nullptr;
    };

    const auto rot_syringe_test = [](const Menu* menu) {
        commands::cycler::rot_syringe_test();
        return nullptr;
    };

    const auto edit_num_rinses = [this, &display, &display_mutex](const Menu* menu) {
        int num_rinses = 0;

        {
            const auto [status, num_rinses_] = commands::config::get_num_rinses();
            if (not defines::status::is_ok(status)) {
                display.line1_puts("       ERROR        ");
                display.line2_puts("  READING # RINSES  ");
                osDelay(3'000);
                return nullptr;
            }
            num_rinses = num_rinses_;
        }

        bool finished = false;
        while (not finished) {
            display_mutex.acquire(osWaitForever);
            display.clear();
            display.line1_puts("                    ");
            display.line2_printf("         %d         ", num_rinses);
            display_mutex.release();

            const auto flags = _wait_flags(osFlagsWaitAny, osWaitForever);
            auto iterable = _iterate_flags(flags);
            if (iterable.begin() == iterable.end()) {
                continue;
            }

            const auto flag = *iterable.begin();
            switch (flag) {
            case ThreadFlag::MOVE_UP:
                num_rinses++;
                break;
            case ThreadFlag::MOVE_DOWN:
                num_rinses--;
                break;
            case ThreadFlag::SELECT:
                finished = true;
                break;
            case ThreadFlag::TIMEOUT:
            case ThreadFlag::UNKNOWN:
                break;
            }

            constexpr auto min_num_rinses = decltype(num_rinses) { 0 };
            constexpr auto max_num_rinses = decltype(num_rinses) { 9 };
            num_rinses = std::max(min_num_rinses, std::min(max_num_rinses, num_rinses));
        }

        {
            const auto status = commands::config::set_num_rinses(num_rinses);
            display.line1_puts("   EDIT # RINSES    ");
            if (defines::status::is_ok(status)) {
                display.line2_puts("     SUCCESSFUL     ");
            } else {
                display.line2_puts("       FAILED       ");
            }
            osDelay(3'000);
        }

        return nullptr;
    };

    const auto edit_dispense_time = [this, &display, &display_mutex](const Menu* menu) {
        int hours = 0;
        int minutes = 0;
        constexpr int increment = 1;

        // Menu must be defined!
        if (menu == nullptr)
            return nullptr;

        {
            const auto [status, dispense_time_]
                = commands::config::get_dispense_time(menu->tag);
            if (not defines::status::is_ok(status)) {
                display.line1_puts("       ERROR        ");
                display.line2_puts("   READING TIMER    ");
                osDelay(3'000);
                return nullptr;
            }
            hours = dispense_time_ / 60;
            minutes = dispense_time_ - (hours * 60);
        }

        int changing = 0;

        while (changing < 2) {
            display_mutex.acquire(osWaitForever);
            display.line1_printf("       %02d:%02d        ", hours, minutes);
            switch (changing) {
                case 0:
                    display.line2_puts("       ^^           ");
                    break;
                case 1:
                    display.line2_puts("          ^^        ");
                    break;
            }
            display_mutex.release();

            const auto flags = _wait_flags(osFlagsWaitAny, osWaitForever);
            auto iterable = _iterate_flags(flags);
            if (iterable.begin() == iterable.end()) {
                continue;
            }

            const auto flag = *iterable.begin();
            switch (flag) {
                case ThreadFlag::MOVE_UP:
                case ThreadFlag::MOVE_UP_HOLDING:
                    if (changing == 0) {
                        hours += increment;
                    }
                    else {
                        minutes += increment;
                    }

                    break;
                case ThreadFlag::MOVE_DOWN:
                case ThreadFlag::MOVE_DOWN_HOLDING:
                    if (changing == 0) {
                        hours -= increment;
                    }
                    else {
                        minutes -= increment;
                    }

                    break;
                case ThreadFlag::SELECT:
                    changing++;
                    break;
                case ThreadFlag::TIMEOUT:
                case ThreadFlag::UNKNOWN:
                    break;
            }

            if (minutes > 59) {
                minutes = 0;
            }
            
            if (minutes < 0) {
                minutes = 59;
            }

            if (hours > 23) {
                hours = 0;
            }

            if (hours < 0) {
                hours = 23;
            }
        }

        {
            const int DISPENSE_OFFSET = 45;
            const int DAY_MINUTES = 1440;

            minutes = (hours * 60) + minutes;
            if (minutes > DAY_MINUTES - 1) minutes = 0;
            const auto status = commands::config::set_dispense_time(menu->tag, minutes);
            display.line1_puts(" EDIT DISPENSE TIME ");
            if (defines::status::is_ok(status)) {
                display.line2_puts("     SUCCESSFUL     ");
            } else {
                display.line2_puts("       FAILED       ");
            }

            // IF the time is edited then set the alarm to new time
            // For Ecostorm there is only one alarm / dispense time
            {
                {
                    int cycle_time = minutes - DISPENSE_OFFSET;
                    if (cycle_time < 0) cycle_time += DAY_MINUTES;
                    uint8_t hours = cycle_time / 60;
                    uint8_t minutes = cycle_time - (hours * 60);
                    commands::config::set_timer(hours, minutes);
                }
            }

            osDelay(3'000);
        }

        return nullptr;
    };

    const auto cycle_now = [&display, &display_mutex](const Menu* menu) {
        const auto [status, hours_, minutes_] = commands::config::get_time();
        if (!defines::status::is_ok(status)) {
            display.line1_puts("      GET TIME      ");
            display.line2_puts("       FAILED       ");
            return nullptr;
        }

        const int DISPENSE_OFFSET = 45;
        const int DAY_MINUTES = 1440;

        int new_cycle_time = (int) hours_ * 60 + (int) minutes_ + 1;
        if (new_cycle_time > DAY_MINUTES - 1) new_cycle_time -= DAY_MINUTES;
        int new_dispense_time = new_cycle_time + DISPENSE_OFFSET;
        if (new_dispense_time > DAY_MINUTES - 1) new_dispense_time -= DAY_MINUTES;

        const auto set_status = commands::config::set_dispense_time(0, new_dispense_time);
        if (!defines::status::is_ok(set_status)) {
            display.line1_puts(" SET DISPENSE TIME  ");
            display.line2_puts("       FAILED       ");
            return nullptr;
        }

        int new_hours = new_cycle_time / 60;
        int new_minutes = new_cycle_time - (new_hours * 60);
        commands::config::set_timer(new_hours, new_minutes);

        display.line1_puts(" STARTING CYCLE AT  ");
        display.line2_printf("       %02d:%02d        ", new_hours, new_minutes);

        osDelay(1'500);

        return nullptr;
    };

    const auto edit_system_time = [this, &display, &display_mutex](const Menu* menu) {
        int hours = 0;
        int minutes = 0;
        constexpr int increment = 1;

        {
            const auto [status, hours_, minutes_] = commands::config::get_time();
            if (not defines::status::is_ok(status)) {
                display.line1_puts("       ERROR        ");
                display.line2_puts("    READING TIME    ");
                osDelay(3'000);
                return nullptr;
            }
            hours = hours_;
            minutes = minutes_;
        }

        int changing = 0;
        while (changing < 2) {
            display_mutex.acquire(osWaitForever);
            display.line1_printf("       %02d:%02d        ", hours, minutes);
            switch (changing) {
                case 0:
                    display.line2_puts("       ^^           ");
                    break;
                case 1:
                    display.line2_puts("          ^^        ");
                    break;
            }
            display_mutex.release();

            const auto flags = _wait_flags(osFlagsWaitAny, osWaitForever);
            auto iterable = _iterate_flags(flags);
            if (iterable.begin() == iterable.end()) {
                continue;
            }

            const auto flag = *iterable.begin();
            switch (flag) {
                case ThreadFlag::MOVE_UP:
                case ThreadFlag::MOVE_UP_HOLDING:
                    if (changing == 0) {
                        hours += increment;
                    }
                    else {
                        minutes += increment;
                    }

                    break;
                case ThreadFlag::MOVE_DOWN:
                case ThreadFlag::MOVE_DOWN_HOLDING:
                    if (changing == 0) {
                        hours -= increment;
                    }
                    else {
                        minutes -= increment;
                    }

                    break;
                case ThreadFlag::SELECT:
                    changing++;
                    break;
                case ThreadFlag::TIMEOUT:
                case ThreadFlag::UNKNOWN:
                    break;
              
                
            }

            if (minutes > 59) {
                minutes = 0;
            }
            
            if (minutes < 0) {
                minutes = 59;
            }

            if (hours > 23) {
                hours = 0;
            }

            if (hours < 0) {
                hours = 23;
            }
        }

        {
            const auto status = commands::config::set_time(hours, minutes);
            display.line1_puts("  EDIT SYSTEM TIME  ");
            if (defines::status::is_ok(status)) {
                display.line2_puts("     SUCCESSFUL     ");
            } else {
                display.line2_puts("       FAILED       ");
            }
            osDelay(3'000);
        }

        return nullptr;
    };

    const auto reset_cycles_run = [](const Menu* menu) {
        commands::config::set_cycles_run(0);
        return nullptr;
    };

    const auto set_cycle_mode = [](const Menu* menu) {
        commands::config::set_cycle_mode(menu->tag);
        return nullptr;
    };

    const auto edit_cycle_count = [this, &display, &display_mutex](const Menu* menu) {
        uint8_t counts[] = {1, 2, 3, 4, 6, 8, 12};
        int count = 0;

        {
            const auto [status, count_] = commands::config::get_cycle_count();
            if (not defines::status::is_ok(status)) {
                display.line1_puts("       ERROR        ");
                display.line2_puts("READING # OF CYCLES ");
                osDelay(3'000);
                return nullptr;
            }
            count = count_;
        }

        bool finished = false;
        while (not finished) {
            display_mutex.acquire(osWaitForever);
            display.clear();
            display.line1_puts("                    ");
            display.line2_printf("         %d         ", counts[count]);
            display_mutex.release();

            const auto flags = _wait_flags(osFlagsWaitAny, osWaitForever);
            auto iterable = _iterate_flags(flags);
            if (iterable.begin() == iterable.end()) {
                continue;
            }

            const auto flag = *iterable.begin();
            switch (flag) {
            case ThreadFlag::MOVE_UP:
                count++;
                break;
            case ThreadFlag::MOVE_DOWN:
                count--;
                break;
            case ThreadFlag::SELECT:
                finished = true;
                break;
            case ThreadFlag::TIMEOUT:
            case ThreadFlag::UNKNOWN:
                break;
            }

            const int min_count = 0;
            const int max_count = static_cast<int>(sizeof(counts)) - 1;
            count = std::max(min_count, std::min(max_count, count));
        }

        {
            const auto status = commands::config::set_cycle_count(count);
            display.line1_puts("  EDIT # OF CYCLES  ");
            if (defines::status::is_ok(status)) {
                display.line2_puts("     SUCCESSFUL     ");
            } else {
                display.line2_puts("       FAILED       ");
            }
            osDelay(3'000);
        }

        return nullptr;
    };

    const auto reset_warning = [](const Menu* menu) {
        using namespace warnings;

        Warning warning = (Warning) (menu->tag);
        if (warning == Warning::Low_Battery) commands::config::update_vbat();
        warnings::reset_warning(warning);
        return nullptr;
    };

    const auto handle_critical_warning = [&]() {
        using namespace warnings;

        Warning active_warning = get_active_warning();

        while (true) {
            display.line1_puts("!!    CRITICAL    !!");

            switch (active_warning) {
                case Warning::Overflow:
                    display.line2_puts("      Overflow      ");
                    break;
                case Warning::Syringe_Homing:
                    display.line2_puts("   Syringe Homing   ");
                    break;
            }

            warning_flash();
            osDelay(500);
            display.line1_puts("!!     UNPLUG     !!");
            osDelay(1'000);
            display.line1_puts("!!    CRITICAL    !!");
            osDelay(1'000);
            display.line1_puts("!!     UNPLUG     !!");
            osDelay(5'000);
        }
    };

    const auto handle_high_warning = [&]() {
        using namespace warnings;

        Warning active_warning = get_active_warning();

        display.line1_puts("!      ALARM       !");

        for (int i = 0; i < 3; i++) {
            switch (active_warning) {
                case Warning::High_Temp:
                    display.line2_puts("  High Temperature  ");
                    break;
                case Warning::Rotary_Homing:
                    display.line2_puts("   Rotary Homing    ");
                    break;
            }

            warning_flash();
            osDelay(2'500);
        }
    };

    const auto handle_battery_warning = [&]() {
        float vbat = commands::config::get_vbat();

        display.line1_puts("!      ALARM       !");

        for (int i = 0; i < 3; i++) {
            display.line2_puts("    Low Battery     ");
            warning_flash();
            osDelay(1'500);
            display.line2_printf("       %.2fV        ", vbat);
            osDelay(2'000);
        }
    };

    {
        const auto [_, configured] = commands::config::get_configured();
        if (not configured) {
            // Set defaults
            commands::config::set_cycles_run(0);
            commands::config::set_num_cycles(0);
            commands::config::set_num_rinses(
                3); // TODO: change 1min rinse time to 1.5min

            // Set dispense time to 1 am (i.e. 60 minutes after midnight)
            commands::config::set_dispense_time(0, 60);

            // Set cycle config to default
            commands::config::set_cycle_mode(0);
            commands::config::set_cycle_count(0);

            // 90-day service cycle
            //commands::config::set_num_service_cycles(90);

            // Reset warnings
            warnings::reset_warnings();

            // Set system time
            wait_for_select("  EDIT SYSTEM TIME  ");
            edit_system_time(nullptr);

            commands::config::set_configured(true);
        }

        // Now that system has started and is configured set alarm
        // For Ecostorm there is only one alarm / dispense time
        // and then from there use the alarm handle to go to the next alarm
        {
            {
                const auto [status, dispense_time]
                    = commands::config::get_dispense_time(0);
                uint8_t hours = dispense_time / 60;
                uint8_t minutes = dispense_time - (hours * 60);
                commands::config::set_timer(hours, minutes);
            }
        }
    }

    // hidden menu
    #include "ui/hidden_code_items.inc"

    const Code* hidden_code_state = &hidden_menu_code[0];

    // clang-format off
    #include "ui/menu_items.inc"

    // clang-format on
    const Menu* menu = main_menu;

    _sleep_timer = xTimerCreate("sleep_timer", SLEEP_TIME, pdFALSE, (void*) &display, _sleeping_timer_callback);
    xTimerStart(_sleep_timer, 0);

    const auto awaken = [&]() {
        menu = main_menu;
        LCD_BL.set();
        awake = true;
    };

    for (;;) {
        display_mutex.acquire(osWaitForever);
        display.clear();
        display.line1_puts(menu->line1);
        display.line2_puts(menu->line2);

        if (menu->parent == &main_menu_errors && menu != &errors_exit) {
            using namespace warnings;

            uint8_t warnings = get_warnings();
            Warning warning = (Warning) (menu->tag);

            if (_is_warning(warnings, warning)) {
                display.line1_puts("     CLEAR NOW      ");
            }
        }

        if (menu == &settings_cycles_run) {
            const auto [_, cycles_run] = commands::config::get_cycles_run();

            if (cycles_run > 0) {
                display.line1_puts("     RESET NOW      ");
            }

            display.line2_printf(" # CYCLES RUN: %d   ", cycles_run);
        }

        if (menu->parent == &hidden_menu_cycle_mode && menu != &hidden_menu_cycle_exit) {
            const auto [_, cycle_mode] = commands::config::get_cycle_mode();

            if (cycle_mode == menu->tag) {
                display.line1_puts("       ACTIVE       ");
            }
        }

        if (menu == &settings_cycle_mode) {
            using namespace commands::config;

            const auto [_, cycle_mode] = get_cycle_mode();

            switch (cycle_mode) {
                case CycleMode::DEFAULT:
                    display.line2_puts("  Default (drains)  ");
                    break;
                case CycleMode::WASTEWATER:
                    display.line2_puts(" Wastewater (mild)  ");
                    break;
                case CycleMode::ANAEROBIC:
                    display.line2_puts(" Anaerobic Digester ");
                    break;
            }
        }

        display_mutex.release();

        const auto flags = _wait_flags(osFlagsWaitAny, osWaitForever);
        auto iterable = _iterate_flags(flags);
        if (iterable.begin() == iterable.end()) {
            continue;
        }

        const auto flag = *iterable.begin();
        
        switch (flag) {
        case ThreadFlag::CRITICAL_WARNING:
            start_holding_awake();
            handle_critical_warning();
            break;
        case ThreadFlag::HIGH_WARNING:
            handle_high_warning();
            break;
        case ThreadFlag::BATTERY_WARNING:
            start_holding_awake();
            handle_battery_warning();
            end_holding_awake();
            break;
        case ThreadFlag::MOVE_UP:
            if (awake && !hold_awake) {
                menu = menu->move_up();
            }
            else {
                awaken();
            }

            if (!hold_awake) xTimerReset(_sleep_timer, 0);

            hidden_code_state = hidden_code_state->advance(ui::Input::UP);

            break;
        case ThreadFlag::MOVE_DOWN:
            if (awake && !hold_awake) {
                menu = menu->move_down();
            }
            else {
                awaken();
            }

            if (!hold_awake) xTimerReset(_sleep_timer, 0);

            hidden_code_state = hidden_code_state->advance(ui::Input::DOWN);
            if (hidden_code_state == nullptr) {
                display.line1_puts("    HIDDEN MENU    ");
                display.line2_puts("                   ");
                osDelay(1000);
                menu = hidden_menu;
                hidden_code_state = &hidden_menu_code[0];
            }

            break;
        case ThreadFlag::SELECT:
            if (awake && !hold_awake) {
                menu = menu->select();
            }
            else {
                awaken();
            }

            if (!hold_awake) xTimerReset(_sleep_timer, 0);

            break;
        case ThreadFlag::TIMEOUT:
        case ThreadFlag::UNKNOWN:
            break;
        }
    }
}

extern "C" void init_ui_task_globals(osThreadId_t handle) { THREAD.set_handle(handle); }

extern "C" void start_ui_task(void* parameters)
{
    auto params_array = static_cast<void**>(parameters);
    auto hi2c = static_cast<I2C_HandleTypeDef*>(params_array[0]);
    auto i2c_mutex_handle = static_cast<osMutexId_t>(params_array[1]);
    auto display_mutex_handle = static_cast<osMutexId_t>(params_array[2]);
    THREAD.target(hi2c, i2c_mutex_handle, display_mutex_handle);
}
