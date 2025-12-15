/**
 * @file
 *
 * Define the valve control task.
 */

#include <algorithm>
#include <array>
#include <bitset>
#include <numeric>
#include <type_traits>

#include "cmsis_os.h"

#include "main.h"

#include "commands/valve.hpp"
#include "drivers/dac.hpp"
#include "drivers/gpio.hpp"
#include "helpers/button_filter.hpp"

#include "drivers/display.hpp"


using Status = defines::Status;

////////////////////////
// Thread Declaration //
////////////////////////

namespace {

#define FOR_ALL_THREAD_FLAGS(FUNC)                                                     \
    FOR_ALL_VALVE_COMMANDS(FUNC)                                                       \
    FUNC(VALVE_HOME_RISING)                                                            \
    FUNC(VALVE_HOME_FALLING)

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

    void target(DAC_HandleTypeDef* hdac, TIM_HandleTypeDef* htim);

} THREAD;

}

//////////////////////
// Controller Queue //
//////////////////////

namespace commands::valve {

constexpr std::size_t REQUEST_QUEUE_SIZE = 8;

#define DECLARE_QUEUE(COMMAND)                                                         \
    static Queue<COMMAND, REQUEST_QUEUE_SIZE> COMMAND##_QUEUE;
FOR_ALL_VALVE_COMMANDS(DECLARE_QUEUE)
#undef DECLARE_QUEUE

} // namespace commands::valve

namespace commands {

#define DEFINE_QUEUE_PUT(COMMAND)                                                      \
    template <>                                                                        \
    osStatus_t queue_put<valve::COMMAND>(                                              \
        const QueueItem<valve::COMMAND>& item, uint32_t timeout)                       \
    {                                                                                  \
        return valve::COMMAND##_QUEUE.put(item, timeout);                              \
    }
FOR_ALL_VALVE_COMMANDS(DEFINE_QUEUE_PUT)
#undef DEFINE_QUEUE_PUT

#define DEFINE_SET_THREAD_FLAG(COMMAND)                                                \
    template <> void set_thread_flag<valve::COMMAND>(void)                             \
    {                                                                                  \
        THREAD.set_flag(ThreadFlag::COMMAND);                                          \
    }
FOR_ALL_VALVE_COMMANDS(DEFINE_SET_THREAD_FLAG)
#undef DEFINE_SET_THREAD_FLAG

} // namespace commands

namespace commands::valve {

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

Status move_to(Position position)
{
    const auto params = std::make_tuple(position);
    ReturnValue<MOVE_TO> return_value {};
    const auto send_status = send_and_block<MOVE_TO>(params, return_value);
    const auto [command_status] = return_value;

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    return defines::status::map_from(command_status);
}

} // namespace commands::valve

void _valve_rehome_fail_callback(TimerHandle_t timer) {
    bool* failed = static_cast<bool*>(pvTimerGetTimerID(timer));
    *failed = true;
}

///////////////////
// Thread Target //
///////////////////

const int NUM_AVG { 35 };
const float DVDT_LIM_NEG { -30.0 };
const float DVDT_LIM_POS { 30.0 };

extern ADC_HandleTypeDef hadc1;
float halleffect_min = 65535.0;
float halleffect_max = 0.0;
float falling_edge_value = 0.0;
float rising_edge_value = 65535.0;

void Thread::target(DAC_HandleTypeDef* hdac, TIM_HandleTypeDef* htim)
{
    /////////////
    // Helpers //
    /////////////

    using Position = commands::valve::Position;

//    const auto positions = {
//        Position::WATER_IN,
//        Position::HOME,
//        Position::SPORES,
//        Position::SUPERCHARGER,
//        Position::WATER_OUT,
//    };

    const auto positions = {
        Position::WATER_IN,
        Position::WATER_OUT,
        Position::SUPERCHARGER,
        Position::SPORES,
        Position::HOME,
    };

    auto min_distance_between = [&](Position from, Position to) {
        auto lhs = std::find(std::cbegin(positions), std::cend(positions), from);
        auto rhs = std::find(std::cbegin(positions), std::cend(positions), to);
        const auto difference = std::distance(lhs, rhs);

        using difference_type = std::remove_cv_t<decltype(difference)>;
        const auto size = static_cast<difference_type>(positions.size());

        return std::min({ difference, difference - size, difference + size },
            [](auto lhs, auto rhs) { return std::abs(lhs) < std::abs(rhs); });
    };

    auto move_forward = [](void) {
        drivers::gpio::pins::VLV_MTR_2.set();
        drivers::gpio::pins::VLV_MTR_1.reset();
    };

    auto move_backward = [](void) {
        drivers::gpio::pins::VLV_MTR_2.reset();
        drivers::gpio::pins::VLV_MTR_1.set();
    };

    auto turn_on_brakes = [](void) {
        drivers::gpio::pins::VLV_MTR_2.set();
        drivers::gpio::pins::VLV_MTR_1.set();
    };

    auto turn_off = [](void) {
        drivers::gpio::pins::VLV_MTR_2.reset();
        drivers::gpio::pins::VLV_MTR_1.reset();
    };

    auto stop = [&](uint32_t delay = 200) {
        turn_on_brakes();
        osDelay(delay);
        turn_off();
    };

    auto stop_on_flag = [&](ThreadFlag flag) {
        _wait_flag(flag, osWaitForever);
        stop();
        _clear_flags();
    };


    // This next section is a bit of a hack to fit in the analog hall effect into this system
    
    // Get average of ADC signal
    auto get_adc_average = [&](void) {
        ADC_ChannelConfTypeDef config = {};
        config.Channel = ADC_CHANNEL_2;
        config.Rank = ADC_REGULAR_RANK_1;
        config.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
        HAL_ADC_ConfigChannel(&hadc1, &config);

        float summation { 0.0 };
        for (int index = 0; index < NUM_AVG; index++)
        {
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 1);
            summation += (float)HAL_ADC_GetValue(&hadc1);
            osDelay(1);
        }
        return summation / (float)NUM_AVG;
    };

    // move valve forward and get max/min values
    auto find_halleffect_min_max = [&](void) {
        move_forward();
        for (int index = 0; index < 400; index++)
        {
            float hall_voltage_adc = get_adc_average();

            // Get min and max values
            if (hall_voltage_adc < halleffect_min) halleffect_min = hall_voltage_adc;
            if (hall_voltage_adc > halleffect_max) halleffect_max = hall_voltage_adc;
        }
        stop();

        float span = halleffect_max - halleffect_min;
        if (span > 10.0)
        {
            falling_edge_value = halleffect_min + (0.35 * span);
            rising_edge_value = halleffect_min + (0.70 * span);
        }
    };



    auto move_forward_until_rising_switch_edge = [&](TimerHandle_t timer, bool* failed) {
        if (*failed) return (uint32_t) 0;

        const auto start = osKernelGetTickCount();
        move_forward();
        // Changed to analog interface - JDL (11/1/2023)
        // stop_on_flag(ThreadFlag::VALVE_HOME_RISING);
        // Wait until next location
        while (!(*failed) && get_adc_average() < rising_edge_value);
  
        // Spin until we get flat area (low DVDT on neg side)
        bool at_location { false };
        float previous_value = get_adc_average();
        while (!(*failed) && at_location == false)
        {
            osDelay(1);
            float currentPos = get_adc_average();
            float dVdt = currentPos - previous_value;
            if ((currentPos < falling_edge_value) &
                (dVdt > DVDT_LIM_POS)) at_location = true;
         
            // Reset to new position, delay
            previous_value = currentPos;
        }
        if (timer != NULL) xTimerReset(timer, 0);
        stop();
        return osKernelGetTickCount() - start;
    };

    auto move_forward_until_falling_switch_edge = [&](void) {
        const auto start = osKernelGetTickCount();
        move_forward();
        // Changed to analog interface - JDL (11/1/2023)
        // stop_on_flag(ThreadFlag::VALVE_HOME_RISING);
        
        // Spin until we get flat area (low DVDT on neg side)
        bool at_location { false };
        float previous_value = get_adc_average();
        while (at_location == false)
        {
            osDelay(1);
            float currentPos = get_adc_average();
            float dVdt = currentPos - previous_value;
            if ((currentPos < falling_edge_value) &
                (dVdt < DVDT_LIM_NEG)) at_location = true;
         
            // Reset to new position, delay
            previous_value = currentPos;
        }
        stop();
        return osKernelGetTickCount() - start;
    };

    auto move_backward_until_rising_switch_edge = [&](void) {
        const auto start = osKernelGetTickCount();
        move_backward();
        // Changed to analog interface - JDL (11/1/2023)
        // stop_on_flag(ThreadFlag::VALVE_HOME_FALLING);
        // Wait until next location
        while (get_adc_average() < rising_edge_value);

        // Spin until we get flat area (low DVDT on pos side)
        bool at_location { false };
        float previous_value = get_adc_average();
        while (at_location == false)
        {
            osDelay(1);
            float currentPos = get_adc_average();
            float dVdt = currentPos - previous_value;
            if ((currentPos < falling_edge_value) &
                (dVdt > DVDT_LIM_POS)) at_location = true;
         
            // Reset to new position, delay
            previous_value = currentPos;
        }
        stop();
        return osKernelGetTickCount() - start;
    };

    auto move_backward_until_falling_switch_edge = [&](void) {
        const auto start = osKernelGetTickCount();
        move_backward();
        // Changed to analog interface - JDL (11/1/2023)
        // stop_on_flag(ThreadFlag::VALVE_HOME_FALLING);

        // Spin until we get flat area (low DVDT on pos side)
        bool at_location { false };
        float previous_value = get_adc_average();
        while (at_location == false)
        {
            osDelay(1);
            float currentPos = get_adc_average();
            float dVdt = currentPos - previous_value;
            if ((currentPos < falling_edge_value) &
                (dVdt < DVDT_LIM_NEG)) at_location = true;
         
            // Reset to new position, delay
            previous_value = currentPos;
        }
        stop();

        return osKernelGetTickCount() - start;
    };
// Removed to improve accuracy - JDL (11/1/2023)
/*
    auto center_hole_forward = [&](void) {
        const uint32_t ticks = move_forward_until_falling_switch_edge();
        move_backward_until_rising_switch_edge();
        move_backward_until_falling_switch_edge();
        move_forward_until_rising_switch_edge();
        move_forward();
        osDelay((ticks + 1) / 2);
        stop();
    };

    auto center_hole_backward = [&](void) {
        const uint32_t ticks = move_backward_until_falling_switch_edge();
        move_forward_until_rising_switch_edge();
        move_forward_until_falling_switch_edge();
        move_backward_until_rising_switch_edge();

        move_backward();
        osDelay((ticks + 1) / 2);
        stop();
    };
*/
    auto rehome = [&](Position& position) {
        // Find Min/Max voltage values
        find_halleffect_min_max();

        const uint32_t FAIL_TIME = 5 * 60 * 1000;
        bool failed = false;

        TimerHandle_t _fail_timer = xTimerCreate("valve_rehome_fail_timer", FAIL_TIME, pdFALSE, (void*) &failed, _valve_rehome_fail_callback);
        xTimerStart(_fail_timer, 0);
        
        // Go to first edge
        move_forward_until_rising_switch_edge(_fail_timer, &failed);
        
        // Get times between edges (use to find home position, and valve orientation)
        const std::array times = {
            move_forward_until_rising_switch_edge(_fail_timer, &failed),
            move_forward_until_rising_switch_edge(_fail_timer, &failed),
            move_forward_until_rising_switch_edge(_fail_timer, &failed),
            move_forward_until_rising_switch_edge(_fail_timer, &failed),
            move_forward_until_rising_switch_edge(_fail_timer, &failed),
        };

        xTimerStop(_fail_timer, 0);
        xTimerDelete(_fail_timer, 0);

        if (failed) return Status::ERROR;

        const uint32_t sum = std::accumulate(times.cbegin(), times.cend(), 0);
        const uint32_t average = sum / times.size();


        std::bitset<times.size()> residuals;
        for (std::size_t index = 0; index < times.size(); index++) {
            residuals[index] = times[index] < average;
        }

 /*       switch (residuals.to_ulong()) {
        case 0b11000:
            position = Position::SPORES;
            break;
        case 0b01100:
            position = Position::SUPERCHARGER;
            break;
        case 0b00110:
            position = Position::WATER_OUT;
            break;
        case 0b00011:
            position = Position::WATER_IN;
            break;
        case 0b10001:
            position = Position::HOME;
            break;
        default:
            return Status::ERROR;
        }
*/        
        switch (residuals.to_ulong()) {
        case 0b11000:
            position = Position::WATER_IN;
            break;
        case 0b01100:
            position = Position::WATER_OUT;
            break;
        case 0b00110:
            position = Position::SUPERCHARGER;
            break;
        case 0b00011:
            position = Position::SPORES;
            break;
        case 0b10001:
            position = Position::HOME;
            break;
        default:
            return Status::ERROR;
        }
        
        /*center_hole_forward();
        */
        return Status::OK;
    };

    auto move_between = [&](Position from, Position to) {
        if (from == to) {
            return;
        }
// Previous code to center adn run backwards - removed to more accurate position - JDL
// 11/1/2023
/*        const auto distance = min_distance_between(from, to);
        const bool forward = distance >= 0;
        const auto steps = std::abs(distance);

        if (forward) {
            for (auto index = decltype(steps) { 0 }; index < steps; index++) {
                move_forward_until_rising_switch_edge();
            }
            center_hole_forward();
        } else {
            for (auto index = decltype(steps) { 0 }; index < steps; index++) {
                move_backward_until_rising_switch_edge();
            }
            center_hole_backward();
        }
*/

        bool failed = false;

        const auto distance = min_distance_between(from, to);
        auto steps = std::abs(distance);

        if (distance < 0) steps = 5 - steps;
        for (auto index = decltype(steps) { 0 }; index < steps; index++) {
            move_forward_until_rising_switch_edge(NULL, &failed);
        }

    };

    /////////////////////////////
    // Reference level setting //
    /////////////////////////////

    drivers::DAConverter dac(hdac);
    const auto max_reference_level = (uint16_t { 1 } << 12) - 1;
    dac.set_value_ch1(max_reference_level, DAC_ALIGN_12B_R);
    dac.start_ch1();

    //////////////////////////////////////
    // Home Switch Sampling / Filtering //
    //////////////////////////////////////

    htim->PeriodElapsedCallback = [](TIM_HandleTypeDef* htim) {
        constexpr std::size_t filter_size = 64;
        constexpr std::size_t margin = 8;
        constexpr std::size_t low_threshold = margin;
        constexpr std::size_t high_threshold = filter_size - margin;

        static bool logic_state = false;
        static std::size_t index = 0;
        static std::array<bool, filter_size> samples = { false };

        auto home_switch_activated = [](void) {
            return drivers::gpio::pins::VLV_HOME.read() == GPIO_PIN_RESET;
        };

        samples[index] = home_switch_activated();
        index = (index + 1) % filter_size;

        const uint32_t sum = std::accumulate(samples.cbegin(), samples.cend(), 0);

        if (logic_state == false and sum >= high_threshold) {
            logic_state = true;
            THREAD.set_flag(ThreadFlag::VALVE_HOME_RISING);
        }

        if (logic_state == true and sum <= low_threshold) {
            logic_state = false;
            THREAD.set_flag(ThreadFlag::VALVE_HOME_FALLING);
        }
    };

    HAL_TIM_Base_Start_IT(htim);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);

    using namespace commands;
    using namespace commands::valve;

    auto current_position = Position::HOME;

    auto handle_rehome = [&](Parameters<REHOME>& params) {
        const auto command_status = rehome(current_position);
        return std::make_tuple(command_status);
    };

    auto handle_move_to = [&](Parameters<MOVE_TO>& params) {
        const auto [position] = params;
        move_between(current_position, position);
        current_position = position;
        return std::make_tuple(Status::OK);
    };

    while (true) {
        const uint32_t flags = _wait_flags(osFlagsWaitAny, osWaitForever);
        for (const auto& flag : _iterate_flags(flags)) {
            switch (flag) {
            case ThreadFlag::REHOME: {
                handle_queue<REHOME>(REHOME_QUEUE, handle_rehome);
                break;
            }
            case ThreadFlag::MOVE_TO: {
                handle_queue<MOVE_TO>(MOVE_TO_QUEUE, handle_move_to);
                break;
            }
            case ThreadFlag::VALVE_HOME_RISING: {
                break;
            }
            case ThreadFlag::VALVE_HOME_FALLING: {
                break;
            }
            case ThreadFlag::UNKNOWN: {
                break;
            }
            }
        }
    }
}

extern "C" void init_valve_control_task_globals(osThreadId_t handle)
{
    THREAD.set_handle(handle);
#define INIT_QUEUE(COMMAND) commands::valve::COMMAND##_QUEUE.init();
    FOR_ALL_VALVE_COMMANDS(INIT_QUEUE)
#undef INIT_QUEUE
}

/**
 * Control the valve based on given commands.
 */
extern "C" void start_valve_control_task(void* parameters)
{
    auto params_array = static_cast<void**>(parameters);
    auto hdac = static_cast<DAC_HandleTypeDef*>(params_array[0]);
    auto htim = static_cast<TIM_HandleTypeDef*>(params_array[1]);
    THREAD.target(hdac, htim);
}
