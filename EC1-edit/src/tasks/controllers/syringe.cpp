/**
 * @file
 *
 * Define the syringe control task.
 */

#include <functional>

#include "cmsis_os.h"

#include "main.h"

#include "timers.h"

#include "commands/syringe.hpp"
#include "defines/status.hpp"
#include "drivers/gpio.hpp"
#include "drivers/stepper.hpp"

using Status = defines::Status;

/////////////////////////////
// Main thread declaration //
/////////////////////////////

namespace {

#define FOR_ALL_THREAD_FLAGS(FUNC)                                                     \
    FUNC(WAIT_TIMEOUT)                                                                 \
    FOR_ALL_SYRINGE_COMMANDS(FUNC)

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

    void target(
        SPI_HandleTypeDef* hspi, osMutexId_t spi_mutex_handle, TIM_HandleTypeDef* htim);
} THREAD;

}

///////////////////////
// Controller Queues //
///////////////////////

namespace commands::syringe {

constexpr std::size_t REQUEST_QUEUE_SIZE = 8;

#define DECLARE_QUEUE(COMMAND)                                                         \
    static Queue<COMMAND, REQUEST_QUEUE_SIZE> COMMAND##_QUEUE;
FOR_ALL_SYRINGE_COMMANDS(DECLARE_QUEUE)
#undef DECLARE_QUEUE

} // namespace commands::syringe

namespace commands {

#define DEFINE_QUEUE_PUT(COMMAND)                                                      \
    template <>                                                                        \
    osStatus_t queue_put<syringe::COMMAND>(                                            \
        const QueueItem<syringe::COMMAND>& item, uint32_t timeout)                     \
    {                                                                                  \
        return syringe::COMMAND##_QUEUE.put(item, timeout);                            \
    }
FOR_ALL_SYRINGE_COMMANDS(DEFINE_QUEUE_PUT)
#undef DEFINE_QUEUE_PUT

#define DEFINE_SET_THREAD_FLAG(COMMAND)                                                \
    template <> void set_thread_flag<syringe::COMMAND>(void)                           \
    {                                                                                  \
        THREAD.set_flag(ThreadFlag::COMMAND);                                          \
    }
FOR_ALL_SYRINGE_COMMANDS(DEFINE_SET_THREAD_FLAG)
#undef DEFINE_SET_THREAD_FLAG

} // namespace commands

namespace commands::syringe {

Status rehome(void)
{
    const auto params = std::make_tuple();
    ReturnValue<REHOME> return_value;
    const auto send_status = send_and_block<REHOME>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status push(uint32_t steps)
{
    const auto params = std::make_tuple(steps);
    ReturnValue<PUSH> return_value;
    const auto send_status = send_and_block<PUSH>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status pull(uint32_t steps)
{
    const auto params = std::make_tuple(steps);
    ReturnValue<PULL> return_value;
    const auto send_status = send_and_block<PULL>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

Status fill(void)
{
    const auto params = std::make_tuple();
    ReturnValue<PULL> return_value;
    const auto send_status = send_and_block<FILL>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [command_status] = return_value;
    return defines::status::map_from(command_status);
}

} // namespace commands::syringe

void _rehome_fail_callback(TimerHandle_t timer) {
    bool* failed = static_cast<bool*>(pvTimerGetTimerID(timer));
    *failed = true;
}

///////////////////
// Thread Target //
///////////////////

void Thread::target(
    SPI_HandleTypeDef* hspi, osMutexId_t spi_mutex_handle, TIM_HandleTypeDef* htim)
{
    cmsis_os::Mutex spi_mutex(spi_mutex_handle);

    const auto init_stepper = [&](void) {
        using namespace drivers::stepper;

        spi_mutex.acquire(osWaitForever);

        HAL_SPI_DeInit(hspi);

        hspi->Init = {
            .Mode = SPI_MODE_MASTER,
            .Direction = SPI_DIRECTION_2LINES,
            .DataSize = SPI_DATASIZE_8BIT,
            .CLKPolarity = SPI_POLARITY_HIGH,
            .CLKPhase = SPI_PHASE_2EDGE,
            .NSS = SPI_NSS_SOFT,
            .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16,
            .FirstBit = SPI_FIRSTBIT_MSB,
            .TIMode = SPI_TIMODE_DISABLE,
            .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
            .CRCPolynomial = 0,
            .CRCLength = 0,
            .NSSPMode = SPI_NSS_PULSE_ENABLE,
        };

        HAL_SPI_Init(hspi);

        auto& CS = drivers::gpio::pins::SPI2_NSS5;

        CS.set();
        osDelay(100);

        auto read_bytes = [hspi](uint8_t* bytes, std::size_t num_bytes) {
            return HAL_SPI_Receive(hspi, bytes, num_bytes, HAL_MAX_DELAY);
        };

        auto write_bytes = [hspi](uint8_t* bytes, std::size_t num_bytes) {
            return HAL_SPI_Transmit(hspi, bytes, num_bytes, HAL_MAX_DELAY);
        };

        registers::SHORT_CONF short_conf = {
            .S2VS_LEVEL = 15,
            ._unused0 = 0,
            .S2G_LEVEL = 15,
            ._unused1 = 0,
            .SHORTFILTER = 3,
            .shortdelay = 0,
            ._unused2 = 0,
        };

        CS.reset();
        write_register(short_conf, write_bytes);
        CS.set();

        registers::GSTAT gstat = {
            .reset = true,
            .drv_err = true,
            .uv_cp = true,
            ._unused0 = 0,
            ._unused1 = 0,
        };

        CS.reset();
        write_register(gstat, write_bytes);
        CS.set();

        CS.reset();
        request_read_register<registers::IOIN>(write_bytes);
        CS.set();

        CS.reset();
        drivers::stepper::Status status = {};
        registers::IOIN ioin = {};
        read_register(status, ioin, read_bytes);
        CS.set();

        CS.reset();
        request_read_register<registers::DRV_STATUS>(write_bytes);
        CS.set();

        CS.reset();
        registers::DRV_STATUS drv_status = {};
        read_register(status, drv_status, read_bytes);
        CS.set();

        registers::CHOPCONF chopconf = {
            .toff = 3,
            .hstrt = 0b100,
            .hend = 0b0001,
            .fd3 = 0,
            .disfdcc = 0,
            ._reserved0 = 0,
            .chm = false,
            .tbl = 0b10,
            ._reserved1 = 0,
            .vhighfs = 0,
            .vhighchm = 0,
            .tpfd = 0,
            .mres = 0b0110,
            .intpol = 0,
            .dedge = 0,
            .diss2g = 0,
            .diss2vs = 0,
        };

        CS.reset();
        write_register(chopconf, write_bytes);
        CS.set();


        // Ihold was 10, run = 31, changed to new values for power - JDL (11/1/2023)
        registers::IHOLD_IRUN ihold_irun = {
            .IHOLD = 5,
            ._unused0 = 0,
            .IRUN = 15,
            ._unused1 = 0,
            .IHOLDDELAY = 6,
            ._unused2 = 0,
        };

        CS.reset();
        write_register(ihold_irun, write_bytes);
        CS.set();

        registers::TPOWERDOWN tpowerdown = {
            .TPOWERDOWN = 10,
            ._unused = 0,
        };

        CS.reset();
        write_register(tpowerdown, write_bytes);
        CS.set();

        registers::GCONF gconf = {
            .recalibrate = 0,
            .fastandstill = 0,
            .en_pwm_mode = true,
            .multistep_filt = 0,
            .shaft = 0,
            .diag0_error = 0,
            .diag0_otpw = 0,
            .diag0_stall = 0,
            .diag1_stall = 0,
            .diag1_index = 0,
            .diag1_onstate = 0,
            .diag1_steps_skipped = 0,
            .diag0_int_pushpull = 0,
            .diag1_pushpull = 0,
            .small_hysteresis = 0,
            .stop_enable = 0,
            .direct_mode = 0,
            .test_mode = 0,
            ._unused0 = 0,
            ._unused1 = 0,
        };

        CS.reset();
        write_register(gconf, write_bytes);
        CS.set();

        registers::TPWMTHRS tpwmthrs = {
            .TPWMTHRS = 0,
            ._unused = 0,
        };

        CS.reset();
        write_register(tpwmthrs, write_bytes);
        CS.set();

        spi_mutex.release();
    };

    htim->PeriodElapsedCallback = [](TIM_HandleTypeDef* htim) {
        THREAD.set_flag(ThreadFlag::WAIT_TIMEOUT);
        HAL_TIM_Base_Stop_IT(htim);
    };

    const auto pulse_delay = [&](void) {
        HAL_TIM_Base_Start_IT(htim);
        _wait_flag(ThreadFlag::WAIT_TIMEOUT, osWaitForever);
    };

    const auto home_is_activated
        = [](void) { return drivers::gpio::pins::STPR_HOME.read() == GPIO_PIN_RESET; };

    const auto set_direction_pull = [](void) { drivers::gpio::pins::STPR_DIR.reset(); };

    const auto set_direction_push = [](void) { drivers::gpio::pins::STPR_DIR.set(); };

    const auto step_stepper = [&](void) {
        drivers::gpio::pins::STPR_STEP.reset();
        pulse_delay();
        drivers::gpio::pins::STPR_STEP.set();
        pulse_delay();
    };

    const auto enable = [](void) { drivers::gpio::pins::STPR_EN.reset(); };
    const auto disable = [](void) { drivers::gpio::pins::STPR_EN.set(); };

    int32_t step_count = 0;

    // Calibrated by pulling syringe until stepper slips and taking 75% of steps.
    // John: Changed to 95000 of steps (by Sherman).
    // John: Changed to 75% of steps (by John).
    constexpr int32_t max_step_count = 65'000;

    // Measured about 75 mL of liquid with 25,000 syringe steps.
    // John: Changed to 324 steps / ml (by Sherman)
    // Matt/Mike: CHanges to 275, reduced by 15% from 324 to 275 per Charles Greenwald 12/19/2024
    // Hunter: Increase to 350 (6/6/2025)
    constexpr uint32_t steps_per_ml = 350;

    const auto ml_to_steps
        = [](uint32_t ml) { return static_cast<uint32_t>(steps_per_ml * ml); };

    const auto rehome = [&](void) {
        bool failed = false;
        const TickType_t PUSH_FAIL_TIME = 4 * 60 * 1000;
        const TickType_t PULL_FAIL_TIME = 3 * 60 * 1000;
        TimerHandle_t _rehome_fail_timer = xTimerCreate("syringe_rehome_fail_timer", PUSH_FAIL_TIME, pdFALSE, (void*) &failed, _rehome_fail_callback);
        xTimerStart(_rehome_fail_timer, 0);

        enable();
        if (not home_is_activated()) {
            set_direction_push();
            while (not home_is_activated()) {
                step_stepper();
                if (failed) {
                    xTimerStop(_rehome_fail_timer, 0);
                    xTimerDelete(_rehome_fail_timer, 0);
                    disable();
                    return Status::ERROR;
                }
            }
            step_stepper();
        }

        xTimerReset(_rehome_fail_timer, 0);
        xTimerChangePeriod(_rehome_fail_timer, PULL_FAIL_TIME, 0);
        set_direction_pull();
        while (home_is_activated()) {
            step_stepper();
            if (failed) {
                xTimerStop(_rehome_fail_timer, 0);
                xTimerDelete(_rehome_fail_timer, 0);
                disable();
                return Status::ERROR;
            }
        }

        step_count = 0;
        disable();

        xTimerStop(_rehome_fail_timer, 0);
        xTimerDelete(_rehome_fail_timer, 0);
        return Status::OK;
    };

    const auto push = [&](uint32_t ml) {
        const uint32_t steps = ml_to_steps(ml);
        enable();
        set_direction_push();
        pulse_delay();
        for (auto index = decltype(steps) { 0 }; index < steps; index++) {
            if (not home_is_activated()) {
                step_stepper();
                step_count--;
            } else {
                step_count = 0;
                break;
            }
        }
        disable();
        return Status::OK;
    };

    const auto pull = [&](uint32_t ml) {
        const uint32_t steps = ml_to_steps(ml);
        enable();
        set_direction_pull();
        pulse_delay();
        for (auto index = decltype(steps) { 0 }; index < steps; index++) {
            if (step_count < max_step_count) {
                step_stepper();
                step_count++;
            } else {
                step_count = max_step_count;
                break;
            }
        }
        disable();
        return Status::OK;
    };

    using namespace commands;
    using namespace commands::syringe;

    const auto handle_rehome = [&](Signature<REHOME>::Parameters params) {
        const auto status = rehome();
        return Signature<REHOME>::ReturnValue { status };
    };

    /**
     * Push the syringe stepper a given number of milliliters.
     */
    const auto handle_push = [&](Signature<PUSH>::Parameters params) {
        const auto [steps] = params;
        const auto status = push(steps);
        return Signature<PUSH>::ReturnValue { status };
    };

    /**
     * Pull the syringe stepper a given number of milliliters.
     */
    const auto handle_pull = [&](Signature<PULL>::Parameters params) {
        const auto [steps] = params;
        const auto status = pull(steps);
        return Signature<PULL>::ReturnValue { status };
    };

    /**
     * Completely fill the syringe stepper.
     */
    const auto handle_fill = [&](Signature<FILL>::Parameters params) {
        const auto steps = max_step_count - step_count;
        const auto status = pull(steps);
        return Signature<FILL>::ReturnValue { status };
    };

    disable();
    init_stepper();

    while (true) {
        const uint32_t flags = _wait_flags(osFlagsWaitAny, osWaitForever);
        for (const auto& flag : _iterate_flags(flags)) {
            switch (flag) {
            case ThreadFlag::PUSH: {
                handle_queue<PUSH>(PUSH_QUEUE, handle_push);
                break;
            }
            case ThreadFlag::PULL: {
                handle_queue<PULL>(PULL_QUEUE, handle_pull);
                break;
            }
            case ThreadFlag::FILL: {
                handle_queue<FILL>(FILL_QUEUE, handle_fill);
                break;
            }
            case ThreadFlag::REHOME: {
                handle_queue<REHOME>(REHOME_QUEUE, handle_rehome);
                break;
            }
            case ThreadFlag::WAIT_TIMEOUT: {
                break;
            }
            case ThreadFlag::UNKNOWN: {
                break;
            }
            }
        }
    }
}

extern "C" void init_syringe_control_task_globals(osThreadId_t handle)
{
    THREAD.set_handle(handle);
#define INIT_QUEUE(COMMAND) commands::syringe::COMMAND##_QUEUE.init();
    FOR_ALL_SYRINGE_COMMANDS(INIT_QUEUE)
#undef INIT_QUEUE
}

/**
 * Control the syringe based on given commands.
 */
extern "C" void start_syringe_control_task(void* parameters)
{
    auto params_array = static_cast<void**>(parameters);
    auto hspi = static_cast<SPI_HandleTypeDef*>(params_array[0]);
    auto spi_mutex_handle = static_cast<osMutexId_t>(params_array[1]);
    auto htim = static_cast<TIM_HandleTypeDef*>(params_array[2]);
    THREAD.target(hspi, spi_mutex_handle, htim);
}
