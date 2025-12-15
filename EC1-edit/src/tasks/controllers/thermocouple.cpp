/**
 * @file
 *
 * ADC producer task.
 */

#include <cstring>
#include <tuple>

#include "cmsis_os.h"

#include "main.h"

#include "commands/thermocouple.hpp"
#include "defines/status.hpp"
#include "drivers/gpio.hpp"
#include "drivers/thermocouple.hpp"
#include "helpers/cmsis_os.hpp"

//////////////////
// Thread Flags //
//////////////////

namespace {

#define FOR_ALL_THREAD_FLAGS(FUNC)                                                     \
    FOR_ALL_THERMOCOUPLE_COMMANDS(FUNC)                                                \
    FUNC(ERROR)

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

    void target(SPI_HandleTypeDef* hspi, osMutexId_t spi_mutex_handle);

} THREAD;

} // anonymous namespace

///////////////////////
// Controller Queues //
///////////////////////

namespace commands::thermocouple {

constexpr std::size_t REQUEST_QUEUE_SIZE = 8;

#define DECLARE_QUEUE(COMMAND)                                                         \
    static Queue<COMMAND, REQUEST_QUEUE_SIZE> COMMAND##_QUEUE;
FOR_ALL_THERMOCOUPLE_COMMANDS(DECLARE_QUEUE)
#undef DECLARE_QUEUE

} // namespace commands::thermocouple

namespace commands {

#define DEFINE_QUEUE_PUT(COMMAND)                                                      \
    template <>                                                                        \
    osStatus_t queue_put<thermocouple::COMMAND>(                                       \
        const QueueItem<thermocouple::COMMAND>& item, uint32_t timeout)                \
    {                                                                                  \
        return thermocouple::COMMAND##_QUEUE.put(item, timeout);                       \
    }
FOR_ALL_THERMOCOUPLE_COMMANDS(DEFINE_QUEUE_PUT)
#undef DEFINE_QUEUE_PUT

#define DEFINE_SET_THREAD_FLAG(COMMAND)                                                \
    template <> void set_thread_flag<thermocouple::COMMAND>(void)                      \
    {                                                                                  \
        THREAD.set_flag(ThreadFlag::COMMAND);                                          \
    }
FOR_ALL_THERMOCOUPLE_COMMANDS(DEFINE_SET_THREAD_FLAG)
#undef DEFINE_SET_THREAD_FLAG

} // namespace commands

namespace commands::thermocouple {

bool overheat_trigger;

Status read(int16_t& heater, int16_t& liquid, bool& overheated)
{
    const auto params = std::make_tuple();
    ReturnValue<READ> return_value {};
    const auto send_status = send_and_block<READ>(params, return_value);

    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto [status, heater_reading, liquid_reading] = return_value;

    overheated = overheat_trigger;
    overheat_trigger = false;

    if (!defines::status::is_ok(status)) {
        return status;
    }

    heater = heater_reading;
    liquid = liquid_reading;
    return defines::status::map_from(status);
}

} // namespace commands::thermocouple

///////////////////
// Thread Target //
///////////////////

void Thread::target(SPI_HandleTypeDef* hspi, osMutexId_t spi_mutex_handle)
{
    cmsis_os::Mutex spi_mutex(spi_mutex_handle);

    const SPI_InitTypeDef spi_init = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_8BIT,
        .CLKPolarity = SPI_POLARITY_LOW,
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

    using namespace commands;
    using namespace commands::thermocouple;

    /////////////
    // Helpers //
    /////////////

    const auto is_valid_reading = [](int16_t reading) {
        constexpr auto max_valid_reading = decltype(reading) { 110 };
        constexpr auto min_valid_reading = decltype(reading) { 0 };
        if (reading > max_valid_reading) overheat_trigger = true;
        return (reading <= max_valid_reading) && (reading >= min_valid_reading);
    };

    const auto read_values = [&](int16_t& heater, int16_t& liquid) {
        spi_mutex.acquire(osWaitForever);

        HAL_SPI_DeInit(hspi);
        hspi->Init = spi_init;
        auto hal_status = HAL_SPI_Init(hspi);

        if (not defines::status::is_ok(hal_status)) {
            spi_mutex.release();
            return defines::status::map_from(hal_status);
        }

        auto read_bytes = [hspi](uint8_t* bytes, std::size_t num_bytes) {
            return HAL_SPI_Receive(hspi, bytes, num_bytes, HAL_MAX_DELAY);
        };

        const auto heater_read_status = [&heater, &read_bytes] {
            auto& CS = drivers::gpio::pins::SPI2_NSS3;
            CS.reset();
            const auto status
                = drivers::thermocouple::read_temperature(heater, read_bytes);
            CS.set();
            return status;
        }();

        if (not defines::status::is_ok(heater_read_status)) {
            spi_mutex.release();
            return defines::status::map_from(hal_status);
        }

        if (not is_valid_reading(heater)) {
            spi_mutex.release();
            return Status::ERROR;
        }

        const auto liquid_read_status = [&liquid, &read_bytes] {
            // Use this chip-select for the Mexico demo unit.
            // auto& CS = drivers::gpio::pins::SPI2_NSS1;

            // Use this CS for the normal, unhacked board.
            auto& CS = drivers::gpio::pins::SPI2_NSS4;

            CS.reset();
            const auto status
                = drivers::thermocouple::read_temperature(liquid, read_bytes);
            CS.set();
            return status;
        }();

        if (not defines::status::is_ok(liquid_read_status)) {
            spi_mutex.release();
            return defines::status::map_from(hal_status);
        }

        if (not is_valid_reading(liquid)) {
            spi_mutex.release();
            return Status::ERROR;
        }

        spi_mutex.release();
        return Status::OK;
    };

    while (true) {
        _wait_flags(osFlagsWaitAny, osWaitForever);

        int16_t heater;
        int16_t liquid;
        const auto status = read_values(heater, liquid);

        const auto return_value = std::make_tuple(status, heater, liquid);
        flush_queue<READ>(READ_QUEUE, return_value);
    }
}

extern "C" void init_temp_monitor_task_globals(osThreadId_t handle)
{
    THREAD.set_handle(handle);
#define INIT_QUEUE(COMMAND) commands::thermocouple::COMMAND##_QUEUE.init();
    FOR_ALL_THERMOCOUPLE_COMMANDS(INIT_QUEUE)
#undef INIT_QUEUE
}

/**
 * Start ADC monitor task.
 */
extern "C" void start_temp_monitor_task(void* parameters)
{
    auto params_array = static_cast<void**>(parameters);
    auto hspi = static_cast<SPI_HandleTypeDef*>(params_array[0]);
    auto spi_mutex_handle = static_cast<osMutexId_t>(params_array[1]);
    THREAD.target(hspi, spi_mutex_handle);
}
