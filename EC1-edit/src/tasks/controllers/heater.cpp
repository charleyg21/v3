/**
 * @file
 *
 * Define the heater control task.
 */

#include "cmsis_os.h"

#include "main.h"

#include "commands/heater.hpp"
#include "commands/thermocouple.hpp"
#include "defines/status.hpp"
#include "drivers/gpio.hpp"

using Status = defines::Status;

namespace {

#define FOR_ALL_THREAD_FLAGS(FUNC) FOR_ALL_HEATER_COMMANDS(FUNC)

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

    void target(void);
} THREAD;

}

///////////////////////
// Controller Queues //
///////////////////////

namespace commands::heater {

constexpr std::size_t REQUEST_QUEUE_SIZE = 1;

#define DECLARE_QUEUE(COMMAND)                                                         \
    static Queue<COMMAND, REQUEST_QUEUE_SIZE> COMMAND##_QUEUE;
FOR_ALL_HEATER_COMMANDS(DECLARE_QUEUE)
#undef DECLARE_QUEUE

} // namespace commands::heater

namespace commands {

#define DEFINE_QUEUE_PUT(COMMAND)                                                      \
    template <>                                                                        \
    osStatus_t queue_put<heater::COMMAND>(                                             \
        const QueueItem<heater::COMMAND>& item, uint32_t timeout)                      \
    {                                                                                  \
        return heater::COMMAND##_QUEUE.put(item, timeout);                             \
    }
FOR_ALL_HEATER_COMMANDS(DEFINE_QUEUE_PUT)
#undef DEFINE_QUEUE_PUT

#define DEFINE_SET_THREAD_FLAG(COMMAND)                                                \
    template <> void set_thread_flag<heater::COMMAND>(void)                            \
    {                                                                                  \
        THREAD.set_flag(ThreadFlag::COMMAND);                                          \
    }
FOR_ALL_HEATER_COMMANDS(DEFINE_SET_THREAD_FLAG)
#undef DEFINE_SET_THREAD_FLAG

} // namespace commands

namespace commands::heater {

Status turn_on(int16_t target)
{
    const auto params = std::make_tuple(target);
    ReturnValue<TURN_ON> return_value {};
    const auto send_status = send_and_block<TURN_ON>(params, return_value);
    return defines::status::map_from(send_status);
}

Status turn_off(void)
{
    const auto params = std::make_tuple();
    ReturnValue<TURN_OFF> return_value {};
    const auto send_status = send_and_block<TURN_OFF>(params, return_value);
    return defines::status::map_from(send_status);
}
Status status(bool& heater_state)
{
    const auto params = std::make_tuple();
    ReturnValue<STATUS> return_value {};
    const auto send_status = send_and_block<STATUS>(params, return_value);

    if (!defines::status::is_ok(send_status)) {
        return send_status;
    }

    const auto [heater_on] = return_value;
    heater_state = heater_on;

    return defines::status::map_from(send_status);
}

} // namespace commands::heater

///////////////////
// Thread Target //
///////////////////

void Thread::target(void)
{
    using namespace commands;
    using namespace commands::heater;

    constexpr uint32_t loop_delay = 1'000;

    const auto turn_on = [](void) { drivers::gpio::pins::HTR_EN.set(); };
    const auto turn_off = [](void) { drivers::gpio::pins::HTR_EN.reset(); };

    bool heater_on = false;

    // Change to 36 for now, should set via parameters from calling function.
    int16_t target_liquid = 36;

    const auto handle_turn_on = [&](Signature<TURN_ON>::Parameters params) {
        const auto [target] = params;
        target_liquid = target;
        heater_on = true;
        return Signature<TURN_ON>::ReturnValue {};
    };

    const auto handle_turn_off = [&](Signature<TURN_OFF>::Parameters params) {
        turn_off();
        heater_on = false;
        return Signature<TURN_OFF>::ReturnValue {};
    };

    const auto handle_status = [&](Signature<STATUS>::Parameters params) {
        return Signature<STATUS>::ReturnValue {heater_on};
    };

    const auto regulate_temperature = [&turn_on, &turn_off](int16_t target) {
        using namespace thermocouple;

        const auto parameters = std::make_tuple();
        Future<READ> future;
        future.init();

        const auto send_status = commands::send<READ>(parameters, future, 0);
        if (not defines::status::is_ok(send_status)) {
            turn_off();
            return defines::status::map_from(send_status);
        }

        ReturnValue<READ> return_value;

        auto get_status = future.get(return_value, loop_delay);
        if (not defines::status::is_ok(get_status)) {
            turn_off();

            get_status = future.get(return_value, osWaitForever);
            if (not defines::status::is_ok(get_status)) {
                return defines::status::map_from(get_status);
            }
        }

        const auto [read_status, heater, liquid] = return_value;
        if (not defines::status::is_ok(read_status)) {
            turn_off();
            return defines::status::map_from(read_status);
        }

        constexpr int16_t max_heater_temperature = 90;
        constexpr int16_t min_heater_temperature = 80;
        if (heater > max_heater_temperature) {
            turn_off();
        }
        else if (heater < min_heater_temperature) {
            turn_on();
        }

        return Status::OK;
    };

    while (true) {
        const uint32_t flags = _wait_flags(osFlagsWaitAny, loop_delay);
        for (const auto& flag : _iterate_flags(flags)) {
            switch (flag) {
            case ThreadFlag::TURN_ON: {
                handle_queue<TURN_ON>(TURN_ON_QUEUE, handle_turn_on);
                break;
            }
            case ThreadFlag::TURN_OFF: {
                handle_queue<TURN_OFF>(TURN_OFF_QUEUE, handle_turn_off);
                break;
            }
            case ThreadFlag::STATUS: {
                handle_queue<STATUS>(STATUS_QUEUE, handle_status);
                break;
            }
            case ThreadFlag::UNKNOWN: {
                break;
            }
            }
        }

        if (heater_on) {
            regulate_temperature(target_liquid);
        }
    }
}

extern "C" void init_heater_control_task_globals(osThreadId_t handle)
{
    THREAD.set_handle(handle);
#define INIT_QUEUE(COMMAND) commands::heater::COMMAND##_QUEUE.init();
    FOR_ALL_HEATER_COMMANDS(INIT_QUEUE)
#undef INIT_QUEUE
}

/**
 * Control the heater based on given commands.
 */
extern "C" void start_heater_control_task(void* parameters)
{
    UNUSED(parameters);
    THREAD.target();
}
