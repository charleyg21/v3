/**
 * @file
 *
 * Implemention of valve console commands.
 */

#include <cstring>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#include "task.h"

#include "main.h"

#include "commands/valve.hpp"
#include "console/commands.hpp"
#include "defines/status.hpp"
#include "helpers/buffer_printer.hpp"

namespace console::commands {

using Status = defines::Status;

#define FOR_ALL_VALVE_POSITIONS(FUNC)                                                  \
    FUNC(HOME, home)                                                                   \
    FUNC(WATER_IN, water_in)                                                           \
    FUNC(WATER_OUT, water_out)                                                         \
    FUNC(SUPERCHARGER, supercharger)                                                   \
    FUNC(SPORES, spores)

#define CREATE_MOVE_TO_POSITION_COMMAND(UPPER, LOWER)                                  \
    BaseType_t run_valve_move_to_##LOWER(                                              \
        char* write_buffer, size_t write_buffer_len, const char* command_string)       \
    {                                                                                  \
        UNUSED(write_buffer_len);                                                      \
                                                                                       \
        helpers::BufferPrinter printer(write_buffer, write_buffer_len);                \
                                                                                       \
        using Position = ::commands::valve::Position;                                  \
        const auto status = ::commands::valve::move_to(Position::UPPER);               \
        const auto message                                                             \
            = (defines::status::is_ok(status)) ? "Success!\r\n" : "Error!\r\n";        \
        printer.puts(message);                                                         \
                                                                                       \
        return pdFALSE;                                                                \
    }                                                                                  \
                                                                                       \
    const CLI_Command_Definition_t valve_move_to_##LOWER = { "valve_move_to_" #LOWER,  \
        "valve_move_to_" #LOWER ": Move the valve to the home position.\r\n",          \
        run_valve_move_to_##LOWER, 0 };

FOR_ALL_VALVE_POSITIONS(CREATE_MOVE_TO_POSITION_COMMAND)

#undef CREATE_MOVE_TO_POSITION_COMMAND

BaseType_t run_valve_rehome(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    UNUSED(write_buffer_len);

    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    const auto status = ::commands::valve::rehome();

    const auto message
        = (defines::status::is_ok(status)) ? "Success!\r\n" : "Error!\r\n";
    printer.puts(message);

    return pdFALSE;
}

const CLI_Command_Definition_t valve_rehome
    = { "valve_rehome", "valve_rehome: Rehome the valve.\r\n", run_valve_rehome, 0 };

} // namespace console::commands
