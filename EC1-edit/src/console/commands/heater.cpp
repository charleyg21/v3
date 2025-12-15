/**
 * @file
 *
 * Implemention of valve console commands.
 */

#include <charconv>
#include <cstring>
#include <utility>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#include "task.h"

#include "main.h"

#include "commands/heater.hpp"
#include "console/commands.hpp"
#include "defines/status.hpp"
#include "helpers/buffer_printer.hpp"

namespace console::commands {

using Status = defines::Status;

BaseType_t run_heater_regulate(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    UNUSED(write_buffer_len);

    helpers::BufferPrinter printer(write_buffer, write_buffer_len);
    printer.puts("Success!\r\n");

    return pdFALSE;
}

const CLI_Command_Definition_t heater_regulate = { "heater_regulate",
    "heater_regulate: Regulate temperature of syringe.\r\n", run_heater_regulate, 0 };

BaseType_t run_heater_turn_off(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    UNUSED(write_buffer_len);

    helpers::BufferPrinter printer(write_buffer, write_buffer_len);
    printer.puts("Success!\r\n");

    return pdFALSE;
}

const CLI_Command_Definition_t heater_turn_off = { "heater_turn_off",
    "heater_turn_off: Turn off the heater syringe.\r\n", run_heater_turn_off, 0 };

} // namespace console::commands
