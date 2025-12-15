/**
 * @file
 *
 * Implemention of valve console commands.
 */

#include <charconv>
#include <cstring>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#include "task.h"

#include "main.h"

#include "console/commands.hpp"

#include "commands/syringe.hpp"
#include "defines/status.hpp"
#include "helpers/buffer_printer.hpp"

namespace console::commands {

using Status = defines::Status;

static std::pair<Status, int> parse_cli_parameter(
    const char* command_string, UBaseType_t parameter)
{
    BaseType_t length;
    const char* start = FreeRTOS_CLIGetParameter(command_string, parameter, &length);
    const char* stop = start + length;

    int result;
    const auto [p, ec] = std::from_chars(start, stop, result);
    const auto status = (ec == std::errc()) ? Status::OK : Status::ERROR;
    return std::pair(status, result);
}

BaseType_t run_syringe_pull(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    const auto [parse_status, steps] = parse_cli_parameter(command_string, 1);
    if (not defines::status::is_ok(parse_status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    const auto command_status = ::commands::syringe::pull(steps);
    if (not defines::status::is_ok(command_status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    printer.puts("Success!\r\n");
    return pdFALSE;
}

const CLI_Command_Definition_t syringe_pull
    = { "syringe_pull", "syringe_pull: Pull the syringe a given number of steps.\r\n",
          run_syringe_pull, 1 };

BaseType_t run_syringe_push(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    const auto [parse_status, steps] = parse_cli_parameter(command_string, 1);
    if (not defines::status::is_ok(parse_status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    const auto command_status = ::commands::syringe::push(steps);
    if (not defines::status::is_ok(command_status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    printer.puts("Success!\r\n");
    return pdFALSE;
}

const CLI_Command_Definition_t syringe_push
    = { "syringe_push", "syringe_push: Push the syringe a given number of steps.\r\n",
          run_syringe_push, 1 };

BaseType_t run_syringe_rehome(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    UNUSED(write_buffer_len);

    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    const auto command_status = ::commands::syringe::rehome();
    if (not defines::status::is_ok(command_status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    printer.puts("Success!\r\n");
    return pdFALSE;
}

const CLI_Command_Definition_t syringe_rehome = { "syringe_rehome",
    "syringe_rehome: Rehome the syringe.\r\n", run_syringe_rehome, 0 };

} // namespace console::commands
