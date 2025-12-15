/**
 * @file
 *
 * Implemention of RTC console commands.
 */

#include <charconv>
#include <cstring>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#include "task.h"

#include "main.h"

#include "console/commands.hpp"

#include "commands/config.hpp"
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

BaseType_t run_get_time(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    const auto [status, hours, minutes] = ::commands::config::get_time();
    if (not defines::status::is_ok(status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    printer.printf("%02d:%02d\r\n", hours, minutes);
    return pdFALSE;
}

const CLI_Command_Definition_t rtc_get_time
    = { "rtc_get_time", "rtc_get_time: Get the system time.\r\n", run_get_time, 0 };

BaseType_t run_set_time(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    const auto [hours_status, hours] = parse_cli_parameter(command_string, 1);
    if (not defines::status::is_ok(hours_status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    const auto [minutes_status, minutes] = parse_cli_parameter(command_string, 2);
    if (not defines::status::is_ok(minutes_status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    const auto status = ::commands::config::set_time(hours, minutes);
    if (not defines::status::is_ok(status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    printer.puts("Success!\r\n");
    return pdFALSE;
}

const CLI_Command_Definition_t rtc_set_time
    = { "rtc_set_time", "rtc_set_time: Set the system time.\r\n", run_set_time, 2 };

BaseType_t run_get_timer(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    const auto [status, hours, minutes] = ::commands::config::get_timer();
    if (not defines::status::is_ok(status)) {
        printer.puts("Error!\r\n");
        return pdFALSE;
    }

    printer.printf("%02d:%02d\r\n", hours, minutes);
    return pdFALSE;
}

const CLI_Command_Definition_t rtc_get_timer = { "rtc_get_timer",
    "rtc_get_timer: Get the dispense time.\r\n", run_get_timer, 0 };

} // namespace console::commands
