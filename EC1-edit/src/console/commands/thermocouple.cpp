/**
 * @file
 *
 * Implemention of FreeRTOS-CLI "tempsensor" command.
 */

#include <cstdio>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "task.h"

#include "main.h"

#include "commands/thermocouple.hpp"
#include "console/commands.hpp"
#include "helpers/buffer_printer.hpp"

namespace console::commands {

BaseType_t run_read_thermocouple(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    UNUSED(write_buffer_len);

    int16_t heater;
    int16_t liquid;
    bool overheated;
    auto status = ::commands::thermocouple::read(heater, liquid, overheated);

    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    if (defines::status::is_ok(status)) {
        printer.printf("heater [C] = %d, ", heater);
        printer.printf("liquid [C] = %d\r\n", liquid);
    } else {
        printer.puts("Error reading thermocouple.\r\n");
    }

    return pdFALSE;
}

const CLI_Command_Definition_t read_thermocouple
    = { "read_thermocouple", "read_thermocouple: Read current thermocouple values.\r\n",
          run_read_thermocouple, 0 };

} // namespace console::commands
