/**
 * @file
 *
 * Implemention of FreeRTOS-CLI "print_adc" command.
 */

#include <cstdio>

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"
#include "task.h"

#include "main.h"

#include "console/commands.hpp"
#include "controllers/adc.hpp"
#include "helpers/buffer_printer.hpp"

namespace console::commands {

BaseType_t run_print_adc(
    char* write_buffer, size_t write_buffer_len, const char* command_string)
{
    UNUSED(write_buffer_len);

    controllers::adc::Receiver receive_adc_values;
    receive_adc_values.init();

    controllers::adc::Values values;
    auto status = receive_adc_values(values);

    receive_adc_values.deinit();

    helpers::BufferPrinter printer(write_buffer, write_buffer_len);

    if (status == defines::Status::OK) {
        printer.printf("cputemp [C] = %d\r\n", values.cpu_temperature);
        printer.printf("vbat   [mV] = %4d\r\n", values.vbat);
    } else {
        printer.puts("Error reading ADC.");
    }

    return pdFALSE;
}

const CLI_Command_Definition_t print_adc
    = { "print_adc", "print_adc: Prints current ADC values.\r\n", run_print_adc, 0 };

} // namespace console::commands
