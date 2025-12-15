#pragma once

#include "FreeRTOS.h"
#include "FreeRTOS_CLI.h"

#include "defines/commands.hpp"

namespace console::commands {

#define DECLARE_CLI_COMMAND_DEFINITION(CMD) extern const CLI_Command_Definition_t CMD;
FOR_ALL_CLI_COMMANDS(DECLARE_CLI_COMMAND_DEFINITION)
#undef DECLARE_CLI_COMMAND_DEFINITION

inline void register_all(void)
{
#define REGISTER_CLI_COMMAND(CMD) FreeRTOS_CLIRegisterCommand(&CMD);
    FOR_ALL_CLI_COMMANDS(REGISTER_CLI_COMMAND)
#undef REGISTER_CLI_COMMAND
}

} // namespace console::commands
