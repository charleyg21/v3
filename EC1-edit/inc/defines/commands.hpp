/**
 * @file
 *
 * FreeRTOS-CLI command defines.
 */

#pragma once

#define FOR_ALL_CLI_COMMANDS(FUNC)                                                     \
    FUNC(heater_regulate)                                                              \
    FUNC(heater_turn_off)                                                              \
    FUNC(read_thermocouple)                                                            \
    FUNC(syringe_pull)                                                                 \
    FUNC(syringe_push)                                                                 \
    FUNC(syringe_rehome)                                                               \
    FUNC(task_stats)                                                                   \
    FUNC(valve_move_to_home)                                                           \
    FUNC(valve_move_to_water_in)                                                       \
    FUNC(valve_move_to_water_out)                                                      \
    FUNC(valve_move_to_supercharger)                                                   \
    FUNC(valve_move_to_spores)                                                         \
    FUNC(valve_rehome)                                                                 \
    FUNC(rtc_get_time)                                                                 \
    FUNC(rtc_set_time)                                                                 \
    FUNC(rtc_get_timer)
