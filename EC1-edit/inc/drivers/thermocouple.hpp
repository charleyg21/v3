/**
 * @file
 *
 * Temperature sensor driver functions.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <utility>

#include "main.h"

#include "drivers/gpio.hpp"

namespace drivers::thermocouple {

template <typename ReadBytes>
auto read_temperature(int16_t& temperature, ReadBytes& read_bytes)
{
    struct alignas(uint16_t) Reading {
        unsigned int state : 1;
        unsigned int device_id : 1;
        unsigned int thermocouple_input : 1;
        unsigned int temperature_reading : 12;
        unsigned int _dummy_sign_bit : 1;
    } __attribute__((__packed__));

    static_assert(
        sizeof(Reading) == sizeof(uint16_t), "Invalid size of reading data model.");

    union {
        Reading as_struct;
        uint8_t as_bytes[sizeof(Reading)];
        uint16_t as_uint16;
    } reading = { .as_uint16 = 0 };

    const auto read_status = read_bytes(reading.as_bytes, sizeof(reading.as_bytes));

    if (not defines::status::is_ok(read_status)) {
        return defines::status::map_from(read_status);
    }

    reading.as_uint16 = __builtin_bswap16(reading.as_uint16);

    if (not reading.as_uint16) {
        return defines::Status::ERROR;
    }

    // Calculate nearest degree rounding up.
    temperature = (reading.as_struct.temperature_reading + 2) / 4;
    return defines::Status::OK;
}

} // namespace drivers::thermocouple
