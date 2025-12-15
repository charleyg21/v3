/*
 * Copyright © 2021 Jonathan Starr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

/**
 * @file
 *
 * @author Jonathan Starr <jonstarr@utexas.edu>
 * @date April 2nd, 2021
 *
 * @brief Display driver class definition.
 */

#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "defines/status.hpp"

#include "drivers/i2c.hpp"

#include "helpers/buffer_printer.hpp"

namespace drivers {

/**
 * @brief Display driver class.
 */
class Display {
public:
    using Status = defines::Status;

    /**
     * @brief Instantiate a display driver object.
     */
    explicit Display(I2C& i2c, uint32_t timeout = HAL_MAX_DELAY)
        : _i2c(i2c)
    {
    }

    /**
     * @brief Specify the display line length.
     */
    static constexpr std::size_t LINE_LENGTH = 20;

    /**
     * @brief Initialize display.
     */
    Status init(void)
    {
        if (_initialized) {
            return Status::ERROR;
        }

        Status status;

        status = _function_set(DataInterface::WIDTH_8_BITS, NumLines::TWO,
            DoubleHeight::OFF, InstructionTable::TABLE0);
        if (status != Status::OK) {
            return status;
        }
        _delay_ms(10);

        status = _function_set(DataInterface::WIDTH_8_BITS, NumLines::TWO,
            DoubleHeight::OFF, InstructionTable::TABLE1);
        if (status != Status::OK) {
            return status;
        }
        _delay_ms(10);

        status = _bias_set(Bias::QUARTER, Fixed::LOW);
        if (status != Status::OK) {
            return status;
        }

        const uint8_t contrast = 0b0001'1000;
        status = _contrast_set(contrast);

        if (status != Status::OK) {
            return status;
        }

        status = _power_icon_control_contrast_set(Icon::ON, Booster::ON, contrast);
        if (status != Status::OK) {
            return status;
        }

        const uint8_t Rab = 0b101;
        status = _follower_control(Follower::ON, Rab);
        if (status != Status::OK) {
            return status;
        }

        status = _display_on_off(DisplayPower::ON, Cursor::OFF, Blinking::OFF);
        if (status != Status::OK) {
            return status;
        }

        status = _clear_display();
        if (status != Status::OK) {
            return status;
        }

        status = _entry_mode_set(CursorMoveDirection::INCREMENT, Shift::OFF);
        if (status != Status::OK) {
            return status;
        }
        _delay_ms(10);

        status = _function_set(DataInterface::WIDTH_8_BITS, NumLines::TWO,
            DoubleHeight::OFF, InstructionTable::TABLE0);
        if (status != Status::OK) {
            return status;
        }
        _delay_ms(10);

        _initialized = true;
        return Status::OK;
    }

    /**
     * @brief Clear the display.
     */
    Status clear(void)
    {
        Status status = _clear_display();
        _delay_ms(10); // Delay needed to give display time to clear.
        return status;
    }

    /**
     * @brief Write a string to the first line of display.
     */
    Status line1_puts(const char* line)
    {
        // set DDRAM address
        const uint8_t line1_address = 0x00;
        return _puts_at_ddram_address(line, line1_address);
    }

    /**
     * @brief Write formatted text to first line of display.
     */
    Status line1_printf(const char* format, ...)
    {
        char line[LINE_LENGTH + 1] = {};

        /* Declare a va_list type variable */
        va_list va_args;

        /* Initialise the va_list variable with the ... after fmt */
        va_start(va_args, format);

        /* Forward the '...' to vprintf */
        vsnprintf(line, LINE_LENGTH + 1, format, va_args);

        /* Clean up the va_list */
        va_end(va_args);

        return line1_puts(line);
    }

    /**
     * @brief Write a string to the second line of display.
     */
    Status line2_puts(const char* line)
    {
        // set DDRAM address
        const uint8_t line2_address = 0x40;
        return _puts_at_ddram_address(line, line2_address);
    }

    /**
     * @brief Write formatted text to the second line of display.
     */
    Status line2_printf(const char* format, ...)
    {
        char line[LINE_LENGTH + 1] = {};

        /* Declare a va_list type variable */
        va_list va_args;

        /* Initialise the va_list variable with the ... after fmt */
        va_start(va_args, format);

        /* Forward the '...' to vprintf */
        vsnprintf(line, LINE_LENGTH + 1, format, va_args);

        /* Clean up the va_list */
        va_end(va_args);

        return line2_puts(line);
    }

private:
    /// Maintain a static address for the display.
    static constexpr uint8_t _ADDRESS = 0x78;

    /// Track if this display has already been initialized.
    bool _initialized = false;

    /// Maintain a reference to an i2c driver object.
    I2C& _i2c;

    /**
     * @brief Delay a given number of milliseconds.
     */
    void _delay_ms(uint32_t ms) { osDelay(ms); }

    /**
     * @brief Send a sequence of bytes to the display.
     */
    Status _send_bytes(const uint8_t* bytes, uint16_t size)
    {
        return _i2c.transmit(_ADDRESS, bytes, size);
    }

    /**
     * @brief Reinterpret a given struct type as a byte.
     */
    template <typename T> uint8_t _reinterpret_value_as_uint8(T value)
    {
        static_assert(sizeof(value) == 1, "All control/data values must be one byte.");

        const union {
            T value;
            uint8_t uint8;
        } value_as_union = { .value = value };

        return value_as_union.uint8;
    }

    /**
     * @brief Send a command to the display.
     */
    Status _send_command(uint8_t data_byte)
    {
        const struct __packed {
            unsigned int zeros : 6;
            bool RS : 1;
            bool Co : 1;
        } bitfield = {
            .zeros = 0,
            .RS = false,
            .Co = false,
        };

        const uint8_t control_byte = _reinterpret_value_as_uint8(bitfield);

        const uint8_t bytes[] = {
            control_byte,
            data_byte,
        };

        return _send_bytes(bytes, std::extent<decltype(bytes)>::value);
    }

    /*********************
     * Base Instructions *
     *********************/

    /**
     * @brief Send the "clear display" command.
     */
    Status _clear_display(void)
    {
        struct __packed {
            unsigned int prefix : 8;
        } bitfield { .prefix = 0b00000001 };

        const uint8_t control_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(control_byte);
    }

    /**
     * @brief Enumerate possible directions that the cursor can move.
     */
    enum class CursorMoveDirection {
        DECREMENT = 0b0,
        INCREMENT = 0b1,
    };

    /**
     * @brief Enumerate display shift options.
     */
    enum class Shift {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Send the "entry mode set" command.
     */
    Status _entry_mode_set(CursorMoveDirection ID, Shift S)
    {
        struct __packed {
            Shift S : 1;
            CursorMoveDirection ID : 1;
            unsigned int prefix : 6;
        } bitfield { .S = S, .ID = ID, .prefix = 0b000001 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /**
     * @brief Enumerate display power on/off options.
     */
    enum class DisplayPower {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Enumerate cursor on/off options.
     */
    enum class Cursor {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Enumerate display blinking on/off options.
     */
    enum class Blinking {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Send the "display on/off" command.
     */
    Status _display_on_off(DisplayPower D, Cursor C, Blinking B)
    {
        struct __packed {
            Blinking B : 1;
            Cursor C : 1;
            DisplayPower D : 1;
            unsigned int prefix : 5;
        } bitfield { .B = B, .C = C, .D = D, .prefix = 0b00001 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /**
     * @brief Enumerate the data interface bus width options.
     */
    enum class DataInterface {
        WIDTH_4_BITS = 0b0,
        WIDTH_8_BITS = 0b1,
    };

    /**
     * @brief Enumerate the display line number options.
     */
    enum class NumLines {
        ONE = 0b0,
        TWO = 0b1,
    };

    /**
     * @brief Enumerate the display double height on/off options.
     */
    enum class DoubleHeight {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Enumerate the display instruction table options.
     */
    enum class InstructionTable {
        TABLE0 = 0b00,
        TABLE1 = 0b01,
        TABLE2 = 0b10,
    };

    /**
     * @brief Send the "function set" command.
     */
    Status _function_set(
        DataInterface DL, NumLines N, DoubleHeight DH, InstructionTable IS)
    {
        struct __packed {
            InstructionTable IS : 2;
            DoubleHeight DH : 1;
            NumLines N : 1;
            DataInterface DL : 1;
            unsigned int prefix : 3;
        } bitfield { .IS = IS, .DH = DH, .N = N, .DL = DL, .prefix = 0b001 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /**
     * @brief Send the "set DDRAM address" command.
     */
    Status _set_ddram_address(uint8_t address)
    {
        struct __packed {
            unsigned int address : 7;
            unsigned int prefix : 1;
        } bitfield { .address = address, .prefix = 0b1 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /***********************
     * Instruction Table 1 *
     ***********************/

    /**
     * @brief Enumerate the options for biasing.
     */
    enum class Bias {
        FIFTH = 0b0,
        QUARTER = 0b1,
    };

    /**
     * @brief Enumerate the display options for fixed high or low.
     */
    enum class Fixed {
        LOW = 0b0,
        HIGH = 0b1,
    };

    /**
     * @brief Send the "bias set" command.
     */
    Status _bias_set(Bias BS, Fixed FX)
    {
        struct __packed {
            Fixed FX : 1;
            unsigned int FB : 2;
            Bias BS : 1;
            unsigned int prefix : 4;
        } bitfield { .FX = FX, .FB = 0b10, .BS = BS, .prefix = 0b0001 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /**
     * @brief Enumerate the icon on/off options.
     */
    enum class Icon {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Enumerate the booster on/off options.
     */
    enum class Booster {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Send the "power/icon/control/contrast set" command.
     */
    Status _power_icon_control_contrast_set(Icon Ion, Booster Bon, uint8_t C)
    {
        // Select the 2 most-significant bits of contrast.
        C &= 0b00110000;
        C >>= 4;

        struct __packed {
            unsigned int C : 2;
            Booster Bon : 1;
            Icon Ion : 1;
            unsigned int prefix : 4;
        } bitfield { .C = C, .Bon = Bon, .Ion = Ion, .prefix = 0b0101 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /**
     * @brief Enumerate the follower on/off options.
     */
    enum class Follower {
        OFF = 0b0,
        ON = 0b1,
    };

    /**
     * @brief Send the "follower control" command.
     */
    Status _follower_control(Follower Fon, uint8_t Rab)
    {
        struct __packed {
            unsigned int Rab : 3;
            Follower Fon : 1;
            unsigned int prefix : 4;
        } bitfield { .Rab = Rab, .Fon = Fon, .prefix = 0b0110 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /**
     * @brief Send the "contrast set" command.
     */
    Status _contrast_set(uint8_t C)
    {
        const uint8_t contrast_mask = 0x0F;
        C &= contrast_mask; // Select the 4 least-significant bits of contrast.

        struct __packed {
            unsigned int C : 4;
            unsigned int prefix : 4;
        } bitfield { .C = C, .prefix = 0b0111 };

        const auto command_byte = _reinterpret_value_as_uint8(bitfield);
        return _send_command(command_byte);
    }

    /**
     * @brief Put a string at a specific DDRAM address.
     */
    Status _puts_at_ddram_address(const char* line, uint8_t address)
    {
        // set DDRAM address
        _set_ddram_address(address);

        uint8_t buffer[LINE_LENGTH + 1] = {};
        uint8_t length = std::min(strlen(line), LINE_LENGTH);

        const struct __packed {
            unsigned int zeros : 6;
            bool RS : 1;
            bool Co : 1;
        } control = {
            .zeros = 0,
            .RS = true,
            .Co = false,
        };

        buffer[0] = _reinterpret_value_as_uint8(control);
        memcpy(buffer + 1, line, length);

        return _send_bytes(buffer, length + 1);
    }
};

} // namespace drivers
