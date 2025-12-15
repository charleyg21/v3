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
 * @brief Buffer printer helper class definition.
 */

#pragma once

#include <algorithm>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace helpers {

/**
 * @brief Buffer printer class definition.
 */
class BufferPrinter {
public:
    /**
     * @brief Instantiate a buffer printer object.
     * @param buffer Reference to char array of size bytes.
     */
    template <std::size_t size>
    explicit BufferPrinter(char (&buffer)[size])
        : BufferPrinter(buffer, size)
    {
    }

    /**
     * @brief Instantiate a buffer printer object.
     * @param buffer Pointer to char array.
     * @param size   Size of char array in bytes.
     */
    BufferPrinter(char* buffer, std::size_t size)
        : _buffer(buffer)
        , _size(size)
        , _position(buffer)
        , _remaining(size)
    {
        memset(buffer, 0, size);
    }

    /**
     * @brief Print formatted text into the buffer.
     * @param  format  Format string.
     * @param  vargs   Variable arguments in style of printf.
     * @return written Number of characters written.
     */
    std::size_t printf(const char* format, ...)
    {
        va_list va_args;
        va_start(va_args, format);
        const std::size_t length = vsnprintf(_position, _remaining, format, va_args);
        va_end(va_args);

        const std::size_t written = std::min(length, _remaining - 1);

        _position += written;
        _remaining -= written;

        return written;
    }

    /**
     * @brief Put a string into the buffer.
     * @param  string  String to copy into the buffer.
     * @param  strlen  Length of string to copy into the buffer.
     * @return written Number of characters written.
     */
    std::size_t puts(const char* s, std::size_t strlen)
    {
        const std::size_t written = std::min(strlen, _remaining - 1);

        memcpy(_buffer, s, written);

        _position += written;
        _remaining -= written;

        return written;
    }

    /**
     * @brief Put a string into the buffer.
     * @param  string  String to copy into the buffer.
     * @return written Number of characters written.
     */
    std::size_t puts(const char* s) { return puts(s, strlen(s)); }

    /**
     * @brief Put a string into the buffer.
     * @param  string  Reference to string of size bytes.
     * @return written Number of characters written.
     */
    template <std::size_t size> std::size_t puts(const char (&s)[size])
    {
        const std::size_t strlen = (size > 0) ? size - 1 : std::size_t { 0 };
        return puts(s, strlen);
    }

    /**
     * @brief Reset the buffer to an empty state.
     */
    void reset(void)
    {
        _position = _buffer;
        _remaining = _size;
        memset(_buffer, 0, _size);
    }

private:
    // Store a pointer to the buffer.
    char* const _buffer;

    // Store the size of the buffer.
    const std::size_t _size;

    // Store the current position in the buffer.
    char* _position;

    // Store the remaining number of characters in the buffer.
    std::size_t _remaining;
};

} // namespace helpers
