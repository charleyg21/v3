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
 * @date May 13th, 2021
 *
 * @brief Definitions and helpers for capturing system status.
 */

#pragma once

#include <cstdint>

#include "cmsis_os.h"

#include "main.h"

namespace defines {

/**
 * @brief Enumeration for capturing system status.
 *
 * Very high-level status that covers basic error cases.
 */
enum class Status : uint8_t {
    OK,
    BUSY,
    ERROR,
    TIMEOUT,
};

namespace status {

    inline Status map_from(Status status) { return status; }

    /**
     * @brief Map an osStatus_t to a system status.
     */
    inline Status map_from(osStatus_t status)
    {
        switch (status) {
        case osOK:
            return Status::OK;
        case osErrorTimeout:
            return Status::TIMEOUT;
        default:
            return Status::ERROR;
        }
    }

    /**
     * @brief Map a HAL_StatusTypeDef to a system status.
     */
    inline Status map_from(HAL_StatusTypeDef status)
    {
        switch (status) {
        case HAL_OK:
            return Status::OK;
        case HAL_TIMEOUT:
            return Status::TIMEOUT;
        default:
            return Status::ERROR;
        }
    }

    inline bool is_ok(osStatus status) { return status == osOK; }
    inline bool is_ok(HAL_StatusTypeDef status) { return status == HAL_OK; }
    inline bool is_ok(Status status) { return status == Status::OK; }

} // namespace status

} // namespace defines