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
 * @brief DAC driver function declarations.
 */

#pragma once

#include <cstdint>

#include "main.h"

#include "defines/status.hpp"

namespace drivers {

class DAConverter {
public:
    using Status = defines::Status;

    explicit DAConverter(DAC_HandleTypeDef* hdac)
        : _hdac(hdac)
    {
    }

    /**
     * @brief Start the DAC CH1 output.
     *
     * @return System status.
     */
    Status start_ch1(void)
    {
        auto status = HAL_DAC_Start(_hdac, DAC_CHANNEL_1);
        return defines::status::map_from(status);
    }

    /**
     * @brief Stop the DAC CH1 output.
     *
     * @return System status.
     */
    Status stop_ch1(void)
    {
        auto status = HAL_DAC_Stop(_hdac, DAC_CHANNEL_1);
        return defines::status::map_from(status);
    }

    /**
     * @brief Set the DAC CH1 value.
     *
     * @param value Input analog value
     *
     * @return System status.
     */
    Status set_value_ch1(uint16_t value, uint32_t alignment = DAC_ALIGN_12B_R)
    {
        auto status = HAL_DAC_SetValue(_hdac, DAC_CHANNEL_1, alignment, value);
        return defines::status::map_from(status);
    }

    /**
     * @brief Start the DAC CH2 output.
     *
     * @return System status.
     */
    Status start_ch2(void)
    {
        auto status = HAL_DAC_Start(_hdac, DAC_CHANNEL_2);
        return defines::status::map_from(status);
    }

    /**
     * @brief Stop the DAC CH2 output.
     *
     * @return System status.
     */
    Status stop_ch2(void)
    {
        auto status = HAL_DAC_Stop(_hdac, DAC_CHANNEL_2);
        return defines::status::map_from(status);
    }

    /**
     * @brief Set the DAC CH2 value.
     *
     * @param value Input analog value
     * @return System status.
     */
    Status set_value_ch2(uint16_t value, uint32_t alignment = DAC_ALIGN_12B_R)
    {
        auto status = HAL_DAC_SetValue(_hdac, DAC_CHANNEL_2, alignment, value);
        return defines::status::map_from(status);
    }

private:
    DAC_HandleTypeDef* _hdac;
};

} // namespace drivers
