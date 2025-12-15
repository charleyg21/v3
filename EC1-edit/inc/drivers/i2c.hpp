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
 * @brief I2C driver class definition.
 */

#pragma once

#include <cstdint>

#include "main.h"

#include "defines/status.hpp"
#include "helpers/cmsis_os.hpp"

namespace drivers {

/**
 * @brief I2C driver class definition.
 */
class I2C {
public:
    using Status = defines::Status;

    /**
     * @brief Instantiate the I2C driver object.
     * @param hi2c    Pointer to ST HAL I2C handle object.
     * @param mutex   Reference CMSIS-OS I2C mutex object.
     * @param timeout Timeout to use for all OS and HAL operations.
     */
    explicit I2C(I2C_HandleTypeDef* hi2c, cmsis_os::Mutex& mutex,
        uint32_t timeout = HAL_MAX_DELAY)
        : _mutex(mutex)
        , _hi2c(hi2c)
        , _timeout(timeout)
    {
    }

    /**
     * @brief Transmit data to a device over I2C.
     * @param address Address of I2C device to send to.
     * @param data    Pointer to data buffer to send.
     * @param size    Number of bytes of data to send.
     * @return System::OK if successful; Status error value otherwise.
     */
    Status transmit(uint8_t address, const uint8_t* data, uint16_t size)
    {
        osStatus_t os_status;
        if ((os_status = _mutex.acquire(_timeout)) != osOK) {
            return defines::status::map_from(os_status);
        }

        auto non_const_data = const_cast<uint8_t*>(data);
        auto status
            = HAL_I2C_Master_Transmit(_hi2c, address, non_const_data, size, _timeout);

        if ((os_status = _mutex.release()) != osOK) {
            return defines::status::map_from(os_status);
        }

        return defines::status::map_from(status);
    }

    /**
     * @brief Receive data from a device over I2C.
     * @param address Address of I2C device to receive from.
     * @param data    Pointer to data buffer to store received data.
     * @param size    Number of bytes of data to receive.
     * @return System::OK if successful; Status error value otherwise.
     */
    Status receive(uint8_t address, uint8_t* data, uint16_t size)
    {
        osStatus_t os_status;
        if ((os_status = _mutex.acquire(_timeout)) != osOK) {
            return defines::status::map_from(os_status);
        }

        auto status = HAL_I2C_Master_Receive(_hi2c, address, data, size, _timeout);

        if ((os_status = _mutex.release()) != osOK) {
            return defines::status::map_from(os_status);
        }

        return defines::status::map_from(status);
    }

    /**
     * @brief Read from a device's register over I2C.
     * @param address   Address of I2C device to read from.
     * @param register_ Register address to read.
     * @param data      Pointer to data buffer to store received data.
     * @param size      Number of bytes of data to receive.
     * @return System::OK if successful; Status error value otherwise.
     */
    Status read_register(
        uint8_t address, uint8_t register_, uint8_t* data, uint16_t size)
    {
        osStatus_t os_status;
        if ((os_status = _mutex.acquire(_timeout)) != osOK) {
            return defines::status::map_from(os_status);
        }

        auto status = HAL_I2C_Mem_Read(
            _hi2c, address, register_, I2C_MEMADD_SIZE_8BIT, data, size, _timeout);

        if ((os_status = _mutex.release()) != osOK) {
            return defines::status::map_from(os_status);
        }

        return defines::status::map_from(status);
    }

    /**
     * @brief Write to a device's register over I2C.
     * @param address   Address of I2C device to write to.
     * @param register_ Register address to write to.
     * @param data      Pointer to data buffer to write.
     * @param size      Number of bytes of data to write.
     * @return System::OK if successful; Status error value otherwise.
     */
    Status write_register(
        uint8_t address, uint8_t register_, const uint8_t* data, uint16_t size)
    {
        osStatus_t os_status;
        if ((os_status = _mutex.acquire(_timeout)) != osOK) {
            return defines::status::map_from(os_status);
        }

        auto non_const_data = const_cast<uint8_t*>(data);
        auto status = HAL_I2C_Mem_Write(_hi2c, address, register_, I2C_MEMADD_SIZE_8BIT,
            non_const_data, size, _timeout);

        if ((os_status = _mutex.release()) != osOK) {
            return defines::status::map_from(os_status);
        }

        return defines::status::map_from(status);
    }

private:
    // Store a reference to the I2C mutex.
    cmsis_os::Mutex& _mutex;

    // Store a pointer to the I2C HAL handle.
    I2C_HandleTypeDef* const _hi2c;

    // Store the OS and HAL timeout value to use.
    const uint32_t _timeout;
};

} // namespace drivers
