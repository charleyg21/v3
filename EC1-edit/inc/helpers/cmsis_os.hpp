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
 * @brief Definitions for CMSIS-OS helper classes.
 *
 * The following class definitions are wrappers for CMSIS-OS primitives. They
 * are generally pretty lightweight and it's a good idea to refer to the
 * CMSIS-OSv2 documentation for more information on any of the functions which
 * are delegated to.
 */

#pragma once

#include <algorithm>
#include <climits>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <numeric>

#include "cmsis_os.h"

#include "main.h"

namespace cmsis_os {

/**
 * @brief CMSIS-OS message queue class definition.
 * @param T Typename of underlying stored type.
 */
template <typename T> class MessageQueue {
public:
    /**
     * @brief Instantiate a CMSIS-OS message queue object.
     * @param handle Reference to CMSIS-OS message queue handle.
     */
    explicit MessageQueue(osMessageQueueId_t handle = nullptr)
        : _handle(handle)
    {
    }

    /**
     * @brief Destruct a CMSIS-OS message queue object.
     */
    virtual ~MessageQueue(void) = default;

    /**
     * @brief Put a message into the message queue.
     * @param  msg     Reference of message to put into the message queue.
     * @param  timeout Timeout to wait for OS operation.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t put(const T& msg, uint32_t timeout)
    {
        return osMessageQueuePut(_handle, static_cast<const void*>(&msg), 0, timeout);
    }

    /**
     * @brief Put a message into the message queue.
     * @param[out] msg     Reference of message to store get result in.
     * @param      timeout Timeout to wait for OS operation.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t get(T& msg, uint32_t timeout)
    {
        return osMessageQueueGet(_handle, static_cast<void*>(&msg), NULL, timeout);
    }

    /**
     * @brief Reset the message queue.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t reset(void) { return osMessageQueueReset(_handle); }

    /**
     * @brief Get the current count of messages in the message queue.
     * @return Number of messages in the message queue.
     */
    uint32_t get_count(void) { return osMessageQueueGetCount(_handle); }

    /**
     * @brief Get the CMSIS-OS message queue handle.
     * @return The CMSIS-OS message queue handle.
     */
    osMessageQueueId_t get_handle(void) { return _handle; }

    /**
     * @brief Set the CMSIS-OS message queue handle.
     * @param handle The CMSIS-OS message queue handle.
     */
    void set_handle(osMessageQueueId_t handle) { _handle = handle; }

private:
    // Store a reference to the message queue handle.
    osMessageQueueId_t _handle;
};

/**
 * @brief CMSIS-OS static message queue class definition.
 * @param T    Typename of underlying stored type.
 * @param SIZE Size of queue in terms of number of messages.
 *
 * A StaticMessageQueue differs from its parent MessageQueue in that it
 * provides an allocation for the control blocks and buffers and further
 * provides init() and deinit() methods.
 */
template <typename T, size_t SIZE> class StaticMessageQueue : public MessageQueue<T> {
public:
    /**
     * @brief Instantiate a CMSIS-OS message queue object.
     */
    explicit StaticMessageQueue(void)
        : MessageQueue<T>()
        , _name(nullptr)
    {
    }

    /**
     * @brief Instantiate a CMSIS-OS message queue object.
     * @param name Name of message queue.
     */
    explicit StaticMessageQueue(const char* name)
        : MessageQueue<T>()
        , _name(name)
    {
    }

    /**
     * @brief Destruct a CMSIS-OS message queue object.
     */
    ~StaticMessageQueue(void) { deinit(); }

    /**
     * @brief Initialize a CMSIS-OS message queue object.
     */
    void init(void)
    {
        if (_initialized) {
            return;
        }

        _initialized = true;
        const auto handle = osMessageQueueNew(SIZE, sizeof(T), &_attrs);
        MessageQueue<T>::set_handle(handle);
    }

    /**
     * @brief Deinitialize a CMSIS-OS message queue object.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t deinit(void)
    {
        if (not _initialized) {
            return osError;
        }

        _initialized = false;
        const auto handle = MessageQueue<T>::get_handle();
        return osMessageQueueDelete(handle);
    }

private:
    // Maintain a pointer to the message queue name (if any).
    const char* _name;

    // Maintain whether this object has been initialized or not.
    bool _initialized = false;

    // Allocate space for underlying control block and buffer.
    StaticQueue_t _control_block;
    uint8_t _buffer[SIZE * sizeof(T)] = {};

    // Create attributes struct for static allocation.
    const osMessageQueueAttr_t _attrs = {
        .name = _name,
        .attr_bits = 0,
        .cb_mem = &_control_block,
        .cb_size = sizeof(_control_block),
        .mq_mem = &_buffer,
        .mq_size = sizeof(_buffer),
    };
};

/**
 * @brief CMSIS-OS mutex class definition.
 */
class Mutex {
public:
    /**
     * @brief Instantiate an CMSIS-OS mutex object.
     * @param handle Reference to CMSIS-OS mutex handle.
     */
    explicit Mutex(osMutexId_t handle = nullptr)
        : _handle(handle)
    {
    }

    /**
     * @brief Destruct an CMSIS-OS mutex object.
     */
    virtual ~Mutex(void) = default;

    /**
     * @brief Acquire the mutex.
     * @param  timeout Timeout to wait for OS operation.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t acquire(uint32_t timeout) { return osMutexAcquire(_handle, timeout); }

    /**
     * @brief Release the mutex.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t release(void) { return osMutexRelease(_handle); }

    /**
     * @brief Get the CMSIS-OS mutex handle.
     * @return The CMSIS-OS mutex handle.
     */
    osMutexId_t get_handle(void) { return _handle; }

    /**
     * @brief Set the CMSIS-OS mutex handle.
     * @param handle The CMSIS-OS mutex handle.
     */
    void set_handle(osMutexId_t handle) { _handle = handle; }

private:
    // Store a reference to the mutex handle.
    osMutexId_t _handle;
};

/**
 * @brief CMSIS-OS static mutex class definition.
 *
 * A StaticMutex differs from its parent Mutex in that it provides an
 * allocation for the control blocks and buffers and further provides init()
 * and deinit() methods.
 */
class StaticMutex : Mutex {
public:
    /**
     * @brief Instantiate a CMSIS-OS static mutex object.
     */
    explicit StaticMutex(void)
        : Mutex()
        , _name(nullptr)
    {
    }

    /**
     * @brief Instantiate a CMSIS-OS mutex object.
     * @param name Name of message queue.
     */
    explicit StaticMutex(const char* name)
        : Mutex()
        , _name(name)
    {
    }

    /**
     * @brief Destruct a CMSIS-OS mutex object.
     */
    ~StaticMutex(void) { deinit(); }

    /**
     * @brief Initialize a CMSIS-OS mutex object.
     */
    void init(void)
    {
        if (_initialized) {
            return;
        }

        _initialized = true;
        const auto handle = osMutexNew(&_attrs);
        set_handle(handle);
    }

    /**
     * @brief Deinitialize a CMSIS-OS mutex object.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t deinit(void)
    {
        if (not _initialized) {
            return osError;
        }

        _initialized = false;
        const auto handle = get_handle();
        return osMutexDelete(handle);
    }

private:
    // Maintain a pointer to the message queue name (if any).
    const char* _name;

    // Maintain whether this object has been initialized or not.
    bool _initialized = false;

    // Allocate space for underlying control block and buffer.
    StaticSemaphore_t _control_block;

    // Create attributes struct for static allocation.
    const osMutexAttr_t _attrs = {
        .name = _name,
        .attr_bits = 0,
        .cb_mem = &_control_block,
        .cb_size = sizeof(_control_block),
    };
};

/**
 * @brief CMSIS-OS thread class definition.
 */
class Thread {
public:
    /**
     * @brief Instantiate a CMSIS-OS thread object.
     * @param handle Reference to CMSIS-OS thread handle.
     */
    explicit Thread(osThreadId_t handle = nullptr)
        : _handle(handle)
    {
    }

    /**
     * @brief Destruct a CMSIS-OS thread object.
     */
    virtual ~Thread(void) = default;

    /**
     * @brief Get the CMSIS-OS thread handle.
     * @return The CMSIS-OS thread handle.
     */
    osThreadId_t get_handle(void) { return _handle; }

    /**
     * @brief Set the CMSIS-OS thread handle.
     * @param handle The thread handle.
     */
    void set_handle(osThreadId_t handle) { _handle = handle; }

private:
    // Store handle to the CMSIS-OS thread.
    osThreadId_t _handle;
};

/**
 * @brief CRTP-style mix-in class for creating "Flaggable" threads.
 *
 * "Flaggable" threads can have their thread flags externally. Additionally,
 * "Flaggable" threads can have its own flags accessed and manipulated by
 * methods of its class.
 *
 * @param Derived Class that is deriving from this class in the CRTP-style.
 */
template <typename Derived, typename Flag, Flag UNKNOWN = Flag::UNKNOWN,
    typename FlagList = std::initializer_list<Flag>>
class Flaggable {
public:
    constexpr explicit Flaggable(const FlagList& flags)
        : _flags(flags)
    {
    }

    /**
     * @brief Set a flag in this thread's list of thread flags.
     * @param flag Flag to set on this thread.
     * @return Bit vector of old thread flags before setting.
     */
    uint32_t set_flag(Flag flag) noexcept
    {
        osThreadId_t handle = static_cast<Derived*>(this)->get_handle();
        const uint32_t bv = _flag_to_bit_indicator(flag);
        return osThreadFlagsSet(handle, bv);
    }

protected:
    const FlagList& _flags;

    /**
     * @brief Convert a flag to a indicator bit vector.
     * @param flag Flag to convert to a bit vector indincator.
     * @return Bit vector indicator of this flag.
     */
    static constexpr uint32_t _flag_to_bit_indicator(Flag flag) noexcept
    {
        if (flag == UNKNOWN) {
            return 0;
        }
        return uint32_t { 1 } << static_cast<uint32_t>(flag);
    }

    /**
     * @brief Get a mask indicating given flags.
     * @return Bit vector of given thread flags.
     */
    static constexpr uint32_t _flags_mask(const FlagList& flag_list) noexcept
    {
        auto bitwise_OR_fold = [](uint32_t mask, Flag flag) {
            return mask | _flag_to_bit_indicator(flag);
        };

        return std::accumulate(
            std::cbegin(flag_list), std::cend(flag_list), 0, bitwise_OR_fold);
    }

    /**
     * @brief Clear given flags.
     * @return Bit vector of old thread flags before clearing.
     */
    static uint32_t _clear_flags(const FlagList& flag_list) noexcept
    {
        const uint32_t mask = _flags_mask(flag_list);
        return osThreadFlagsClear(mask);
    }

    /**
     * @brief Clear all flags.
     * @return Bit vector of old thread flags before clearing.
     */
    uint32_t _clear_flags(void) noexcept { return _clear_flags(_flags); }

    /**
     * @brief Clear given flag.
     * @return Bit vector of old thread flags before clearing.
     */
    static uint32_t _clear_flag(Flag flag) noexcept { return _clear_flags({ flag }); }

    /**
     * @brief Check if a thread flags vector has an error code set.
     * @return true if the given an error code is set; otherwise, false.
     */
    static constexpr bool _flags_have_error_code(uint32_t flags) noexcept
    {
        const uint32_t pos = CHAR_BIT * sizeof(uint32_t) - 1;
        const uint32_t msb = 1 << pos;
        return HAL_IS_BIT_SET(flags, msb);
    }

    /**
     * @brief Check if a given thread flag is set in the given flags bit vector.
     * @param flags Flags bit vector to check against.
     * @param flag  Flag to check if set.
     * @return true if the given thread flag is set; otherwise, false.
     */
    static constexpr bool _flag_is_set(uint32_t flags, Flag flag) noexcept
    {
        if (_flags_have_error_code(flags) or flag == UNKNOWN) {
            return false;
        }
        const uint32_t bit = _flag_to_bit_indicator(flag);
        return HAL_IS_BIT_SET(flags, bit);
    }

    /**
     * @brief Check if a given thread flag is set.
     * @param flag Flag to check if set.
     * @return true if the given thread flag is set; otherwise, false.
     */
    static bool _flag_is_set(Flag flag) noexcept
    {
        const uint32_t flags = osThreadFlagsGet();
        return _flag_is_set(flags, flag);
    }

    /**
     * @brief Check if a given thread flag is clear.
     * @param flag  Flag to check if clear.
     * @return true if the given thread flag is clear; otherwise, false.
     */
    static bool _flag_is_clear(Flag flag) noexcept { return not _flag_is_set(flag); }

    /**
     * @brief Wait for given flags.
     * @param flag_list List of flags.
     * @return Bit vector of flags after waiting.
     */
    template <typename... Ts>
    static uint32_t _wait_flags(const FlagList& flag_list, Ts... params) noexcept
    {
        const uint32_t mask = _flags_mask(flag_list);
        return osThreadFlagsWait(mask, params...);
    }

    /**
     * @brief Wait for all flags.
     * @return Bit vector of flags after waiting.
     */
    template <typename... Ts> uint32_t _wait_flags(Ts... params) noexcept
    {
        return _wait_flags(_flags, params...);
    }

    /**
     * @brief Wait for given flag.
     * @param flag Flag to wait for.
     * @return Bit vector of flags after waiting.
     */
    template <typename... Ts>
    static uint32_t _wait_flag(Flag flag, Ts... params) noexcept
    {
        return _wait_flags({ flag }, osFlagsWaitAny, params...);
    }

    /**
     * @brief Iterator class for iterating over a thread flags bit vector.
     */
    class FlagIterator {
    public:
        // Create the type for an iterator over ThreadFlag::LIST.
        typedef decltype(std::cbegin(std::declval<const FlagList&>())) FlagListIterator;

        /**
         * @brief Instantiate a flag iterator object.
         * @param flags Thread flags bit vector of set thread flags.
         * @param begin Beginning iterator over the list of possible thread flags.
         * @param end   Ending iterator over the list of possible thread flags.
         */
        explicit constexpr FlagIterator(
            uint32_t flags, FlagListIterator begin, FlagListIterator end) noexcept
            : _flags { flags }
            , _it { begin }
            , _end { end }
        {
            _it = _seek_first(_it);
        }

        /**
         * @brief Post-fix increment to the next set thread flag.
         */
        FlagIterator operator++() noexcept
        {
            _it = _seek_next(_it);
            return *this;
        }

        /**
         * @brief Pre-fix increment to the next set thread flag.
         */
        FlagIterator operator++(int) noexcept
        {
            FlagIterator prev_this = *this;
            _it = _seek_next(_it);
            return prev_this;
        }

        /**
         * @brief Check if this iterator is equal to another one.
         * @param rhs The other iterator to test against.
         * @return true if equal; otherwise, false.
         */
        bool operator==(const FlagIterator& rhs) const noexcept
        {
            return (_it == rhs._it);
        }

        /**
         * @brief Check if this iterator is not equal to another one.
         * @param rhs The other iterator to test against.
         * @return true if not equal; otherwise, false.
         */
        bool operator!=(const FlagIterator& rhs) const noexcept
        {
            return not this->operator==(rhs);
        }

        /**
         * @brief Get the flag at the current iterator position.
         * @return The flag at the current position.
         */
        Flag operator*() const noexcept { return *_it; }

    private:
        // Maintain the current set of flags as a bit vector.
        const uint32_t _flags;

        // Maintain the possible thread flags iterator and its end point.
        FlagListIterator _it;
        FlagListIterator _end;

        /**
         * @brief Seek the first set thread flag.
         */
        constexpr FlagListIterator _seek_first(FlagListIterator it) noexcept
        {
            auto flag_is_set
                = [flags = _flags](auto flag) { return _flag_is_set(flags, flag); };
            return std::find_if(it, _end, flag_is_set);
        }

        /**
         * @brief Seek the next set thread flag.
         */
        constexpr FlagListIterator _seek_next(FlagListIterator it) noexcept
        {
            it++;
            return _seek_first(it);
        }
    };

    /**
     * @brief Represent a thread flag bit vector as an iterable type.
     */
    class FlagIterable {
    public:
        /**
         * @object Instantiate a FlagIterable object.
         * @param flags Thread flags bit vector.
         */
        explicit constexpr FlagIterable(
            uint32_t flag_bv, const FlagList& flag_list) noexcept
            : _flag_bv { flag_bv }
            , _flag_list { flag_list }
        {
        }

        /**
         * Get an iterator of the set flags at the beginning.
         */
        constexpr FlagIterator begin() const noexcept
        {
            if (not _flags_have_error_code(_flag_bv)) {
                auto cbegin = std::cbegin(_flag_list);
                auto cend = std::cend(_flag_list);
                return FlagIterator { _flag_bv, cbegin, cend };
            } else {
                return end();
            }
        }

        /**
         * Get an iterator of the set flags at the end.
         */
        constexpr FlagIterator end() const noexcept
        {
            auto cend = std::cend(_flag_list);
            return FlagIterator { _flag_bv, cend, cend };
        }

    private:
        // Maintain the thread flags bit vector.
        const uint32_t _flag_bv;
        const FlagList& _flag_list;
    };

    /**
     * @brief Convert a thread flags vector to an iterable.
     * @param flags Thread flags vector to convert.
     * @return A FlagIterable object containing the given flags.
     */
    auto _iterate_flags(uint32_t flags) noexcept
    {
        return FlagIterable { flags, _flags };
    }
};

/**
 * @brief CMSIS-OS static thread class definition.
 * @param SIZE Size of stack for underlying thread.
 *
 * A StaticThread differs from its parent Thread in that it provides an
 * allocation for the control blocks and buffers and further provides an init()
 * method.
 */
template <size_t SIZE> class StaticThread : public Thread {
public:
    /**
     * @brief Instantiate a CMSIS-OS static thread object.
     * @param priority OS priority of the thread.
     */
    explicit StaticThread(osPriority_t priority)
        : Thread()
        , _name(nullptr)
        , _priority(priority)
    {
    }

    /**
     * @brief Instantiate a CMSIS-OS message queue object.
     * @param name     Name of the thread.
     * @param priority OS priority of the thread.
     */
    explicit StaticThread(const char* name, osPriority_t priority)
        : Thread()
        , _name(name)
        , _priority(priority)
    {
    }

    /**
     * @brief Instantiate a CMSIS-OS message queue object.
     * @param func       Target function for thread to call.
     * @param parameters Void pointer of parameters to pass to target function.
     */
    void init(osThreadFunc_t func, void* parameters = nullptr)
    {
        auto handle = osThreadNew(func, parameters, &_attrs);
        set_handle(handle);
    }

private:
    // Maintain a pointer to the name of the thread (if any).
    const char* _name;

    // Maintain the os priority of the thread.
    const osPriority_t _priority;

    // Allocate space for underlying control block and buffer.
    StaticTask_t _control_block;
    uint32_t _buffer[SIZE] = {};

    // Create attributes struct for static allocation.
    const osThreadAttr_t _attrs = {
        .name = _name,
        .attr_bits = 0,
        .cb_mem = &_control_block,
        .cb_size = sizeof(_control_block),
        .stack_mem = &_buffer[0],
        .stack_size = sizeof(_buffer),
        .priority = _priority,
        .tz_module = 0,
        .reserved = 0,
    };
};

/**
 * @brief CMSIS-OS timer class definition.
 */
class Timer {
public:
    /**
     * @brief Signature type for allowed callback functions.
     */
    typedef void (*TimerCallback_t)(void* parameters);

    /**
     * @brief Instantiate a Timer instance.
     * @param handle Reference to CMSIS-OS timer handle.
     */
    explicit Timer(osTimerId_t handle = nullptr)
        : _handle(handle)
    {
    }

    /**
     * @brief Destruct an CMSIS-OS timer object.
     */
    virtual ~Timer(void) = default;

    /**
     * @brief Start the timer.
     * @param  ticks Ticks to wait before the timer callback.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t start(uint32_t ticks) { return osTimerStart(_handle, ticks); }

    /**
     * @brief Stop the timer.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t stop(void) { return osTimerStop(_handle); }

    /**
     * @brief Restart the timer.
     *
     * This simply stops and then starts the timer, successively.
     *
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t restart(uint32_t ticks)
    {
        stop();
        return start(ticks);
    }

    /**
     * @brief Get the CMSIS-OS timer handle.
     * @return The CMSIS-OS timer handle.
     */
    osTimerId_t get_handle(void) { return _handle; }

    /**
     * @brief Set the CMSIS-OS timer handle.
     * @param handle The CMSIS-OS timer handle.
     */
    void set_handle(osTimerId_t handle) { _handle = handle; }

private:
    // Store a reference to the timer handle.
    osTimerId_t _handle;
};

/**
 * @brief CMSIS-OS static timer class definition.
 *
 * A StaticTimer differs from its parent Timer in that it provides an
 * allocation for the control blocks and buffers and further provides init()
 * and deinit() methods.
 */
class StaticTimer : public Timer {
public:
    explicit StaticTimer(
        const char* name, TimerCallback_t callback, void* parameters = nullptr)
        : Timer()
        , _name(name)
        , _callback(callback)
        , _parameters(parameters)
    {
    }

    /**
     * @brief Instantiate a CMSIS-OS static timer object.
     * @param callback Callback function to call on timeout.
     * @param parameters Void pointer to parameters to pass to callback.
     */
    explicit StaticTimer(TimerCallback_t callback, void* parameters = nullptr)
        : StaticTimer(nullptr, callback, parameters)
    {
    }

    /**
     * @brief Destruct a CMSIS-OS static timer object.
     */
    ~StaticTimer(void) { deinit(); }

    /**
     * @brief Initialize a CMSIS-OS static timer object.
     */
    void init(osTimerType_t timer_type)
    {
        if (_initialized) {
            return;
        }

        _initialized = true;
        const auto handle = osTimerNew(_callback, timer_type, _parameters, &_attrs);
        set_handle(handle);
    }

    /**
     * @brief Deinitialize a CMSIS-OS static timer object.
     * @return osOK if successful; otherwise, an osStatus_t error value.
     */
    osStatus_t deinit(void)
    {
        if (not _initialized) {
            return osError;
        }

        _initialized = false;
        const auto handle = get_handle();
        return osTimerDelete(handle);
    }

private:
    // Maintain a pointer to the name of the thread (if any).
    const char* _name;

    // Maintain a pointer to the callback function.
    TimerCallback_t const _callback;

    // Maintain a pointer to the callback parameters.
    void* const _parameters;

    // Maintain whether this object has been initialized or not.
    bool _initialized = false;

    // Allocate space for underlying control block and buffer.
    StaticTimer_t _control_block;

    // Create attributes struct for static allocation.
    const osTimerAttr_t _attrs = {
        .name = _name,
        .attr_bits = 0,
        .cb_mem = &_control_block,
        .cb_size = sizeof(_control_block),
    };
};
}
