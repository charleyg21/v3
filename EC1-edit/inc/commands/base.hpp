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
 * @author Jonathan Starr <jonstarr@utexas.edu>
 * @date May 13th, 2021
 *
 * @brief Base template definitions for task command system.
 */

#pragma once

#include <functional>
#include <tuple>

#include "defines/status.hpp"
#include "defines/warnings.hpp"
#include "helpers/cmsis_os.hpp"

namespace commands {

using Status = defines::Status;
using Warning = warnings::Warning;
using WarningLevel = warnings::WarningLevel;

template <auto command> struct Signature;
template <auto command> using ReturnValue = typename Signature<command>::ReturnValue;
template <auto command> using Parameters = typename Signature<command>::Parameters;

template <auto command, typename ReturnValue = ReturnValue<command>>
using Future = typename cmsis_os::StaticMessageQueue<ReturnValue, 1>;

template <auto command, typename Signature = Signature<command>,
    typename Parameters = typename Signature::Parameters,
    typename Future = Future<command>>
using QueueItem = std::tuple<Parameters, Future*>;

template <auto command, std::size_t size>
using Queue = cmsis_os::StaticMessageQueue<QueueItem<command>, size>;

template <auto command>
osStatus_t queue_put(const QueueItem<command>& item, uint32_t timeout);

template <auto command> void set_thread_flag(void);

template <auto command, typename Signature = Signature<command>,
    typename Parameters = typename Signature::Parameters,
    typename ReturnValue = typename Signature::ReturnValue,
    typename Future = Future<command>>
auto send(const Parameters& parameters, Future& future, uint32_t timeout = 0)
{
    const auto queue_item = QueueItem<command> { parameters, &future };
    const auto status = queue_put<command>(queue_item, timeout);
    if (defines::status::is_ok(status)) {
        set_thread_flag<command>();
    }
    return defines::status::map_from(status);
}

template <auto command, typename Signature = Signature<command>,
    typename Parameters = typename Signature::Parameters,
    typename ReturnValue = typename Signature::ReturnValue,
    typename Future = Future<command>>
auto send(const Parameters& parameters, uint32_t timeout = 0)
{
    const auto queue_item = QueueItem<command> { parameters, nullptr };
    const auto status = queue_put<command>(queue_item, timeout);
    if (defines::status::is_ok(status)) {
        set_thread_flag<command>();
    }
    return defines::status::map_from(status);
}

template <auto command, typename Signature = Signature<command>,
    typename Parameters = typename Signature::Parameters,
    typename ReturnValue = typename Signature::ReturnValue,
    typename Future = Future<command>>
auto send_and_block(
    const Parameters& parameters, ReturnValue& return_value, uint32_t timeout = 0)
{
    Future future;
    future.init();

    const auto send_status = send<command>(parameters, future, timeout);
    if (not defines::status::is_ok(send_status)) {
        return defines::status::map_from(send_status);
    }

    const auto get_status = future.get(return_value, osWaitForever);
    if (not defines::status::is_ok(get_status)) {
        return defines::status::map_from(get_status);
    }

    return Status::OK;
}

template <auto command, typename Queue, typename QueueItem = QueueItem<command>,
    typename Future = Future<command>, typename Signature = Signature<command>,
    typename ReturnValue = typename Signature::ReturnValue>
auto flush_queue(Queue& queue, const ReturnValue& return_value)
{
    auto count = queue.get_count();
    for (auto index = decltype(count) { 0 }; index < count; index++) {
        QueueItem item {};
        const auto get_status = queue.get(item, 0);
        if (not defines::status::is_ok(get_status)) {
            continue;
        }

        auto [_, future] = item;
        if (future) {
            future->put(return_value, 0);
        }
    }
    return count;
}

template <auto command, typename Queue, typename QueueItem = QueueItem<command>,
    typename Future = Future<command>, typename Signature = Signature<command>,
    typename ReturnValue = typename Signature::ReturnValue,
    typename Parameters = typename Signature::Parameters,
    typename Handler = typename std::function<ReturnValue(Parameters&)>>
auto handle_queue(Queue& queue, Handler handler)
{
    auto count = queue.get_count();
    for (auto index = decltype(count) { 0 }; index < count; index++) {
        QueueItem item {};
        const auto get_status = queue.get(item, 0);
        if (not defines::status::is_ok(get_status)) {
            continue;
        }

        auto [params, future] = item;
        const auto return_value = handler(params);

        if (future) {
            future->put(return_value, 0);
        }
    }
    return count;
}

} // namespace commands
