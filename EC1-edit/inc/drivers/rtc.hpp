/**
 * @file
 *
 * Driver class for real-time clock.
 */

#pragma once

#include <cstdint>
#include <tuple>

#include "main.h"

#include "defines/status.hpp"
#include "helpers/cmsis_os.hpp"

namespace drivers {

using Status = defines::Status;

class RealTimeClock {
public:
    static constexpr auto NUM_RTC_BACKUP_REGISTERS = static_cast<std::size_t>(5);

    explicit RealTimeClock(RTC_HandleTypeDef* hrtc)
        : _hrtc(hrtc)
    {
    }

    std::tuple<Status, uint8_t, uint8_t> get_time(void)
    {
        RTC_TimeTypeDef time = {};
        const auto hal_status = HAL_RTC_GetTime(_hrtc, &time, RTC_FORMAT_BIN);
        const auto status = defines::status::map_from(hal_status);
        return std::make_tuple(status, time.Hours, time.Minutes);
    }

    Status set_time(uint8_t hours, uint8_t minutes)
    {
        RTC_TimeTypeDef time = {
            .Hours = hours,
            .Minutes = minutes,
            .Seconds = 0,
            .TimeFormat = 0,
            .SubSeconds = 0,
            .SecondFraction = 0,
            .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
            .StoreOperation = RTC_STOREOPERATION_RESET,
        };
        const auto hal_status = HAL_RTC_SetTime(_hrtc, &time, RTC_FORMAT_BIN);
        return defines::status::map_from(hal_status);
    }

    std::tuple<Status, uint8_t, uint8_t, uint8_t> get_date(void)
    {
        RTC_DateTypeDef date = {};
        const auto hal_status = HAL_RTC_GetDate(_hrtc, &date, RTC_FORMAT_BIN);
        if (defines::status::is_ok(hal_status)) { }
        const auto status = defines::status::map_from(hal_status);
        return std::make_tuple(status, date.Year, date.Month, date.Date);
    }

    Status set_date(uint8_t year, uint8_t month, uint8_t day)
    {
        RTC_DateTypeDef date = {
            .WeekDay = 0,
            .Month = month,
            .Date = day,
            .Year = year,
        };
        const auto hal_status = HAL_RTC_SetDate(_hrtc, &date, RTC_FORMAT_BIN);
        return defines::status::map_from(hal_status);
    }

    Status set_alarm_a(uint8_t hours, uint8_t minutes)
    {
        RTC_AlarmTypeDef alarm = {
            .AlarmTime = {
               .Hours = hours,
               .Minutes = minutes,
               .Seconds = 0,
               .TimeFormat = 0,
               .SubSeconds = 0,
               .SecondFraction = 0,
               .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
               .StoreOperation = RTC_STOREOPERATION_RESET,
            },
            .AlarmMask = RTC_ALARMMASK_DATEWEEKDAY,
            .AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL,
            .AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE,
            .AlarmDateWeekDay = 0,
            .Alarm = RTC_ALARM_A,
        };
        const auto hal_status = HAL_RTC_SetAlarm_IT(_hrtc, &alarm, RTC_FORMAT_BIN);
        return defines::status::map_from(hal_status);
    }

    std::tuple<Status, uint8_t, uint8_t> get_alarm_a(void)
    {
        RTC_AlarmTypeDef alarm = {};
        const auto hal_status
            = HAL_RTC_GetAlarm(_hrtc, &alarm, RTC_ALARM_A, RTC_FORMAT_BIN);
        const auto alarm_time = alarm.AlarmTime;
        const auto status = defines::status::map_from(hal_status);
        return std::make_tuple(status, alarm_time.Hours, alarm_time.Minutes);
    }

    Status deactivate_alarm_a(void)
    {
        const auto hal_status = HAL_RTC_DeactivateAlarm(_hrtc, RTC_ALARM_A);
        return defines::status::map_from(hal_status);
    }

    void write_backup(std::size_t index, uint32_t data)
    {
        HAL_RTCEx_BKUPWrite(_hrtc, index, data);
    }

    uint32_t read_backup(std::size_t index) { return HAL_RTCEx_BKUPRead(_hrtc, index); }

private:
    RTC_HandleTypeDef* _hrtc;
};

} // namespace drivers
