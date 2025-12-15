/**
 * @file
 *
 * Warning definitions
 */

#pragma once

namespace warnings {
    /**
     * @brief Enumeration for warning levels
     */
    enum WarningLevel {
        None,
        Normal,
        High,
        Critical
    };

    /**
     * @brief Enumeration for warnings
     */
    enum Warning {
        Overflow,
        Rotary_Homing,
        Syringe_Homing,
        High_Temp,
        Low_Temp,
        Low_Battery
    };
}