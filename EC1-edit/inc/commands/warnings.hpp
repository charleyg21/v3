/**
 * @file
 *
 * Warning header
 */

#pragma once

#include <tuple>

#include "defines/warnings.hpp"
#include "defines/status.hpp"

namespace warnings {
    extern WarningLevel warning_level;

    void set_warning(WarningLevel new_warning_level, Warning new_warning);
    uint8_t get_warnings(void);
    Warning get_active_warning(void);
    WarningLevel get_warning_level(void);
    WarningLevel _warning_level_from_warnings(uint8_t warnings);
    bool _is_warning(uint8_t warnings, Warning warning);
    bool _is_any_warning(uint8_t warnings);
    void reset_warnings(void);
    void reset_warning(Warning warning);

    void ui_warning(WarningLevel level);
}