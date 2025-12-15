/**
 * @file
 *
 * Declarations for modeling a hierarchical menu.
 */

#pragma once

#include <cstdint>
#include <functional>

#include "drivers/display.hpp"

namespace ui {

/**
 * Control holding awake
 */
void start_holding_awake();
void end_holding_awake();
void resume_menu();

/**
 * Model a node within a menu hierarchy.
 */
struct Menu {
    /**
     * Data tag
     */
    const unsigned int tag;

    /**
     * Text to display on first line of LCD .
     */
    const char* line1;

    /**
     * Text to display on second line of LCD .
     */
    const char* line2;

    /**
     * Menu node above current one.
     */
    Menu* parent;

    /**
     * Menu node below current one.
     */
    Menu* child;

    /**
     * Next menu node on same level of menu hierarchy.
     */
    Menu* up;

    /**
     * Previous menu node on same level of menu hierarchy.
     */
    Menu* down;

    /**
     * Move to next menu node on same level of menu hierarchy.
     */
    const Menu* move_up(void) const { return (up) ? up : this; }

    /**
     * Move to previous menu node on same level of menu hierarchy.
     */
    const Menu* move_down(void) const { return (down) ? down : this; }

    using Display = drivers::Display;

    /**
     * Select this menu node; call callback if given and move to child node.
     */
    const Menu* select(void) const
    {
        if (callback) ui::start_holding_awake();
        const auto result = (callback) ? callback(this) : nullptr;
        if (callback) ui::end_holding_awake();
        return (result) ? result : ((child) ? child : this);
    }

    const std::function<const Menu*(const Menu*)> callback;
};

/**
 * Forward declaration of main menu node.
 */
extern const Menu* main_menu;

}