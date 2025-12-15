/**
 * @file
 *
 * Declarations for the hidden menu code
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <functional>

namespace ui {

/**
 * Possible inputs to advance towards the hidden menu
 */
enum class Input {
    UP,
    DOWN
};

/**
 * A node for the hidden menu code
 */
struct Code {
    /**
     * The value the input must match to advance to the next node
     * 0 = up, 1 = down
     */
    Input match;

    /**
     * The next node to advance to if the input matches
     */
    const Code* next;

    /**
     * The fail node to advance to if the input does not match
     */
    const Code* fail;

    /**
     * Advance the node
     */
    const Code* advance(Input input) const {
        return (input == match) ? next : fail;
    }
};

}