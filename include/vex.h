/**
 * @file vex.h
 * @brief Original VEXcode header. Defining competition settings.
 *
 * This file includes necessary VEX Robotics headers and defines macros for
 * common operations and configurations.
 *
 * @copyright 2024 SJTU VEX
 */

#include "v5.h"
#include "v5_vcs.h"

#define waitUntil(condition) \
    do {                     \
        wait(5, msec);       \
    } while (!(condition))

#define repeat(iterations) for (int iterator = 0; iterator < iterations; iterator++)

// ----------define robot color---------
#define RED_ALLIANCE
// #define BLUE_ALLIANCE

// ----------define competition---------
#define COMPETITION

// ----------define skill---------------
// #define SKILL