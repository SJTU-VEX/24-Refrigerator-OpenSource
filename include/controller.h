/**
 * @file controller.h
 * @brief Header file for defining controller button mappings and input handling.
 *
 * This file contains the declarations for the controller button mappings and
 * the function to update the controller inputs.
 *
 * @copyright 2024 SJTU VEX
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

// Define the buttons
extern int t, A1, A2, A3, A4, L1, L2, R1, R2, X, Y, A, B, LEFT, RIGHT, UP, DOWN, last_L1, last_L2, last_R1, last_R2,
    last_X, last_Y, last_A, last_B, last_LEFT, last_RIGHT, last_UP, last_DOWN;

extern bool press_X, press_Y, press_A, press_B, press_UP, press_DOWN, press_LEFT, press_RIGHT, press_L1, press_L2;

/// @brief Update the controller inputs as a thread function
void defineController();

#endif