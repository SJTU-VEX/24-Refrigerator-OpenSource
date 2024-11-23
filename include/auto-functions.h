/**
 * @file auto-functions.h
 * @brief Header file containing function declarations for autonomous robot movements.
 *
 * This file includes function declarations for various autonomous movements
 * such as moving forward for a specified duration, moving forward with rotation,
 * moving forward for a specified distance, and turning to a specific heading.
 * It also includes functions for moving to specific positions.
 * 
 * @copyright 2024 SJTU VEX
 */

#ifndef AUTO_FUNCTIONS_H_
#define AUTO_FUNCTIONS_H_

#include "geometry.h"
#include "vex.h"

/// @param power pct
/// @param duration msec
void timerForward(double power, int duration);

/// @param forwardPower pct
/// @param rotatePower pct  positive for clockwise
/// @param duration msec
void timerForwardWithRotate(double forwardPower, double rotatePower, int duration);

/// @param power pct
/// @param distance unit
void distanceForward(double power, double distance);

/// @param power pct
/// @param distance cm
void distanceForwardCM(double power, double distance);

/// @param power max power
/// @param target heading deg
void turnTo(double power, double target);

/// @param power max power
/// @param target heading deg
void turnPreciseTo(double power, double target);

void forwardToPos(double power, Point tar);
void forwardToPos(double power, double x, double y);
void backToPos(double power, Point tar);
void backToPos(double power, double x, double y);

#endif