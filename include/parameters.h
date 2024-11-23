/**
 * @file parameters.h
 * @brief This file contains the parameter definitions for the robot configuration.
 * 
 * @copyright 2024 SJTU VEX
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "robot-config.h"
#include "vex.h"

// Refreshing time of thread
const int REFRESH_TIME = 10;  // ms

// Inertial sensor correction coefficient, obtained by rotate robot 3600 deg and read
// the origin data got from `Inertial.rotation`.
const double IMU_MOD_COEFFICIENT = 3580.23;

const int PID_Exit_time = 1500;  // ms

// The distance traveled by the wheel when its motor rotates 1 radian
const double WHEEL_TRANSITION_COEFFICIENT = 5.27;  // cm/rad

// Elevator parameters obtained in test
const double ELEVATOR_LEFT_TOP_HEIGHT_DEG = 2150;  // deg

#endif