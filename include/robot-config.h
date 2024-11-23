/**
 * @file robot-config.h
 * @brief Original VEXcode header. Defining basic devices.
 *
 * @copyright 2024 SJTU VEX
 */

#include "vex.h"
using namespace vex;

extern brain Brain;
extern controller Controller;

extern inertial Inertial;

// BaseMotor
//   1
// 2 3 4
extern motor MotorBase_L1;
extern motor MotorBase_L2;
extern motor MotorBase_L3;
extern motor MotorBase_L4;
extern motor MotorBase_R1;
extern motor MotorBase_R2;
extern motor MotorBase_R3;
extern motor MotorBase_R4;

// OtherMotor
extern motor Intaker;

extern motor Elevator_L;
extern motor Elevator_R;

// Piston
extern digital_out Piston_left_fold;
extern digital_out Piston_right_fold;
extern digital_out Piston_left_unfold;
extern digital_out Piston_right_unfold;
