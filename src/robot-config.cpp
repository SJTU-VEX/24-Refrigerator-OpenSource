/**
 * @file robot-config.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;
controller Controller = controller(primary);

inertial Inertial = inertial(PORT11);

// BaseMotor
// HEAD-12-34-TAIL
motor MotorBase_L1 = motor(PORT4, ratio6_1, true);
motor MotorBase_L2 = motor(PORT3, ratio6_1, false);
motor MotorBase_L3 = motor(PORT2, ratio6_1, false);
motor MotorBase_L4 = motor(PORT1, ratio6_1, true);
motor MotorBase_R1 = motor(PORT9, ratio6_1, false);
motor MotorBase_R2 = motor(PORT8, ratio6_1, true);
motor MotorBase_R3 = motor(PORT6, ratio6_1, true);
motor MotorBase_R4 = motor(PORT5, ratio6_1, false);

// OtherMotor
motor Intaker = motor(PORT14, ratio6_1, false);

motor Elevator_L = motor(PORT16, ratio6_1, true);
motor Elevator_R = motor(PORT19, ratio6_1, false);

// Piston
digital_out Piston_left_unfold = digital_out(Brain.ThreeWirePort.C);
digital_out Piston_right_unfold = digital_out(Brain.ThreeWirePort.G);
digital_out Piston_left_fold = digital_out(Brain.ThreeWirePort.B);
digital_out Piston_right_fold = digital_out(Brain.ThreeWirePort.H);

// GPS
gps GPS = gps(PORT16);