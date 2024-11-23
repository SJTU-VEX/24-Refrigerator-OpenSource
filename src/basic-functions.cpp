/**
 * @file basic-functions.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "basic-functions.h"

#include <math.h>

#include "PID.h"
#include "geometry.h"
#include "parameters.h"
#include "vex.h"

using namespace vex;
using namespace std;

// _input ranges from -100 : 100
// powers all motors on left side of base with duty cycle _input%
void moveLeft(double _input) {
    if (abs(_input) > 100) {
        _input = sign(_input) * 100;
    }
    MotorBase_L1.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
    MotorBase_L2.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
    MotorBase_L3.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
    MotorBase_L4.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
}

// _input ranges from -100 : 100
// powers all motors on right side of base with duty cycle _input%
void moveRight(double _input) {
    if (abs(_input) > 100) {
        _input = sign(_input) * 100;
    }
    MotorBase_R1.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
    MotorBase_R2.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
    MotorBase_R3.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
    MotorBase_R4.spin(directionType::fwd, 127 * _input, voltageUnits::mV);
}

void unfold_left() {
    Piston_left_unfold.set(true);
    Piston_left_fold.set(false);
}

void fold_left() {
    Piston_left_fold.set(true);
    Piston_left_unfold.set(false);
}

void unfold_right() {
    Piston_right_unfold.set(true);
    Piston_right_fold.set(false);
}

void fold_right() {
    Piston_right_fold.set(true);
    Piston_right_unfold.set(false);
}

double getForwardVel() {
    double l1 = MotorBase_L1.velocity(rpm);
    double l2 = MotorBase_L2.velocity(rpm);
    double l3 = MotorBase_L3.velocity(rpm);
    double l4 = MotorBase_L4.velocity(rpm);
    double r1 = MotorBase_R1.velocity(rpm);
    double r2 = MotorBase_R2.velocity(rpm);
    double r3 = MotorBase_R3.velocity(rpm);
    double r4 = MotorBase_R4.velocity(rpm);
    return (l1 + l2 + l3 + l4 + r1 + r2 + r3 + r4) / 8;
}
double getRotationVel() {
    double l1 = MotorBase_L1.velocity(rpm);
    double l2 = MotorBase_L2.velocity(rpm);
    double l3 = MotorBase_L3.velocity(rpm);
    double l4 = MotorBase_L4.velocity(rpm);
    double r1 = MotorBase_R1.velocity(rpm);
    double r2 = MotorBase_R2.velocity(rpm);
    double r3 = MotorBase_R3.velocity(rpm);
    double r4 = MotorBase_R4.velocity(rpm);
    return (l1 + l2 + l3 + l4 - r1 - r2 - r3 - r4) / 4;
}

double IMUHeading() {
    double heading = Inertial.rotation(rotationUnits::deg);
    heading = heading / IMU_MOD_COEFFICIENT * 3600;
    while (heading < 0) heading += 360;
    while (heading >= 360) heading -= 360;
    return heading;
}

void resetHeading() { Inertial.resetRotation(); }

void clearBrainScr() { Brain.Screen.clearScreen(); }

void clearControllerScr() {
    Controller.Screen.setCursor(5, 1);
    Controller.Screen.print("                                                             ");
}

void flickBall(flickSide side, int num) {
    switch (side) {
        case FLICKLEFT:
            for (int i = 0; i < num; i++) {
                unfold_left();
                this_thread::sleep_for(1500);
                fold_left();
                this_thread::sleep_for(500);
            }
            break;
        case FLICKRIGHT:
            for (int i = 0; i < num; i++) {
                unfold_right();
                this_thread::sleep_for(1500);
                fold_right();
                this_thread::sleep_for(500);
            }
            break;
    }
}