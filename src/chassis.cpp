/**
 * @file chassis.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "chassis.h"

#include <math.h>

#include "basic-functions.h"
#include "geometry.h"
#include "parameters.h"
#include "robot-config.h"

using std::abs;

Chassis::Chassis() {
    robotFWD = 0;
    robotROT = 0;
    manualFWD = 0;
    manualROT = 0;
    autoFWD = 0;
    autoROT = 0;
    wheelVel[0] = 0;
    wheelVel[1] = 0;
    stopBrakeType = vex::coast;
}

void Chassis::setMotorPower() {
    if (wheelVel[0] == 0) {
        MotorBase_L1.stop(stopBrakeType);
        MotorBase_L2.stop(stopBrakeType);
        MotorBase_L3.stop(stopBrakeType);
        MotorBase_L4.stop(stopBrakeType);
    } else {
        moveLeft(wheelVel[0]);
    }
    if (wheelVel[1] == 0) {
        MotorBase_R1.stop(stopBrakeType);
        MotorBase_R2.stop(stopBrakeType);
        MotorBase_R3.stop(stopBrakeType);
        MotorBase_R4.stop(stopBrakeType);
    } else {
        moveRight(wheelVel[1]);
    }
}

void Chassis::chassisBrake(vex::brakeType brake) {
    robotFWD = 0;
    robotROT = 0;
    manualFWD = 0;
    manualROT = 0;
    autoFWD = 0;
    autoROT = 0;
    wheelVel[0] = 0;
    wheelVel[1] = 0;

    MotorBase_L1.stop(brakeType::brake);
    MotorBase_R1.stop(brakeType::brake);
    MotorBase_L2.stop(brakeType::brake);
    MotorBase_R2.stop(brakeType::brake);
    MotorBase_L3.stop(brakeType::brake);
    MotorBase_R3.stop(brakeType::brake);
    MotorBase_L4.stop(brakeType::brake);
    MotorBase_R4.stop(brakeType::brake);
}

void Chassis::setStopBrakeType(brakeType brake) { stopBrakeType = brake; }

/// @brief chassis refresh thread function
void Chassis::chassisRun() { setMotorPower(); }

void updateChassis() {
    while (true) {
        Chassis::getInstance()->chassisRun();
        this_thread::sleep_for(REFRESH_TIME);
    }
}

void Chassis::manualSetForwardVel(double vel) {
    if (abs(vel) > 100) {
        vel = sign(vel) * 100;
    }
    manualFWD = vel;
    robotFWD = manualFWD + autoFWD;
    double left = robotFWD + robotROT;
    double right = robotFWD - robotROT;
    double k;
    if (abs(left) > 100) {
        k = 100 / abs(left);
        left = sign(left) * 100;
        right = right * k;
    }
    if (abs(right) > 100) {
        k = 100 / abs(right);
        right = sign(right) * 100;
        left = left * k;
    }
    wheelVel[0] = left;
    wheelVel[1] = right;
}

void Chassis::autoSetForwardVel(double vel) {
    if (abs(vel) > 100) {
        vel = sign(vel) * 100;
    }
    autoFWD = vel;
    robotFWD = manualFWD + autoFWD;
    double left = robotFWD + robotROT;
    double right = robotFWD - robotROT;
    double k;
    if (abs(left) > 100) {
        k = 100 / abs(left);
        left = sign(left) * 100;
        right = right * k;
    }
    if (abs(right) > 100) {
        k = 100 / abs(right);
        right = sign(right) * 100;
        left = left * k;
    }
    wheelVel[0] = left;
    wheelVel[1] = right;
}

void Chassis::manualSetRotateVel(double vel) {
    if (abs(vel) > 100) {
        vel = sign(vel) * 100;
    }
    manualROT = vel;
    robotROT = manualROT + autoROT;
    double left = robotFWD + robotROT;
    double right = robotFWD - robotROT;
    double k;
    if (abs(left) > 100) {
        k = 100 / abs(left);
        left = sign(left) * 100;
        right = right * k;
    }
    if (abs(right) > 100) {
        k = 100 / abs(right);
        right = sign(right) * 100;
        left = left * k;
    }
    wheelVel[0] = left;
    wheelVel[1] = right;
}

void Chassis::autoSetRotateVel(double vel) {
    if (abs(vel) > 100) {
        vel = sign(vel) * 100;
    }
    autoROT = vel;
    robotROT = manualROT + autoROT;
    double left = robotFWD + robotROT;
    double right = robotFWD - robotROT;
    double k;
    if (abs(left) > 100) {
        k = 100 / abs(left);
        left = sign(left) * 100;
        right = right * k;
    }
    if (abs(right) > 100) {
        k = 100 / abs(right);
        right = sign(right) * 100;
        left = left * k;
    }
    wheelVel[0] = left;
    wheelVel[1] = right;
}
