/**
 * @file chassis.h
 * @brief A file containing a class to control the robot's chassis, including forward and rotational velocities.
 * 
 * Providing methods to set and control the forward and rotational velocities
 * of the robot, both manually and automatically. It also includes methods to handle
 * braking and to update the chassis state.
 *
 * @copyright 2024 SJTU VEX
 */

#ifndef CHASSIS_H
#define CHASSIS_H

#include <vex.h>

class Chassis {
private:
    // forward velocity (pct)
    double robotFWD;

    // rotate velocity clockwise (pct)
    double robotROT;

    // manual velocity part
    double manualFWD;
    double manualROT;

    // auto velocity part
    double autoFWD;
    double autoROT;

    vex::brakeType stopBrakeType;

    void setMotorPower();

    // wheel velocity (pct) [left, right]
    double wheelVel[2];

public:
    // singleton pattern
    static Chassis *getInstance() {
        static Chassis *c = nullptr;
        if (c == nullptr) {
            c = new Chassis();
        }
        return c;
    }

    static void deleteInstance() {
        Chassis *c = Chassis::getInstance();
        if (c != nullptr) {
            delete c;
            c = nullptr;
        }
    }

    Chassis();
    void autoSetForwardVel(double vel);
    void manualSetForwardVel(double vel);
    void autoSetRotateVel(double vel);
    void manualSetRotateVel(double vel);
    void chassisRun();
    void chassisBrake(vex::brakeType brake);
    void setStopBrakeType(vex::brakeType brake);
};

/// @brief chassis update thread function
void updateChassis();

#endif