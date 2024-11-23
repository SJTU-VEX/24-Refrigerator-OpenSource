/**
 * @file main.cpp
 * @brief Main file for the VEX Robotics project.
 *
 * This file contains the main function that initializes the robot and runs the
 * autonomous and user control functions.
 *
 * @copyright 2024 SJTU VEX
 */

#include "autonomous.h"
#include "basic-functions.h"
#include "chassis.h"
#include "controller.h"
#include "cstdlib"
#include "debugger.h"
#include "iostream"
#include "my-timer.h"
#include "parameters.h"
#include "position.h"
#include "usercontrol.h"
#include "vex.h"

using namespace vex;
using namespace std;

#ifdef COMPETITION
competition Competition;
#endif

int main() {
    Inertial.calibrate();
    waitUntil(!Inertial.isCalibrating());
    Controller.Screen.setCursor(5, 1);
    Controller.Screen.print("        calibrated!");

    thread Tchassis(updateChassis);
    thread Tposition(updatePosition);
    thread Tcontroller(defineController);

    fold_left();
    fold_right();

#ifdef COMPETITION
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
#endif

    Controller.Screen.setCursor(5, 9);
    Controller.Screen.print("            ");
    Controller.Screen.setCursor(3, 1);
    Controller.Screen.print("PITCH: ON");

    while (true) {
        clearBrainScr();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("IMUHeading: %.4f", IMUHeading());

        this_thread::sleep_for(REFRESH_TIME);
    }
}
