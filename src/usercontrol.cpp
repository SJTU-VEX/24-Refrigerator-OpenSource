/**
 * @file usercontrol.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "usercontrol.h"

#include <cmath>
#include <iostream>

#include "basic-functions.h"
#include "chassis.h"
#include "controller.h"
#include "parameters.h"
#include "vex.h"

using namespace std;
using namespace vex;

bool Fold_L = true;
bool Fold_R = true;
static bool Pitch_control = true;

void baseControl() {
    double a3, a1;  // a3 fwd, a1 rot
    a3 = (abs(A3) < 10) ? 0 : A3;
    a1 = (abs(A1) < 10) ? 0 : A1;
    // roll default: 7
    if (Pitch_control)
        if (Inertial.roll() <= -25) {
            if (a3 > 45) a3 = 45;
        } else if (Inertial.roll() >= 5) {
            if (a3 < -45) a3 = -45;
        }
    Chassis::getInstance()->manualSetForwardVel(a3);
    Chassis::getInstance()->manualSetRotateVel(a1);
}

void pitchControl() {
    if (press_RIGHT) {
        press_RIGHT = false;
        Pitch_control = !Pitch_control;
        Controller.Screen.setCursor(3, 1);
        if (Pitch_control)
            Controller.Screen.print("PITCH: ON ");
        else
            Controller.Screen.print("PITCH: OFF");
    }
}

void foldSafety() {
    double v = getForwardVel();
    double r = getRotationVel();
    double mylimit = 400;
    if (v >= mylimit || v <= -mylimit || r >= 2 * mylimit || r <= -2 * mylimit) {
        fold_left();
        fold_right();
    }
}

void elevatorInit() {
    Elevator_R.setBrake(coast);
    Elevator_L.setBrake(coast);
    Elevator_L.spinToPosition(ELEVATOR_LEFT_TOP_HEIGHT_DEG, deg, 100, velocityUnits::pct, true);
    Elevator_R.setBrake(hold);
    Elevator_L.setBrake(hold);
    eleavtorReady = true;
}

void elevatorControl() {
    if (eleavtorReady) {
        if (UP) {
            Elevator_L.spin(fwd, 100, pct);
            Elevator_R.spin(fwd, 100, pct);
        } else if (DOWN) {
            Elevator_L.spin(fwd, -100, pct);
            Elevator_R.spin(fwd, -100, pct);
        } else {
            Elevator_L.stop(hold);
            Elevator_R.stop(hold);
        }
    } else {
        if (DOWN && B) {
            static thread TElevator = thread(elevatorInit);
            Controller.rumble("--");
        }
    }
}

void intakerControl() {
    if (R1) {
        Intaker.spin(fwd, -100, pct);
    } else if (R2) {
        Intaker.spin(fwd, 100, pct);
    } else {
        Intaker.stop(coast);
    }
}
void foldControl() {
    if (press_L1) {
        press_L1 = false;
        Fold_L = !Fold_L;
        if (Fold_L)
            fold_left();
        else
            unfold_left();
    } else if (press_L2) {
        press_L2 = false;
        Fold_R = !Fold_R;
        if (Fold_R)
            fold_right();
        else
            unfold_right();
    }
}

void momentumWheelControl() {
    if (press_A) {
        press_A = false;
        double speed_number = 0.75;
        int iteration = 0;
        while (abs(A1) <= 90 && abs(A3) <= 90 && iteration < 10) {
            iteration++;
            if (Inertial.roll() < -11) {
                for (double i = 0; i <= 100; i += speed_number) {
                    Chassis::getInstance()->autoSetForwardVel(i);
                    this_thread::sleep_for(1);
                }
                Chassis::getInstance()->autoSetForwardVel(-100);
                // while (getForwardVel() >= -600) {
                //     this_thread::sleep_for(1);
                // }
                this_thread::sleep_for(50);
                for (double i = -100; i <= 0; i += speed_number) {
                    Chassis::getInstance()->autoSetForwardVel(i);
                    this_thread::sleep_for(1);
                }
            } else if (Inertial.roll() > -9) {
                for (double i = 0; i >= -100; i -= speed_number) {
                    Chassis::getInstance()->autoSetForwardVel(i);
                    this_thread::sleep_for(1);
                }
                Chassis::getInstance()->autoSetForwardVel(100);
                // while (getForwardVel() <= 600) {
                //     this_thread::sleep_for(1);
                // }
                this_thread::sleep_for(50);
                for (double i = 100; i >= 0; i -= speed_number) {
                    Chassis::getInstance()->autoSetForwardVel(i);
                    this_thread::sleep_for(1);
                }
            } else {
                break;
            }
        }
    }
}
void usrCtlThread() {
    static bool brakeHold = false;
    while (true) {
        Brain.Screen.setCursor(5, 1);
        Brain.Screen.print(Inertial.roll());
        Brain.Screen.setCursor(6, 1);
        Brain.Screen.print(Inertial.pitch());

        // Base chassis control
        baseControl();

        // Limit speed to avoid large pitch
        pitchControl();

        // auto fold wings
        foldSafety();

        // other devices
        foldControl();
        intakerControl();
        elevatorControl();
        momentumWheelControl();

        // chassis brake type switch
        if (press_LEFT) {
            press_LEFT = false;
            brakeHold = !brakeHold;
            Controller.rumble(".");
            if (brakeHold) {
                Controller.Screen.setCursor(3, 17);
                Controller.Screen.print("HOLD");
                Chassis::getInstance()->setStopBrakeType(hold);
            } else {
                Controller.Screen.setCursor(3, 17);
                Controller.Screen.print("      ");
                Chassis::getInstance()->setStopBrakeType(coast);
            }
        }

        this_thread::sleep_for(REFRESH_TIME);
    }
}

// user control initialization
void usercontrolInit() {
    fold_left();
    fold_right();
    Elevator_L.resetPosition();
    Elevator_R.resetPosition();
    Controller.Screen.print(111);
    Elevator_L.spinToPosition(300, deg, true);
    Elevator_L.stop(hold);
}

void usercontrol() {
    static thread Init(usercontrolInit);
    static thread UsrCtl(usrCtlThread);
}