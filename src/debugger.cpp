/**
 * @file debugger.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "debugger.h"

#include <iostream>

#include "auto-functions.h"
#include "autonomous.h"
#include "basic-functions.h"
#include "chassis.h"
#include "controller.h"
#include "my-timer.h"
#include "parameters.h"
#include "position.h"
#include "robot-config.h"
#include "usercontrol.h"

using namespace std;

void debugControl() {
    enum STATE { USR = 1, GYRO, PID, TESTAUTO, ELEVATOR } state;
    state = USR;
    MyTimer timer;
    while (true) {
        switch (state) {
            // user control【1】
            case USR:
                baseControl();
                foldControl();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("GlobalX: %.2f, GlobalY: %.2f", Position::getInstance()->getPos()._x,
                                   Position::getInstance()->getPos()._y);
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("Heading: %.2f", IMUHeading());
                if (press_B) {
                    press_B = false;
                    Inertial.setHeading(0, rotationUnits::deg);
                    Inertial.setRotation(0, rotationUnits::deg);
                    Position::getInstance()->reset();
                    clearBrainScr();
                    clearControllerScr();
                }
                if (press_A) {
                    press_A = false;
                    timer.reset();
                }
                if (press_Y) {
                    press_Y = false;
                    autonomous();
                }
                if (A2 > 70) {
                    state = ELEVATOR;
                    clearControllerScr();
                }
                if (A2 < -70) {
                    state = GYRO;
                    clearControllerScr();
                }
                clearControllerScr();
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print("TESTUSR");
                break;
            // gyro test【2】
            case GYRO:
                baseControl();

                clearBrainScr();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("%f", Inertial.rotation());
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("%f", IMUHeading());
                if (press_B) {
                    press_B = false;
                    Inertial.setHeading(0, rotationUnits::deg);
                    Inertial.setRotation(0, rotationUnits::deg);
                    Position::getInstance()->reset();
                    clearBrainScr();
                    clearControllerScr();
                }

                if (A2 > 70) {
                    state = USR;
                    clearControllerScr();
                }
                if (A2 < -70) {
                    state = PID;
                    clearControllerScr();
                }
                clearControllerScr();
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print("TESTGYRO");
                break;
            // PID test【3】
            case PID:
                baseControl();

                clearBrainScr();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("GlobalX: %.2f, GlobalY: %.2f", Position::getInstance()->getPos()._x,
                                   Position::getInstance()->getPos()._y);
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("Heading: %.2f", IMUHeading());
                if (press_A) {
                    press_A = false;
                    // turnTo(50, 90);
                    //  turnPreciseTo(50, 15);
                    // timerForward(30, 4000);
                    // forwardToPos(50,50,100);
                    distanceForwardCM(50, 100);
                }
                if (press_Y) {
                    press_Y = false;
                    turnTo(50, 15);
                    //  turnPreciseTo(50, 0);
                    // timerForward(100, 500);
                    // distanceForward(30, 3600);
                    // forwardToPos(50,0,0);
                }
                if (press_LEFT) {
                    press_LEFT = false;
                    // turnPreciseTo(50, 0);
                    //  turnPreciseTo(20, 180);
                    // timerForward(20, 1000);
                    distanceForwardCM(70, 100);
                }
                if (press_RIGHT) {
                    press_RIGHT = false;
                    //  turnPreciseTo(50, 270);
                    // timerForward(50, 500);
                    // distanceForward(30, 300);
                    forwardToPos(100, 50, 100);
                }
                if (press_B) {
                    press_B = false;
                    Inertial.setHeading(0, rotationUnits::deg);
                    Inertial.setRotation(0, rotationUnits::deg);
                    Position::getInstance()->reset();
                    clearBrainScr();
                    clearControllerScr();
                }
                if (A2 > 70) {
                    state = GYRO;
                    clearControllerScr();
                }
                if (A2 < -70) {
                    state = TESTAUTO;
                    clearControllerScr();
                }
                clearControllerScr();
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print("TESTPID");
                break;
            // run auto【4】
            case TESTAUTO:
                baseControl();
                foldControl();

                clearBrainScr();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("GlobalX: %.2f, GlobalY: %.2f", Position::getInstance()->getPos()._x,
                                   Position::getInstance()->getPos()._y);
                Brain.Screen.setCursor(2, 1);
                Brain.Screen.print("Heading: %.2f", IMUHeading());

                if (press_B) {
                    press_B = false;
                    Inertial.setHeading(0, rotationUnits::deg);
                    Inertial.setRotation(0, rotationUnits::deg);
                    Position::getInstance()->reset();
                    clearBrainScr();
                    clearControllerScr();
                }

                if (A2 > 70) {
                    state = PID;
                    clearControllerScr();
                }
                if (A2 < -70) {
                    state = ELEVATOR;
                    clearControllerScr();
                }
                clearControllerScr();
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print("TESTAUTO");
                break;

            case ELEVATOR:
                baseControl();
                if (UP) {
                    Elevator_L.spin(fwd, 40, pct);
                    Elevator_R.spin(fwd, 40, pct);
                } else if (DOWN) {
                    Elevator_L.spin(fwd, -40, pct);
                    Elevator_R.spin(fwd, -40, pct);
                } else {
                    Elevator_L.stop(hold);
                    Elevator_R.stop(hold);
                }

                clearBrainScr();
                Brain.Screen.setCursor(1, 1);
                Brain.Screen.print("L: %.2f, R: %.2f", Elevator_L.position(deg), Elevator_R.position(deg));

                if (press_A) {
                    press_A = false;
                    cout << "L: " << Elevator_L.position(deg) << "R: " << Elevator_R.position(deg) << endl;
                }

                if (press_B) {
                    press_B = false;
                    clearBrainScr();
                    Elevator_L.resetPosition();
                    Elevator_R.resetPosition();
                }

                if (A2 > 70) {
                    state = TESTAUTO;
                    clearControllerScr();
                }
                if (A2 < -70) {
                    state = USR;
                    clearControllerScr();
                }
                clearControllerScr();
                Controller.Screen.setCursor(5, 1);
                Controller.Screen.print("ELEVATOR");
                break;
                break;

                this_thread::sleep_for(REFRESH_TIME);
        }
    }
}