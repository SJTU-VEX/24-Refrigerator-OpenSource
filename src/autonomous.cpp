/**
 * @file autonomous.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "autonomous.h"

#include "auto-functions.h"
#include "basic-functions.h"
#include "chassis.h"

using namespace std;

void autonomous() {
    // flick 10 balls
    Chassis::getInstance()->setStopBrakeType(hold);
    flickBall(FLICKRIGHT, 10);
    Intaker.spin(fwd, 100, percentUnits::pct);
    turnTo(50, -10);
    unfold_right();
    timerForward(50, 400);
    Intaker.spin(fwd, 0, percentUnits::pct);
    fold_right();
    turnPreciseTo(30, 70);
    timerForward(-30, 650);
    turnPreciseTo(-30, -120);
    timerForwardWithRotate(-40, -15, 1460);

    // straight
    timerForward(-30, 1800);
    unfold_right();
    timerForward(-30, 1550);
    // fold_right();

    // curve
    timerForwardWithRotate(-25, -8.5, 3600);
    timerForward(-42, 1000);
    this_thread::sleep_for(100);
    timerForward(40, 400);
    this_thread::sleep_for(100);
    timerForward(-52, 800);
    this_thread::sleep_for(100);
    timerForward(40, 400);
    this_thread::sleep_for(100);
    timerForward(-52, 800);
    this_thread::sleep_for(100);
    timerForward(40, 800);
    Chassis::getInstance()->setStopBrakeType(coast);
}