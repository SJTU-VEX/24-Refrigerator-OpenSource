/**
 * @file my-timer.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "my-timer.h"

#include <math.h>

#include "robot-config.h"

using namespace vex;

MyTimer::MyTimer() { startTime = Brain.Timer.value(); }

MyTimer::MyTimer(double init) { startTime = Brain.Timer.value() + init / 1000; }

void MyTimer::reset() { startTime = Brain.Timer.value(); }

int MyTimer::getTime() const {
    return floor((Brain.Timer.value() - startTime) * 1000);  // return time (msec) from startTime
}

double MyTimer::getTimeDouble() const {
    return Brain.Timer.value() - startTime;  // return time (sec) from startTime
}