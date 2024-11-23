/**
 * @file position.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "position.h"

#include <stdlib.h>

#include <cmath>
#include <iostream>

#include "basic-functions.h"
#include "my-timer.h"
#include "parameters.h"

using namespace std;
using namespace vex;

Position::Position() {
    curIMUHeading = 0;
    curLMileage = curRMileage = 0;
    lastLMileage = lastRMileage = 0;
    curLSpeed = curRSpeed = 0;
    lastLSpeed = lastRSpeed = 0;
    selfSpeed = 0;
    globalYSpeed = globalXSpeed = 0;
    globalY = globalX = 0;
    lastTime = 0;
    sampleTime = REFRESH_TIME;
}

// Inertial heading in radians, 0 ~ 2PI
// Forward as 0, clockwise as positive
void Position::updateInertialHeading() { curIMUHeading = deg2rad(IMUHeading()); }

// Left wheel distance in cm
void Position::updateLMileage() {
    lastLMileage = curLMileage;
    curLMileage = -deg2rad(-MotorBase_L1.position(degrees)) * WHEEL_TRANSITION_COEFFICIENT;
}

// Right wheel distance in cm
void Position::updateRMileage() {
    lastRMileage = curRMileage;
    curRMileage = -deg2rad(-MotorBase_R1.position(degrees)) * WHEEL_TRANSITION_COEFFICIENT;
}

// Left wheel speed cm/s
void Position::updateLSpeed() {
    lastLSpeed = curLSpeed;
    double ret = (curLMileage - lastLMileage) * 1000 / sampleTime;
    if (abs(ret) > 1000 || abs(ret) < 0.001) {
        ret = 0;
    }
    curLSpeed = ret;
}

// Right wheel speed cm/s
void Position::updateRSpeed() {
    lastRSpeed = curRSpeed;
    double ret = (curRMileage - lastRMileage) * 1000 / sampleTime;
    if (abs(ret) > 1000 || abs(ret) < 0.001) {
        ret = 0;
    }
    curRSpeed = ret;
}

// Self speed cm/s
void Position::updateSelfSpeed() { selfSpeed = (curLSpeed + curRSpeed) / 2; }

// World coordinate y-axis speed cm/s
void Position::updateGlobalYSpeed() {
    lastGlobalYSpeed = globalYSpeed;
    globalYSpeed = selfSpeed * cos(curIMUHeading);
    if (abs(globalYSpeed) < 0.01) {
        globalYSpeed = 0;
    }
    if (abs(globalYSpeed) > 250) {
        globalYSpeed = lastGlobalYSpeed;
    }
}

// World coordinate x-axis speed cm/s
void Position::updateGlobalXSpeed() {
    lastGlobalXSpeed = globalXSpeed;
    globalXSpeed = selfSpeed * sin(curIMUHeading);
    if (abs(globalXSpeed) < 0.01) {
        globalXSpeed = 0;
    }
    if (abs(globalXSpeed) > 250) {
        globalXSpeed = lastGlobalXSpeed;
    }
}

// Integrate world coordinate y-axis position
void Position::updateGlobalY() {
    double d = (globalYSpeed + lastGlobalYSpeed) * sampleTime / 1000 / 2;
    if (abs(d) < 0.001) {
        return;
    } else {
        globalY = globalY + d;
    }
}

// Integrate world coordinate x-axis position
void Position::updateGlobalX() {
    double d = (globalXSpeed + lastGlobalXSpeed) * sampleTime / 1000 / 2;
    if (abs(d) < 0.001) {
        return;
    } else {
        globalX = globalX + d;
    }
}

// Update global position
void Position::updatePos() {
    double time_cur = Timer.getTimeDouble();
    sampleTime = (time_cur - lastTime) * 1000;
    lastTime = time_cur;

    if (sampleTime < 0.001) {
        // dealing with the situation that sampleTime is too small
        sampleTime = REFRESH_TIME;
    }

    updateInertialHeading();
    updateLMileage();
    updateRMileage();
    updateLSpeed();
    updateRSpeed();
    updateSelfSpeed();
    updateGlobalYSpeed();
    updateGlobalXSpeed();
    updateGlobalY();
    updateGlobalX();
}

Point Position::getPos() const { return Point(globalX, globalY); }

double Position::getXSpeed() const { return globalXSpeed; }

double Position::getYSpeed() const { return globalYSpeed; }

double Position::getLMileage() const { return curLMileage; }

double Position::getRMileage() const { return curRMileage; }

void Position::resetXPosition() { globalX = 0; }

void Position::resetYPosition() { globalY = 0; }

void Position::setGlobalPosition(double _x, double _y) {
    globalX = _x;
    globalY = _y;
}

void updatePosition() {
    while (true) {
        Position::getInstance()->updatePos();
        this_thread::sleep_for(REFRESH_TIME);
    }
}

void Position::reset() {
    globalX = 0;
    globalY = 0;
}