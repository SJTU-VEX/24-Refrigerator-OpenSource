/**
 * @file PID.cpp
 *
 * @copyright 2024 SJTU VEX
 */

#include "PID.h"

#include <math.h>

#include "basic-functions.h"
#include "geometry.h"

using std::abs;

PID::PID() : firstTime(true), arrived(false), IMax(20), IRange(50), jumpTime(50) { myTimer.reset(); }

void PID::setFirstTime() { firstTime = true; }

void PID::setCoefficient(double _kp, double _ki, double _kd) {
    kp = _kp;
    ki = _ki;
    kd = _kd;
}

void PID::setTarget(double _target) { target = _target; }
void PID::setIMax(double _IMax) { IMax = _IMax; }
void PID::setIRange(double _IRange) { IRange = _IRange; }
void PID::setErrorTolerance(double _errorTol) { errorTol = _errorTol; }
void PID::setDTolerance(double _DTol) { DTol = _DTol; }
void PID::setJumpTime(double _jumpTime) { jumpTime = _jumpTime; }
bool PID::targetArrived() { return arrived; }
double PID::getOutput() { return output; }

void PID::update(double input) {
    errorCurt = target - input;  // calculate current error
    P = kp * errorCurt;
    if (firstTime) {  // first time to update
        firstTime = false;
        errorPrev = errorCurt;
        errorInt = 0;
    }
    errorDev = errorCurt - errorPrev;  // calculate the derivative of error
    errorPrev = errorCurt;             // record error
    D = kd * errorDev;                 // calculate D
    if (abs(P) >= IRange) {            // I = 0 for P > IRange
        errorInt = 0;
    } else {  // P <= IRange -> Integrate
        errorInt += errorCurt;
        if (abs(errorInt) * ki > IMax)  // Limit I to IMax
            errorInt = sign(errorInt) * IMax / ki;
    }
    if (sign(errorInt) != sign(errorCurt) || (abs(errorCurt) <= errorTol))  // Clear I for small enough error
        errorInt = 0;
    I = ki * errorInt;  // Calculate I
    if (abs(errorCurt) <= errorTol &&
        abs(D) <= DTol) {  // Exit when staying in tolerated region and maintaining a low enough speed for enough time
        if (myTimer.getTime() >= jumpTime) arrived = true;
    } else {
        myTimer.reset();
    }
    output = P + I + D;
}