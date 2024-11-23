/**
 * @file PID.h
 * @brief Header file for the PID controller class.
 *
 * @copyright 2024 SJTU VEX
 */

#ifndef PID_H_
#define PID_H_

#include "my-timer.h"

class PID {
private:
    double errorCurt = 0, errorPrev = 0, errorDev = 0, errorInt = 0;
    double P = 0, I = 0, D = 0;
    bool firstTime = true;
    bool arrived = false;
    double kp = 1, ki = 0, kd = 0;
    double target = 0, errorTol = 0, DTol = 0;
    double IMax = 20, IRange = 50;  // I < abs(IMAX); I starts to increase when P < IRange
    double output = 0;
    double jumpTime = 50;
    MyTimer myTimer;

public:
    PID();
    void setFirstTime();
    void setCoefficient(double, double, double);
    void setTarget(double);
    void setIMax(double);
    void setIRange(double);
    void setErrorTolerance(double);
    void setDTolerance(double);
    void setJumpTime(double);
    void update(double input);
    bool targetArrived();
    double getOutput();
};

#endif