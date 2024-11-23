/**
 * @file my-timer.h
 * @brief Header file for the MyTimer class.
 *
 * This file contains the definition of the MyTimer class, which provides
 * functionality for timing operations.
 *
 * @copyright 2024 SJTU VEX
 */

#ifndef MYTIMER_H_
#define MYTIMER_H_

#include "vex.h"

class MyTimer {
private:
    double startTime;

public:
    MyTimer();
    MyTimer(double);
    void reset();
    int getTime() const;
    double getTimeDouble() const;
};

#endif