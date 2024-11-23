/**
 * @file position.h
 * @brief Defines the Position class for tracking the robot's position and speed.
 *
 * @copyright 2024 SJTU VEX
 */

#ifndef POSITION_H_
#define POSITION_H_

#include "geometry.h"
#include "my-timer.h"
#include "parameters.h"
#include "robot-config.h"

class Position {
private:
    double curIMUHeading;
    double curLMileage, curRMileage;
    double lastLMileage, lastRMileage;
    double curLSpeed, curRSpeed;
    double lastLSpeed, lastRSpeed;
    double selfSpeed;
    double globalXSpeed, globalYSpeed;
    double lastGlobalXSpeed, lastGlobalYSpeed;
    double globalX, globalY;
    double lastTime, sampleTime;
    MyTimer Timer;

    Position();
    void updateInertialHeading();
    void updateLMileage();
    void updateRMileage();
    void updateLSpeed();
    void updateRSpeed();
    void updateSelfSpeed();
    void updateGlobalYSpeed();
    void updateGlobalXSpeed();
    void updateGlobalY();
    void updateGlobalX();

public:
    // singleton pattern
    static Position *getInstance() {
        static Position *p = NULL;
        if (p == NULL) {
            p = new Position();
        }
        return p;
    }
    static void deleteInstance() {
        Position *p = Position::getInstance();
        if (p != NULL) {
            delete p;
            p = NULL;
        }
    }
    
    void updatePos();
    Point getPos() const;
    double getXSpeed() const;
    double getYSpeed() const;
    double getLMileage() const;
    double getRMileage() const;
    void resetYPosition();
    void resetXPosition();
    void setGlobalPosition(double _x, double _y);
    void reset();
};

void updatePosition();

#endif
