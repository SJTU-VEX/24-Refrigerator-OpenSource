/**
 * @file auto-functions.cpp
 * 
 * @copyright 2024 SJTU VEX
 */

#include "auto-functions.h"

#include <math.h>

#include "PID.h"
#include "basic-functions.h"
#include "chassis.h"
#include "iostream"
#include "my-timer.h"
#include "parameters.h"
#include "position.h"
#include "robot-config.h"
#include "vex.h"

using namespace vex;
using namespace std;

double calDAng(double _tar, double _cur) {
    double temp = _tar - _cur;
    while (temp <= -180) temp += 360;
    while (temp > 180) temp -= 360;
    return temp;
}

void timerForward(double _power, int _duration) {
    MyTimer myTm;

    // max accelerate/decelerate time (ms)
    int max_acc_time = 100;
    // accelerate/decelerate time ratio (pct)
    int acc_time_pct = 20;

    int acc_time = min(max_acc_time, _duration * acc_time_pct / 100);
    int const_time_end = _duration - acc_time;  // constant vel stage ending time

    int time;
    double powerCur;
    double baseLPosition = MotorBase_L1.position(deg);
    double baseRPosition = MotorBase_R1.position(deg);
    Chassis::getInstance()->autoSetRotateVel(0);
    myTm.reset();
    while ((time = myTm.getTime()) < acc_time) {
        powerCur = _power / acc_time * time;
        Chassis::getInstance()->autoSetForwardVel(powerCur);
        this_thread::sleep_for(REFRESH_TIME / 2);
    }
    while (myTm.getTime() < const_time_end) {
        double l = MotorBase_L1.position(deg) - baseLPosition;
        double r = MotorBase_R1.position(deg) - baseRPosition;
        double d = r - l;
        while (d < -180) d += 360;
        while (d > 180) d -= 360;
        double kp = 0.1;
        Chassis::getInstance()->autoSetRotateVel(kp * d);
        Chassis::getInstance()->autoSetForwardVel(_power);
        this_thread::sleep_for(REFRESH_TIME / 2);
    }
    while ((time = myTm.getTime()) < _duration) {
        double l = MotorBase_L1.position(deg) - baseLPosition;
        double r = MotorBase_R1.position(deg) - baseRPosition;
        double d = r - l;
        while (d < -180) d += 360;
        while (d > 180) d -= 360;
        double kp = 0.05;
        Chassis::getInstance()->autoSetRotateVel(kp * d);
        powerCur = _power / acc_time * (_duration - time);
        Chassis::getInstance()->autoSetForwardVel(powerCur);
        this_thread::sleep_for(REFRESH_TIME / 2);
    }
    Chassis::getInstance()->chassisBrake(hold);
}

void turnTo(double _power, double _target) {
    while (_target >= 360) {
        _target -= 360;
    }
    while (_target < 0) {
        _target += 360;
    }
    _power = abs(_power);
    double pid_threshold = 40;  // pid start threshold (deg)
    int sign = 1;
    double dAng = calDAng(_target, IMUHeading());
    if (dAng < 0) {
        sign = -1;
    } else {
        sign = 1;
    }
    while (abs(dAng) > pid_threshold) {
        dAng = calDAng(_target, IMUHeading());
        Chassis::getInstance()->autoSetRotateVel(sign * _power);
        this_thread::sleep_for(REFRESH_TIME);
    }
    PID pid;
    pid.setCoefficient(0.028, 0.001, 0.189);
    pid.setErrorTolerance(3);
    pid.setJumpTime(200);
    pid.setTarget(0);
    pid.update(dAng);
    MyTimer myTm;
    myTm.reset();
    while (myTm.getTime() <= PID_Exit_time && !pid.targetArrived()) {
        dAng = calDAng(_target, IMUHeading());
        pid.update(dAng);
        double vel = -pid.getOutput() * _power;
        if (abs(vel) > _power) {
            vel = sign * _power;
        }
        Chassis::getInstance()->autoSetRotateVel(vel);
        this_thread::sleep_for(REFRESH_TIME);
    }
}

void turnPreciseTo(double _power, double _target) {
    while (_target >= 360) {
        _target -= 360;
    }
    while (_target < 0) {
        _target += 360;
    }
    _power = abs(_power);
    double pid_threshold = 40;  // pid start threshold (deg)
    int sign = 1;
    double dAng = calDAng(_target, IMUHeading());
    if (dAng < 0) {
        sign = -1;
    } else {
        sign = 1;
    }
    while (abs(dAng) > pid_threshold) {
        dAng = calDAng(_target, IMUHeading());
        Chassis::getInstance()->autoSetRotateVel(sign * _power);
        this_thread::sleep_for(REFRESH_TIME);
    }
    PID pid;
    pid.setCoefficient(0.066, 0.0002, 0.33);
    pid.setErrorTolerance(2);
    pid.setJumpTime(50);
    pid.setTarget(0);
    pid.update(dAng);
    MyTimer myTm;
    myTm.reset();
    while (myTm.getTime() <= PID_Exit_time && !pid.targetArrived()) {
        dAng = calDAng(_target, IMUHeading());
        pid.update(dAng);
        double vel = -pid.getOutput() * _power;
        if (abs(vel) > _power) {
            vel = sign * _power;
        }
        Chassis::getInstance()->autoSetRotateVel(vel);
        this_thread::sleep_for(REFRESH_TIME);
    }
}

void distanceForwardCM(double _power, double _distance) {
    double dis_ang = rad2deg(_distance / WHEEL_TRANSITION_COEFFICIENT);
    distanceForward(_power, dis_ang);
}

void distanceForward(double _power, double _distance) {
    MyTimer myTm;

    // accelerate/decelerate time (ms)
    int acc_time = 100;

    int timeCur;
    double powerCur;
    double baseLPosition = MotorBase_L1.position(deg);
    double baseRPosition = MotorBase_R1.position(deg);
    double disCur;
    double pid_threshold = 1440;
    PID pidv;
    PID pidr;
    pidv.setCoefficient(0.028, 0.002, 0.189 * 8);
    pidv.setErrorTolerance(4);
    pidr.setCoefficient(0.025 * 2, 0.0005, 0.170);
    pidv.setTarget(_distance);
    pidr.setTarget(0);
    Chassis::getInstance()->autoSetRotateVel(0);
    myTm.reset();

    while (true) {
        double l = MotorBase_L1.position(deg) - baseLPosition;
        double r = MotorBase_R1.position(deg) - baseRPosition;
        disCur = (l + r) / 2;
        double d = r - l;
        while (d < -180) d += 360;
        while (d > 180) d -= 360;
        double kp = 0.1;
        pidr.update(d);
        Chassis::getInstance()->autoSetRotateVel(-pidr.getOutput());

        if ((timeCur = myTm.getTime()) < acc_time) {
            powerCur = timeCur * 0.3;
            Chassis::getInstance()->autoSetForwardVel(sign(_distance) * powerCur);
        } else {
            if (abs(_distance - disCur) >= pid_threshold) {
                Chassis::getInstance()->autoSetForwardVel(sign(_distance) * _power);
            } else {
                if (!pidv.targetArrived() && myTm.getTime() <= PID_Exit_time + 1000) {
                    pidv.update(disCur);
                    Chassis::getInstance()->autoSetForwardVel(pidv.getOutput());
                } else {
                    break;
                }
            }
        }
        this_thread::sleep_for(REFRESH_TIME);
    }
    Chassis::getInstance()->autoSetRotateVel(0);
    Chassis::getInstance()->chassisBrake(hold);
}

void forwardToPos(double power, Point tar) {
    // turn
    Point cur = Position::getInstance()->getPos();
    Vector diff = tar - cur;
    turnPreciseTo(power, 90 - diff.dir());

    // forward
    cur = Position::getInstance()->getPos();
    diff = tar - cur;
    distanceForwardCM(power, diff.mod());
}

void backToPos(double power, Point tar) {
    // turn
    Point cur = Position::getInstance()->getPos();
    Vector diff = tar - cur;
    turnPreciseTo(power, 270 - diff.dir());

    // back
    cur = Position::getInstance()->getPos();
    diff = tar - cur;
    distanceForwardCM(power, -diff.mod());
}

void forwardToPos(double power, double x, double y) { forwardToPos(power, Point(x, y)); }

void backToPos(double power, double x, double y) { backToPos(power, Point(x, y)); }

void timerForwardWithRotate(double _forwardPower, double _rotatePower, int _duration) {
    MyTimer myTm;

    // max accelerate/decelerate time (ms)
    int max_acc_time = 100;
    // accelerate/decelerate time ratio (pct)
    int acc_time_pct = 20;

    int acc_time = min(max_acc_time, _duration * acc_time_pct / 100);
    int const_time_end = _duration - acc_time;  // constant vel stage ending time

    int time;
    double forwardPowerCur, rotatePowerCur;
    double baseLPosition = MotorBase_L1.position(deg);
    double baseRPosition = MotorBase_R1.position(deg);
    Chassis::getInstance()->autoSetRotateVel(0);
    myTm.reset();
    while ((time = myTm.getTime()) < acc_time) {
        forwardPowerCur = _forwardPower / acc_time * time;
        rotatePowerCur = _rotatePower / acc_time * time;
        Chassis::getInstance()->autoSetForwardVel(forwardPowerCur);
        Chassis::getInstance()->autoSetRotateVel(rotatePowerCur);
        this_thread::sleep_for(REFRESH_TIME / 2);
    }
    while (myTm.getTime() < const_time_end) {
        Chassis::getInstance()->autoSetRotateVel(rotatePowerCur);
        Chassis::getInstance()->autoSetForwardVel(forwardPowerCur);
        this_thread::sleep_for(REFRESH_TIME / 2);
    }
    while ((time = myTm.getTime()) < _duration) {
        forwardPowerCur = _forwardPower / acc_time * (_duration - time);
        rotatePowerCur = _rotatePower / acc_time * (_duration - time);
        Chassis::getInstance()->autoSetForwardVel(forwardPowerCur);
        Chassis::getInstance()->autoSetRotateVel(rotatePowerCur);
        this_thread::sleep_for(REFRESH_TIME / 2);
    }
    Chassis::getInstance()->chassisBrake(hold);
}