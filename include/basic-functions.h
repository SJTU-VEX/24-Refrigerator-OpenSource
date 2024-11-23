
/**
 * @file basic-functions.h
 * @brief Header file contains the declarations of basic functions apart from auto movement.
 * 
 * The functions declared in this file are used for controlling the robot's movements, 
 * getting sensor readings, and performing specific actions like flicking a ball.
 * 
 * @copyright 2024 SJTU VEX
 */

#ifndef BASIC_FUNCTIONS_H_
#define BASIC_FUNCTIONS_H_

#include "parameters.h"
#include "robot-config.h"

void moveLeft(double);
void moveRight(double);

double IMUHeading();
void resetHeading();

void clearBrainScr();
void clearControllerScr();

void unfold_left();
void fold_left();
void unfold_right();
void fold_right();

double getForwardVel();
double getRotationVel();

enum flickSide {FLICKLEFT, FLICKRIGHT};

void flickBall(flickSide side, int num = 1);

#endif