/**
 * @file usercontrol.h
 * @brief Header file for user control functions and variables.
 *
 * This file contains the declarations of functions and variables used for
 * controlling various components of the robot, such as the base, elevator,
 * intaker. It also includes initialization functions and a user control
 * thread function.
 *
 * @copyright 2024 SJTU VEX
 */

#ifndef USERCONTROL_H_
#define USERCONTROL_H_

void baseControl();
void foldSafety();
void elevatorControl();
void elevatorInit();
void intakerControl();
void foldControl();

void usrCtlThread();
void usercontrol();
void momentumWheelControl();

void usercontrolInit();

static bool eleavtorReady = false;

#endif
