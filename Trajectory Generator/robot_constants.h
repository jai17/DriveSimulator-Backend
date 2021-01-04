#pragma once
//COMPUTATION CONSTANTS	
#define kEpsilonVel 2
#define MATH_PI 3.1415927

//EMPIRICAL CONSTANTS 
#define MAX_JERK 10 //ft/s^3 - irrelevant unless using s-curve
#define MAX_ACC 168 //in/s^2 - 14 ft/s^2
#define MAX_DEACC -72 //in/s^2 - 6 ft/s^2
#define MAX_VEL 120 //in/s - 10 ft/s
#define delta_t 0.01 //s - can be retrieved with timer if using RTOS (not currently used)
#define DIST_TO_ZERO 4 //ft (not currently used)
#define DRIVE_RADIUS (25.5 / 2) //in

#define kMaxDX 2.0 // inches
#define kMaxDY 0.25 // inches,  0.05
#define kMaxDTheta 5 * (MATH_PI / 180.0) // radians, 0.1

#define LOOKAHEAD_DISTANCE 15.0

#define DRIVE_WIDTH 25.0

//PID CONSTANTS
#define _kI 1
#define _kD 1
#define _kP 1
#define feedF 1
#define _integral_range 1
#define maxOutput 1
#define minOutput -1

