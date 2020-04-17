#ifndef HEXY_LIB_H
#define HEXY_LIB_H

#include <ros/ros.h>

// mechanical parameters
//#define L1_LENGTH 0.034
//#define L2_LENGTH 0.095
//#define L3_LENGTH 0.134//0.142//0.13395
//#define BODY_WIDTH_S 0.135
//#define BODY_WIDTH_B 0.155
//#define BODY_LENGTH 0.220
//#define BODY_HEIGTH 0.04

// math constants
#define PI 3.14159265

// common parameters
#define LEG_L1 0
#define LEG_L2 1
#define LEG_L3 2
#define LEG_R1 3
#define LEG_R2 4
#define LEG_R3 5

#define EP_X 0
#define EP_Y 1
#define EP_Z 2

#define COXA  0
#define FEMUR 1
#define TIBIA 2

// parameters for dualshock 4
#define SQUARE    0
#define TRIANGLE  1
#define CIRCLE    2
#define CROSS     3
#define L1        4
#define L2        5
#define R1        6
#define R2        7
#define SHARE     8
#define OPTIONS   9
#define PS        10
#define TOUCH     11
#define L3        12
#define R3        13
#define LEFT      14
#define UP        15
#define RIGHT     16
#define DOWN      17

#define L3_HOR     0
#define L3_VERT    1
#define R3_HOR     2
#define R3_VERT    3
#define L2_FLOAT  4
#define R2_FLOAT  5

#define CIRCLE_MODE 1
#define JOY_MODE    2

#define L1_MODE 1
#define L2_MODE 2
#define L3_MODE 3
#define R1_MODE 4
#define R2_MODE 5
#define R3_MODE 6

// parameters for geometry constraints
// xyz constraints for all the legs
#define L1_X_MAX 0.278
#define L1_X_MIN 0.11
#define L1_Y_MAX 0.27
#define L1_Y_MIN 0.125
#define L1_Z_MAX 0.0
#define L1_Z_MIN -0.145

#define L2_X_MAX 0.11
#define L2_X_MIN -0.11
#define L2_Y_MAX 0.297
#define L2_Y_MIN 0.161
#define L2_Z_MAX 0.0
#define L2_Z_MIN -0.145

#define L3_X_MAX -0.11
#define L3_X_MIN -0.278
#define L3_Y_MAX 0.27
#define L3_Y_MIN 0.125
#define L3_Z_MAX 0.0
#define L3_Z_MIN -0.145

#define R1_X_MAX 0.278
#define R1_X_MIN 0.11
#define R1_Y_MAX -0.125
#define R1_Y_MIN -0.27
#define R1_Z_MAX 0.0
#define R1_Z_MIN -0.145

#define R2_X_MAX 0.11
#define R2_X_MIN -0.11
#define R2_Y_MAX -0.161
#define R2_Y_MIN -0.297
#define R2_Z_MAX 0.0
#define R2_Z_MIN -0.145

#define R3_X_MAX -0.11
#define R3_X_MIN -0.278
#define R3_Y_MAX -0.125
#define R3_Y_MIN -0.27
#define R3_Z_MAX 0.0
#define R3_Z_MIN -0.145

void sayHello();

#endif // HEXY_LIB_H
