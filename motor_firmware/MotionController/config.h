#pragma once

// physical measurements
#define WHEEL_RADIUS 0.1651
#define HALF_WHEEL_TRACK_LENGTH 0.4432
#define COUNTS_PER_REV 11765 // encoder counts per full revolution of the wheel
#define MAX_SPEED -1 // TODO

#define MANUAL_TURNING_FACTOR 0.5 // should be between 0 and 1. Sets how "hard" to turn while at max speed.

// pins for uno
#define RA 2 // must be interrupt capable
#define RB 4
#define LA 3 // must be interrupt capable
#define LB 7
#define RSPEED 10 //must be PWM capable
#define RDIR 12
#define LSPEED 11 // must be PWM capable
#define LDIR 13
#define RC_MANUAL_LIN A0
#define RC_MANUAL_ANG A1
#define MANUAL_ENABLE 8

#define DT 0.05 // time in between each PID update in seconds (does not include execution time)
#define SERIAL_BAUD_RATE 115200

// serial commands
#define TWIST_SETPOINT 's'
#define RESET_ENCODERS 'r'
#define GET_DATA 'd'
#define DEBUG 'p'

// useful constants
//const float CPL_2_ANGVEL = (2*M_PI) / (COUNTS_PER_REV * DT);
const float ANGVEL_2_CPL = (COUNTS_PER_REV * DT) / 2*M_PI;
//const float COUNTS_2_METERS = (2*M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
const float MANUAL_TURN_COEFF = (1 - MANUAL_TURNING_FACTOR) / 255;
const int DT_MILLIS = 1000*DT;