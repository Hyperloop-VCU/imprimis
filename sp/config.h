#pragma once

// physical measurements
#define WHEEL_RADIUS 0.1651
#define HALF_WHEEL_TRACK_LENGTH 0.4432
#define COUNTS_PER_REV 3750 // 18.75 * 200
#define MAX_SPEED -1 // TODO

#define MANUAL_TURNING_FACTOR 0.5 // should be between 0 and 1. Sets how "hard" to turn at high speeds.

// pins for motion controller arduino
#define LA 2 // must be interrupt capable
#define LB 4
#define RA 3 // must be interrupt capable
#define RB 5
#define LSPEED 11 // must be PWM capable
#define LDIR 13
#define RSPEED 10 //must be PWM capable
#define RDIR 12
#define RC_MANUAL_LIN A0
#define RC_MANUAL_ANG A1
#define MANUAL_ENABLE 8

#define DT 0.05 // time in between each PID update in seconds (does not include execution time)
#define SERIAL_BAUD_RATE 9600

// serial commands
#define TWIST_SETPOINT 's'
#define RAW_PWM 'o'
#define RESET_ENCODERS 'r'
#define DEBUG 'p'
#define SET_PID 'e'

// useful constants
//const float CPL_2_ANGVEL = (2*M_PI) / (COUNTS_PER_REV * DT);
const float ANGVEL_2_CPL = (COUNTS_PER_REV * DT) / 2*M_PI;
//const float CPL_2_LINVEL = CPL_2_ANGVEL * WHEEL_RADIUS;
const float LINVEL_2_CPL = ANGVEL_2_CPL / WHEEL_RADIUS;
//const float COUNTS_2_METERS = (2*M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
const float MANUAL_TURN_COEFF = (1 - MANUAL_TURNING_FACTOR) / 255;
const int DT_MILLIS = 1000*DT;
