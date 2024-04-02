#pragma once

// physical measurements
#define WHEEL_RADIUS 0.1651
#define HALF_WHEEL_TRACK_LENGTH 0.4432
#define COUNTS_PER_REV 11765 // encoder counts per full revolution of the wheel

// pins
#define RA 2 // must be interrupt capable
#define RB 5
#define RINDEX 4
#define RSPEED 10 //must be PWM capable
#define RDIR 8
#define LA 3 // must be interrupt capable
#define LB 6
#define LINDEX 7
#define LSPEED 11 // must be PWM capable
#define LDIR 9

#define DT 0.05 // time in between each PID update in seconds (does not include execution time)
#define SERIAL_BAUD_RATE 115200

// serial commands
#define DISABLE_MOTORS 's'
#define ENABLE_MOTORS 'g'
#define TWIST_SETPOINT 't'
#define RESET_ANGPOS 'r'
#define GET_DATA 'd'
#define NEW_GAINS 'u'

// useful constants
const float CPL_2_ANGVEL = (2*M_PI) / (COUNTS_PER_REV * DT);
const float ANGVEL_2_CPL = (COUNTS_PER_REV * DT) / 2*M_PI;
const float COUNTS_2_METERS = (2*M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
const int loopsPerSec = 1 / DT;
const int DT_MILLIS = 1000*DT;
