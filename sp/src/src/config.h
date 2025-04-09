#include <math.h>

// physical measurements
#define WHEEL_RADIUS 0.1651
#define HALF_WHEEL_TRACK_LENGTH 0.4432
#define COUNTS_PER_REV 3750 // 18.75 * 200
#define MAX_SPEED -1 // TODO

// should be between 0 and 1. Sets how "hard" to turn at high speeds.
#define MANUAL_TURNING_FACTOR 0.5

// MAC addresses for ESP NOW
uint8_t B_MAC[] = {0x14, 0x2B, 0x2F, 0xDB, 0xCB, 0x9C};
uint8_t A_MAC[] = {0x14, 0x2B, 0x2F, 0xDA, 0x7D, 0x10};

// pins for board B
#define LA 2 // must be interrupt capable
#define LB 4
#define RA 3 // must be interrupt capable
#define RB 5
#define MOTOR_OUTPUT 10

#define DT 0.05 // time in between each PID update in seconds
#define SERIAL_BAUD_RATE_A 115200

// serial commands
#define TWIST_SETPOINT 's'
#define RESET_ENCODERS 'r'
#define SET_PID 'e'




// useful constants
const float ANGVEL_2_CPL = (COUNTS_PER_REV * DT) / 2*M_PI;
//const float CPL_2_LINVEL = CPL_2_ANGVEL * WHEEL_RADIUS;
const float LINVEL_2_CPL = ANGVEL_2_CPL / WHEEL_RADIUS;
//const float COUNTS_2_METERS = (2*M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
const float MANUAL_TURN_COEFF = (1 - MANUAL_TURNING_FACTOR) / 255;
const int DT_MILLIS = 1000*DT;
