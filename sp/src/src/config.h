#include <math.h>

// physical measurements
#define WHEEL_RADIUS 0.1651
#define HALF_WHEEL_TRACK_LENGTH 0.4432
#define COUNTS_PER_REV 3750 // 18.75 * 200
#define MAX_SPEED -1 // TODO

// should be between 0 and 1. Sets how "hard" to turn at high speeds.
#define MANUAL_TURNING_FACTOR 0.5

// MAC addresses for ESP NOW
uint8_t A_MAC[] = {0x14, 0x2B, 0x2F, 0xDA, 0x7D, 0x10};
uint8_t B_MAC[] = {0x14, 0x2B, 0x2F, 0xDB, 0xCB, 0x9C};

// pins for board B
#define LA 26 // must be interrupt capable
#define LB 27
#define RA 14 // must be interrupt capable
#define RB 12
#define LV 4 // goes to "lv" on level shifter, 3.3V output
// Motor output is pin 17. It's assigned automatically when you initialize that serial port.

#define DT 0.05 // time in between each PID update in seconds
#define SERIAL_BAUD_RATE_A 115200 // A to the PC
#define SERIAL_BAUD_RATE_B 9600  // B to the motors

// serial commands
#define TWIST_SETPOINT 's'
#define RESET_ENCODERS 'r'
#define SET_PID 'e'

// "no command" value
#define SETPOINT_RESET -1

// PID parameters
#define Initial_KP 0.057
#define Initial_KI 0
#define Initial_KD 0




// useful constants
const float ANGVEL_2_CPL = (COUNTS_PER_REV * DT) / 2*M_PI;
//const float CPL_2_LINVEL = CPL_2_ANGVEL * WHEEL_RADIUS;
const float LINVEL_2_CPL = ANGVEL_2_CPL / WHEEL_RADIUS;
//const float COUNTS_2_METERS = (2*M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
const float MANUAL_TURN_COEFF = (1 - MANUAL_TURNING_FACTOR) / 255;
const int DT_MILLIS = 1000*DT;