// Main configuration file for the ESP32 firmware.
// Contains definitions and constants for board A and board B.

#ifndef CONFIG_H
#define CONFIG_H

// physical measurements of the robot - should eventually be paramaterized in ROS and sent to this board on startup
#define WHEEL_RADIUS 0.1651
#define HALF_WHEEL_TRACK_LENGTH 0.4432


// encoder counts per one wheel revolution.
#define LEFT_COUNTS_PER_REV 4661
#define RIGHT_COUNTS_PER_REV 4653


// MAC addresses for ESP NOW
const uint8_t A_MAC[] = {0x80, 0xf3, 0xda, 0x62, 0xfa, 0xa8}; //{0x14, 0x2B, 0x2F, 0xDA, 0x7D, 0x10};
const uint8_t B_MAC[] = {0x80, 0xf3, 0xda, 0x62, 0xfc, 0xa4}; //{0x14, 0x2B, 0x2F, 0xDB, 0xCB, 0x9C};

// pins for board A
#define GREEN_LIGHT 25
#define YELLOW_LIGHT 26
#define RC_IBUS_RX 27

// yellow light switching speed for autonomous mode, board A
#define YELLOW_SWITCH_PERIOD_MS 1000


// pins for board B (motor output is pin 17 by default)
#define LA 25 // must be interrupt capable
#define LB 34
#define RA 35 // must be interrupt capable
#define RB 18
#define LV 13 // goes to "lv" on level shifter, 3.3V output


// baud rates
#define SERIAL_BAUD_RATE_A 115200 // A to the PC
#define SERIAL_BAUD_RATE_B 9600   // B to the motors
#define DEBUG_BAUD_RATE_B 115200  // B to the (usually not connected) PC


// serial commands (used by board A only)
#define ANGVEL_SETPOINT 's'
#define RESET_ENCODERS 'r'
#define SET_PID 'e'


// Fixed period at which to update the PID controllers. Changing this will require changing the PID gains.
#define PID_UPDATE_PERIOD_MS 5 // 200 Hz

// timeout for board B. It will wait this long for data from A before stopping the robot.
const unsigned long timeout_ms = 500;


#endif












// Useful constant formulas
// constant to convert angular velocity -> counts per loop = (COUNTS_PER_REV * DT) / 2*M_PI
// constant to convert counts per loop -> angular velocity = 1 / ANGVEL_2_CPL


/* Very old code for handling manual input to the motors.
#define MANUAL_TURNING_FACTOR 0.5 // should be between 0 and 1. Sets how "hard" to turn at high speeds.
const float MANUAL_TURN_COEFF = (1 - MANUAL_TURNING_FACTOR) / 255;
const float COUNTS_2_METERS = (2*M_PI * WHEEL_RADIUS) / COUNTS_PER_REV;
const float CPL_2_LINVEL = CPL_2_ANGVEL * WHEEL_RADIUS;

void setManualMotorInput() { 
  // Get differential drive input from the RC receiver, and set the motor speeds.
  int linearInput = pulseIn(RC_MANUAL_LIN, HIGH, 30000); // 30 millisecond timeout
  int angularInput = pulseIn(RC_MANUAL_ANG, HIGH, 30000);
  if (linearInput < 950 || linearInput > 2050) { // ensure motor speeds are 0 if the RC signal is invalid
    linearInput = 1500;
  }
  if (angularInput < 950 || angularInput > 2050) {
    angularInput = 1500;
  }
  int linNormalized = map(linearInput, 1000, 2000, -255, 255);
  int angNormalized = map(angularInput, 1000, 2000, -255, 255);
  int leftManualInput = round(linNormalized + angNormalized * (1 - MANUAL_TURN_COEFF * linNormalized));
  int rightManualInput = round(linNormalized - angNormalized * (1 - MANUAL_TURN_COEFF * linNormalized));
  leftController.corrected_setSpeed(leftManualInput);
  rightController.corrected_setSpeed(rightManualInput);
}
*/