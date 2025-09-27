// Main configuration file for the ESP32 firmware.
// Controls the config board A and board B.


// TODO: remove the physical measurements section
// and calculate the ANGVEL_2_CPL constant at runtime,
// with the physical measurements passed to board A as a paremeter.
// This ensures the measurements only need to be defined in the high-lvl ros2 control config.

#ifndef CONFIG_H
#define CONFIG_H


// if the data fails to be sent from A to B or B to A this many times in a row, the system will shut down
#define MAX_SEND_RETRIES 10

// physical measurements of the robot - will be removed later
#define WHEEL_RADIUS 0.1651
#define HALF_WHEEL_TRACK_LENGTH 0.4432
#define COUNTS_PER_REV 3750 // 18.75 * 200


// MAC addresses for ESP NOW
const uint8_t A_MAC[] = {0x14, 0x2B, 0x2F, 0xDA, 0x7D, 0x10};
const uint8_t B_MAC[] = {0x14, 0x2B, 0x2F, 0xDB, 0xCB, 0x9C};


// pins for board B
// Motor output is pin 17. It's assigned automatically when the serial port is initialized.
#define LA 25 // must be interrupt capable
#define LB 26
#define RA 14 // must be interrupt capable
#define RB 12
#define LV 13 // goes to "lv" on level shifter, 3.3V output



// baud rates
#define SERIAL_BAUD_RATE_A 115200 // A to the PC
#define SERIAL_BAUD_RATE_B 9600  // B to the motors
#define DEBUG_BAUD_RATE_B 115200 // B to the (usually not connected) PC


// serial commands (used by board A only)
#define ANGVEL_SETPOINT 's'
#define RESET_ENCODERS 'r'
#define SET_PID 'e'

// PID initial parameters
#define Initial_KP 1.0
#define Initial_KI 0
#define Initial_KD 0

// timeout for board B. It will wait this long for data from A before stopping the robot.
const unsigned long timeout_ms = 1000;


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