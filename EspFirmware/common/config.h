// Main configuration file for the ESP32 firmware.
// Contains definitions and constants for board A and board B.

#ifndef CONFIG_H
#define CONFIG_H


// MAC addresses and security for ESP NOW
const uint8_t A_MAC[] = {0x80, 0xf3, 0xda, 0x62, 0xfa, 0xa8};
const uint8_t B_MAC[] = {0x80, 0xf3, 0xda, 0x62, 0xfc, 0xa4};
const char PMK[] = "testtesttesttest";
const char LMK[] = "testtesttesttest";


// BOARD A //
#define GREEN_LIGHT 25
#define YELLOW_LIGHT 26
#define RC_IBUS_RX 27

#define YELLOW_SWITCH_PERIOD_MS 1000
#define IMU_READ_PERIOD_MS 100

#define SERIAL_BAUD_RATE_A 115200 // A to the PC

#define ANGVEL_SETPOINT 's'
#define RESET_ENCODERS 'r'
#define SET_PID 'e'


// BOARD B //
#define LA 25 // must be interrupt capable
#define LB 34
#define RA 35 // must be interrupt capable
#define RB 18
#define LV 13 // goes to "lv" on level shifter, 3.3V output

#define SERIAL_BAUD_RATE_B 9600   // B to the motors
#define DEBUG_BAUD_RATE_B 115200  // B to the (usually not connected) PC

#define PID_UPDATE_PERIOD_MS 5 // PID loop rate - changing this requires changing the PID gains
#define TIMEOUT_MS 500 // If B doesn't receive data for this long, stop the robot
#define DATA_SEND_RATE_MS 10 // How often to send wheel angvels from B to A

#define LEFT_COUNTS_PER_REV 4661 // Encoder counts per wheel revolution for each wheel
#define RIGHT_COUNTS_PER_REV 4653


#endif