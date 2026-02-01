// Main configuration file for the ESP32 firmware.
// Contains definitions and constants for board A and board B.

#ifndef CONFIG_H
#define CONFIG_H


// MAC addresses and security for ESP NOW
const uint8_t A_MAC[] = {0x80, 0xf3, 0xda, 0x62, 0xfa, 0xa8};
const uint8_t B_MAC[] = {0x80, 0xf3, 0xda, 0x62, 0xfc, 0xa4};
const char PMK[] = "testtesttesttest";
const char LMK[] = "testtesttesttest";

// data packets
struct AtoBPacket
{
  bool ignorePacket;           // If this is true, board B will pretend like it didn't receive the message. Used for probing connection status.
  float setLeftAngvel;         // Angular velocity setpoint for the left PID controller, or open-loop effort if openLoop=true
  float setRightAngvel;        // Angular velocity setpoint for the right PID controller, or open-loop effort if openLoop=true
  bool reset;                  // If board A sets this to true, B will reset the motor controllers and encoder counts
  int gainChange;              // 1 to set the left controller's gains to newKp, newKi, newKd. 2 to set the right controller's gains, 0 to not change any of them
  float newKp;                 // Proportional gain of PID controller
  float newKi;                 // Integral gain of PID controller
  float newKd;                 // Derivative gain of PID controller
  bool openLoop;               // If true, board B will treat the setLeftAngvel and setRightAngvel as open-loop efforts instead of closed-loop setpoints (1.0 for max forward effort, 0.0 for no effort, -1.0 for max reverse effort)
};

struct BtoAPacket
{
  float currLeftAngvel;
  float currRightAngvel;
};


// BOARD A //
#define GREEN_LIGHT 25
#define YELLOW_LIGHT 26
#define RX_RC_IBUS 27 // RX on this board
#define TX_RC_IBUS 15 // TX on this board (unused)
#define IMU_SCL 22
#define IMU_SDA 21
#define RX_GPS 16 // RX on board A, TX on GPS
#define TX_GPS 17 // TX on board A, RX on GPS

// GPS config
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_FIX_CTL_10HZ "$PMTK300,100,0,0,0,0*2C"

#define YELLOW_SWITCH_PERIOD_MS 800
#define IMU_READ_PERIOD_MS 50
#define GPS_READ_PERIOD_MS 100
#define CONNECTION_CHECK_PERIOD_MS 500

#define SERIAL_BAUD_RATE_A 921600
#define SERIAL_BAUD_RATE_GPS 9600

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
#define TIMEOUT_MS 300 // If B doesn't receive data for this long, stop the robot
#define DATA_SEND_RATE_MS 10 // How often to send wheel angvels from B to A

#define LEFT_COUNTS_PER_REV 4661 // Encoder counts per wheel revolution for each wheel
#define RIGHT_COUNTS_PER_REV 4653


#endif