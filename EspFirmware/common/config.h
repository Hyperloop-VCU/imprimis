// Main configuration file for the ESP32 firmware.
// Contains definitions and constants for board A and board B.

#ifndef CONFIG_H
#define CONFIG_H
#include "driver/pcnt.h"
#include "hal/pcnt_hal.h"


// MAC addresses and security for ESP NOW
const uint8_t A_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//{0x14, 0x2b, 0x2f, 0xda, 0x7d, 0x10};
const uint8_t B_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};//{0x14, 0x2b, 0x2f, 0xdb, 0xcb, 0x9c};
const char PMK[] = "testtesttesttest";
const char LMK[] = "testtesttesttest";

// data packets
struct AtoBPacket
{
  //bool ignorePacket;           // If this is true, board B will pretend like it didn't receive the message. Used for probing connection status.
  float setLeftAngvel;         // Angular velocity setpoint for the left PID controller, or open-loop effort if openLoop=true
  float setRightAngvel;        // Angular velocity setpoint for the right PID controller, or open-loop effort if openLoop=true
  int32_t t_sent;
  //bool reset;                  // If board A sets this to true, B will reset the motor controllers and encoder counts
  //int gainChange;              // 1 to set the left controller's gains to newKp, newKi, newKd. 2 to set the right controller's gains, 0 to not change any of them
  //float newKp;                 // Proportional gain of PID controller
  //float newKi;                 // Integral gain of PID controller
  //float newKd;                 // Derivative gain of PID controller
  //bool openLoop;               // If true, board B will treat the setLeftAngvel and setRightAngvel as open-loop efforts instead of closed-loop setpoints (1.0 for max forward effort, 0.0 for no effort, -1.0 for max reverse effort)
};

#endif