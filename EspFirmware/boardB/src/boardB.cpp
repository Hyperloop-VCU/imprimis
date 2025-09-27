// This is the firmware for ESP32 board "B" connected to the motor driver and encoders
// Board B has the following responsibilities:
//  - Accept desired counts-per-loop (CPL) commands from board A
//  - Report the actual the CPL of each motor back to board A
//  - Run one PID controller for each motor to make them move.

// The PID loop runs once each time board A sends data to board B.


#include <math.h>
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"
#include "MotorController.cpp"

#define BOARD 'B'

// global variables
int leftEncoderCount = 0;
int rightEncoderCount = 0;
MotorController leftController(Initial_KP, Initial_KI, Initial_KD, 0, COUNTS_PER_REV, INITIAL_DT);
MotorController rightController(Initial_KP, Initial_KI, Initial_KD, 1, COUNTS_PER_REV, INITIAL_DT);
dataPacket data = {0, 0, 0, 0, 0, Initial_KP, Initial_KI, Initial_KD};
esp_now_peer_info_t peerInfo;




// On-data-received callback, runs when board A sends data to board B.
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  // copy received data for processing
  memcpy(&data, incomingData, sizeof(data));

  // disable encoder reading
  noInterrupts();

  // reset encoders if necessary
  if (data.reset != 0)
  {
    leftController.reset();
    rightController.reset();
    data.reset = 0;
  }
  
  // set left and right setpoints if necessary
  if (data.setLeftAngvel != SETPOINT_RESET)
  {
    leftController.newSetpoint(data.setLeftAngvel);
    data.setLeftAngvel = SETPOINT_RESET;
  }
  if (data.setRightAngvel != SETPOINT_RESET)
  {
    rightController.newSetpoint(data.setRightAngvel);
    data.setRightAngvel = SETPOINT_RESET;
  }

  // update PID controllers' outputs
  leftController.update(leftEncoderCount, millis());
  rightController.update(rightEncoderCount, millis());

  // update PID controllers' PID values
  leftController.setPID(data.kp, data.ki, data.kd);
  rightController.setPID(data.kp, data.ki, data.kd);

  // update angvel data from controllers
  leftController.get_wheel_angvel();
  rightController.get_wheel_angvel();

  // re-enable encoder reading
  interrupts();

  // send processed data back to board A
  esp_err_t result = esp_now_send(A_MAC, (uint8_t *)&data, sizeof(data));

}





// Interrupt to update the right encoder count.
void readRightEncoder() 
{ 
  if (!digitalRead(RB)) rightEncoderCount--;
  else rightEncoderCount++;
}

// Interrupt to update the left encoder count. 
void readLeftEncoder() 
{ 
  if(!digitalRead(LB)) leftEncoderCount++;
  else leftEncoderCount--; // reversed for some reason, I don't know why
}





// Setup function: runs once on board B power-on
// sets up the I/O pins and serial port,
// then initializes ESP_now and registers the callback.
void setup() 
{

  // setup serial
  Serial2.begin(SERIAL_BAUD_RATE_B, SERIAL_8N1, 16, 17);
  if (1) Serial.begin(DEBUG_BAUD_RATE_B);

  // setup pins
  pinMode(RA, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
  pinMode(LA, INPUT_PULLUP);
  pinMode(LB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RA), readRightEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(LA), readLeftEncoder, FALLING);
  pinMode(LV, OUTPUT);
  digitalWrite(LV, HIGH);


  // setup ESP-now, register receive callback
  WiFi.mode(WIFI_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, A_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));

}




// nothing happens in the main loop function. 
// all the logic happens in the on-data-received callback.
void loop() 
{
}
