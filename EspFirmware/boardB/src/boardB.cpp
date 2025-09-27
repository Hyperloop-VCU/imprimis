// This is the firmware for ESP32 board "B" connected to the motor driver and encoders
// Board B has the following responsibilities:
//  - Accept angular velocity commands from board A
//  - Report the actual angular velocity of each wheel back to board A
//  - Run one PID controller for each motor to make them move
//  - Read from the encoders to close the PID loop

// The PID loop runs once each time board A sends data to board B.
// The "DT" parameter is thus inferred from the rate at which A sends data to B.
// This rate is controlled by how fast A receives data, which is ultimately controlled by
// Ros2 control and the hardware interface.
// This way, only the ros2_control config's update_rate needs to be changed
// for the whole system to use a different DT.

// A timer is constantly counting down, and it gets reset to its starting value when data is received.
// If the timer ever reaches 0, we stop the robot.
// The robot may be restarted if A resumes sending the data.


#include <math.h>
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"
#include "MotorController.cpp"



// global variables
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
MotorController leftController(Initial_KP, Initial_KI, Initial_KD, 0, COUNTS_PER_REV);
MotorController rightController(Initial_KP, Initial_KI, Initial_KD, 1, COUNTS_PER_REV);
dataPacket data = {0, 0, 0, 0, 0, Initial_KP, Initial_KI, Initial_KD};
esp_now_peer_info_t peerInfo;
volatile unsigned long time_of_last_data_receive = 0;



// On-data-received callback, runs when board A sends data to board B.
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{ 

  // update timer
  time_of_last_data_receive = millis();

  // copy received data for processing
  memcpy(&data, incomingData, sizeof(data));

  // reset encoders if necessary
  if (data.reset != 0)
  {
    leftController.reset();
    rightController.reset();
    data.reset = 0;
  }
  
  // update controller parameters
  leftController.setPID(data.kp, data.ki, data.kd);
  rightController.setPID(data.kp, data.ki, data.kd);

  // update PID controllers' outputs and angular velocities
  data.currLeftAngvel = leftController.update(data.setLeftAngvel, leftEncoderCount, millis());
  data.currRightAngvel = rightController.update(data.setRightAngvel, rightEncoderCount, millis());

  // send processed data back to board A
  printAllData(&data); // TODO remove this printing
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
// Note that board A does not need to be on for this function to run successfully.
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



// main loop function
// the main logic happens on data receive, in the receiveDataCB function.
// this loop constantly checks if it's been too long since the last data packet was received.
// if it has been too long, it directly sets the motors to stop, bypassing the PID controllers.
void loop() 
{
  if (millis() - time_of_last_data_receive > timeout_ms)
  {
    leftController.setSpeed(0);
    rightController.setSpeed(0);
    leftController.reset();
    rightController.reset();
    data.setLeftAngvel = 0;
    data.setRightAngvel = 0;
  }
}
