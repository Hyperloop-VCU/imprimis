// This is the firmware for ESP32 board "B" connected to the motor driver and encoders
// Board B has the following responsibilities:
//  - Accept angular velocity commands from board A
//  - Report the actual angular velocity of each wheel back to board A
//  - Run one PID controller for each motor to make them move
//  - Read from the encoders

// The PID loop runs at a fixed rate, but the controllers still calculate their own DT.

// If we haven't received data for some time,
// we give both controllers a setpoint of '0' to stop the robot.


#include <math.h>
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"
#include "MotorController.cpp"




bool debugB = false; // true to initialize the USB-C serial and print info to it




// global variables
volatile int leftEncoderCount = -1; // should start at 0, but interrupt triggers once for some reason
volatile int rightEncoderCount = 1;
MotorController leftController(2.3, 6.2, 0.0, 0, LEFT_COUNTS_PER_REV, debugB);
MotorController rightController(2.3, 6.2, 0.0, 1, RIGHT_COUNTS_PER_REV, debugB);
dataPacket data{};
esp_now_peer_info_t peerInfo;
volatile unsigned long time_of_last_data_receive = millis();
unsigned long time_of_last_controller_update = millis();
bool inactive = true;




// On-data-received callback, runs when board A sends data to board B.
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{ 

  // update timer
  time_of_last_data_receive = millis();

  // copy received data for processing
  memcpy(&data, incomingData, sizeof(data));

  // reset if we're commanded to, or we're activating
  if (inactive || data.reset)
  {
    leftController.reset();
    rightController.reset();
    leftEncoderCount = 0;
    rightEncoderCount = 0;
    data.reset = false;
  }
  
  // update controller PID gains if necessary
  if (data.gainChange == 1) 
  {
    leftController.setPID(data.newKp, data.newKi, data.newKd);
    data.gainChange = 0;
  }
  else if (data.gainChange == 2) 
  {
    rightController.setPID(data.newKp, data.newKi, data.newKd);
    data.gainChange = 0;
  }

  // update controller setpoints and get angular velocities
  leftController.newSetpoint(data.setLeftAngvel);
  rightController.newSetpoint(data.setRightAngvel);
  data.currLeftAngvel = leftController.getAngvel();
  data.currRightAngvel = rightController.getAngvel();

  // send processed data back to board A
  esp_err_t result = esp_now_send(A_MAC, (uint8_t *)&data, sizeof(data));
  inactive = false;

}


// Interrupt to update the right encoder count.
void IRAM_ATTR readRightEncoder() 
{ 
  if (!digitalRead(RB)) rightEncoderCount--;
  else rightEncoderCount++;
}


// Interrupt to update the left encoder count. 
void IRAM_ATTR readLeftEncoder() 
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
  if (debugB) Serial.begin(DEBUG_BAUD_RATE_B);

  pinMode(LA, INPUT_PULLUP);
  pinMode(RA, INPUT_PULLUP);
  pinMode(LB, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
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
// the loop updates the PID controllers at a fixed rate.
// this loop constantly checks if it's been too long since the last data packet was received.
// if it has been too long, it directly sets the motors to stop, bypassing the PID controllers.
void loop() 
{

  // Stop the robot if we haven't received data for a while
  if (millis() - time_of_last_data_receive > timeout_ms)
  {
    if (debugB)
    {
      Serial.print(leftEncoderCount);
      Serial.print(" ");
      Serial.println(rightEncoderCount);
      if (!inactive) Serial.println("Inactive - robot stopped.");
    }
    leftController.setSpeed(0, false);
    rightController.setSpeed(0, false);
    inactive = true;
    delay(10);
  }


  // Update PID controllers at a fixed rate
  if (millis() - time_of_last_controller_update > PID_UPDATE_PERIOD_MS)
  {
    if (!inactive)
    {
      leftController.update(leftEncoderCount, millis(), inactive);
      rightController.update(rightEncoderCount, millis(), inactive);
      rightEncoderCount = 0;
      leftEncoderCount = 0;
    }
    else
    {
      leftController.setSpeed(0.0, false);
      rightController.setSpeed(0.0, false);
    }
    time_of_last_controller_update = millis();
  }
}
