
// This is the firmware for ESP32 board "A" connected to the PC.
// Board A has the following responsibilities:
//   - interpreting inputs from the ROS hardware interface (angular velocity commands)
//   - providing outputs which go to the hardware interface (current angular velocity of each wheel)
//   - controlling the flow of data between the two ESP boards

// The timing of board B depends heavily on the rate at which the data is being sent to it
// by board A. Whenever the hardware interface writes a velocity command to this board,
// the data is sent.

#define DEBUG 1

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"





// Global variables
unsigned long t0;
dataPacket data = {0, 0, 0, 0, 0, Initial_KP, Initial_KI, Initial_KD}; // initial data
esp_now_peer_info_t peerInfo;




// Receive data callback: runs whenever board B sends data to board A (when a velocity command appears)
// Sets board A's local data to what board B sent it, then passes along
// the position and velocity of each motor to the PC.
// Whenever A sends data to B, B does its processing, updates the data if necessary,
// then sends the data back to A.
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&data, incomingData, sizeof(data));
  if (DEBUG) 
  {
    printAllData(&data);
  }
  else 
  {
    Serial.write((byte*)(&data.currLeftAngvel), sizeof(int));
    Serial.write((byte*)(&data.currRightAngvel), sizeof(int));
  }
}





// Read the first character from serial data, and execute the command associated with that character.
// does nothing if there is no serial data to read
// command RESET_ENCODERS: resets the encoder counts to 0.
// command TWIST_SETPOINT: gives a new setpoint to the PID controllers.
// command SET_PID: updates the pid gains of the controllers. sets both the left and right wheel controllers to the same gain.
void doCommand() 
{

  if (!Serial.available()) return;

  char chr = Serial.read();
  switch(chr) {

    case RESET_ENCODERS:
      data.reset = 1;
      break;

    case ANGVEL_SETPOINT: {
      float receivedLeftAngvel, receivedRightAngvel;
      serialReadFloat(receivedLeftAngvel);
      serialReadFloat(receivedRightAngvel);
      data.setLeftAngvel = receivedLeftAngvel;
      data.setRightAngvel = receivedRightAngvel;
      esp_err_t result = esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
      break;
    }

    case SET_PID: {
      float p, i, d;
      serialReadFloat(p);
      serialReadFloat(i);
      serialReadFloat(d);
      data.kp = p;
      data.ki = i;
      data.kd = d;
    }
  }

}




// Setup function: runs once at power-on
// Connects to board B, registers the on-data-receive callback,
// and initializes the value of t0.
void setup() 
{  
  Serial.begin(SERIAL_BAUD_RATE_A);
  ESPNowInit('a', &peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));
  t0 = millis();
}




// Main loop function: runs over and over again forever
// just calls doCommand over and over again.
void loop() 
{
  doCommand();
}