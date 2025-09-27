
// This is the firmware for ESP32 board "A" connected to the PC.
// Board A has the following responsibilities:
//   - interpreting inputs from the ROS hardware interface (angular velocity commands)
//   - providing outputs which go to the hardware interface (current angular velocity of each wheel)
//   - controlling the flow of data between the two ESP boards

// The timing of board B depends heavily on the rate at which the data is being sent to it
// by board A. Whenever the hardware interface writes a velocity command to this board,
// the data is sent.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"





// Global variables
unsigned long t0;
dataPacket data = {0, 0, 0, 0, 0, Initial_KP, Initial_KI, Initial_KD}; // initial data
esp_now_peer_info_t peerInfo;
bool is_connected = false;
int retryCount = 0;




// Receive data callback: runs whenever board B sends data to board A (when a velocity command appears)
// Sets board A's local data to what board B sent it, then passes along
// the position and velocity of each motor to the PC.
// Whenever A sends data to B, B does its processing, updates the data if necessary,
// then sends the data back to A.
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&data, incomingData, sizeof(data));
  //printAllData(&data);
  //Serial.write((byte*)(&data.currLeftAngvel), sizeof(int));
  //Serial.write((byte*)(&data.currRightAngvel), sizeof(int));
}


// Send data callback: runs whenever board A sends data to board B.
// Ensures board B received the data. Handles retry and is_connected logic.
void sendDataCB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.println(is_connected);
  if (status != ESP_NOW_SEND_SUCCESS)
  {
    if (retryCount > MAX_SEND_RETRIES) is_connected = false;
    else retryCount++;
    return;
  }
  is_connected = true;
  retryCount = 0;
}




// Reads the first character from serial data and executes the command associated with that character.
// Does nothing if there is no serial data to read
// command RESET_ENCODERS: Resets the encoder counts and PID integral to 0.
// command ANGVEL_SETPOINT: Gives a new angular-velocity setpoint to the PID controllers.
// command SET_PID: Updates the pid gains of the controllers. Sets both the left and right wheel controllers to the same gain.
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
      // send data to B
      esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
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




// Setup function: runs once on board A power-on
// Initializes the serial port and registers board B as a peer.
// Note that board B does not need to be on for this function to run successfully.
void setup() 
{  
  delay(100);
  Serial.begin(SERIAL_BAUD_RATE_A);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, B_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));
  esp_now_register_send_cb(esp_now_send_cb_t(sendDataCB));

}




// Main loop function: runs over and over again forever
// just calls doCommand over and over again.
// TODO: do something when is_connected = false
float leftAV = 0, rightAV = 0;
void loop() 
{
  //doCommand();
  data.setLeftAngvel = leftAV;
  data.setRightAngvel = rightAV;
  esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
  delay(30);
}