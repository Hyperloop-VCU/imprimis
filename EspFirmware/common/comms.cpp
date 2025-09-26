// This file defines the format of the data packet shared between the two boards,
// defines a function which initializes the ESP-now communication protocol,
// defines utility functions to read data from the serial port.


#ifndef COMMS_CPP
#define COMMS_CPP
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"




// Data packet structure definition.
struct dataPacket 
{
    int setLeftAngvel;         // Desired counts-per-loop value of the left PID controller
    int setRightAngvel;        // Desired counts-per-loop value of the right PID controller
    int currLeftAngvel;     // Current angular velocity of the left wheel
    int currRightAngvel;    // Current angular velocity of the right wheel
    int reset;              // Flag which indicates if board B needs to reset the encoder counts.
    float kp;               // Proportional gain of each wheel's PID controller
    float ki;               // Integral gain of each wheel's PID controller
    float kd;               // Derivative gain of each wheel's PID controller
};




// Debug function to print all the data in the packet to the USB-C serial port.
// Uses C-style pass by pointer.
void printAllData(dataPacket* data)
{
  Serial.print(data->currLeftAngvel);
  Serial.print(" ");
  Serial.print(data->currRightAngvel);
  Serial.print(" ");
  Serial.print(data->setLeftAngvel);
  Serial.print(" ");
  Serial.print(data->setRightAngvel);
  Serial.print(" ");
  Serial.print(data->reset);
  Serial.print(" ");
  Serial.print(data->kp);
  Serial.print(" ");
  Serial.print(data->ki);
  Serial.print(" ");
  Serial.println(data->kd);
}



// Initialization function, which sets up ESP-now communication between the two boads.
// Must be called individually on each board, and both boards must be powered on for it to work.
// Return codes:
//    0 : success
//    1 : ESP initialization failed (shouldn't happen)
//    2 : Could not find the other board
int ESPNowInit(char board, esp_now_peer_info* peerInfo)
{
    // Turn this board into a wifi station
    WiFi.mode(WIFI_STA);
  
    // initialize ESP now
    if (esp_now_init() != ESP_OK) return 1;
  
    // set the correct MAC address for peer
    if (board == 'b') memcpy(peerInfo->peer_addr, A_MAC, 6);
    if (board == 'a') memcpy(peerInfo->peer_addr, B_MAC, 6);
  
    // register peer
    peerInfo->channel = 0;  
    peerInfo->encrypt = false;
    if (esp_now_add_peer(peerInfo) != ESP_OK) return 2;
  
    return 0;
}
  



// Utility: read a float from the serial input and store it in f.
void serialReadFloat(float &f) 
{ 
  while (!Serial.available());
  f = Serial.parseFloat();
}




// Utility: read an int from the serial input and store it in i.
void serialReadInt(int &i) 
{
  while (!Serial.available());
  i = Serial.parseInt();
}

#endif