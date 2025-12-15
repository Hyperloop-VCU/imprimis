// This file defines the format of the data packet shared between the two boards
// and defines utility functions to read data from the serial port.


#ifndef COMMS_CPP
#define COMMS_CPP

#include <Arduino.h>
#include "config.h"



// Data packet structure definition.
struct dataPacket 
{
    float setLeftAngvel;         // Angular velocity setpoint for the left PID controller
    float setRightAngvel;        // Angular velocity setpoint for the left PID controller
    float currLeftAngvel;        // Current angular velocity of the left wheel
    float currRightAngvel;       // Current angular velocity of the right wheel
    bool reset;                  // Flag which indicates if board B needs to reset the encoder counts

    int gainChange;              // indicates which PID controller to change when modifying the gains. 0 for no change, 1 for left, 2 for right. Can't use an enum because of serial data
    float newKp;                 // Proportional gain of each wheel's PID controller
    float newKi;                 // Integral gain of each wheel's PID controller
    float newKd;                 // Derivative gain of each wheel's PID controller

    float pidLeftError;          // left PID controller error value
    float pidRightError;         // right PID controller error value    
};



// Debug function to print all the data in the packet (except gain change and PID gains) to the USB-C serial port.
// Uses C-style pass by pointer.
void printAllData(dataPacket* data)
{
  Serial.print("setLeftAngvel: " + String(data->setLeftAngvel));
  Serial.print(", setRightAngvel: " + String(data->setRightAngvel));
  Serial.print(", currLeftAngvel: " + String(data->currLeftAngvel));
  Serial.print(", currRightAngvel: " + String(data->currRightAngvel));
  Serial.print(", reset: " + String(data->reset));
  Serial.print(", pidLeftError: " + String(data->pidLeftError));
  Serial.println(", pidRightError: " + String(data->pidRightError));
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