// This file defines the format of the data packet shared between the two boards
// and defines utility functions to read data from the serial port.


#ifndef COMMS_CPP
#define COMMS_CPP

#include <Arduino.h>
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
  Serial.print("setLeftAngvel: " + String(data->setLeftAngvel));
  Serial.print(", setRightAngvel: " + String(data->setRightAngvel));
  Serial.print(", currLeftAngvel: " + String(data->currLeftAngvel));
  Serial.print(", currRightAngvel: " + String(data->currRightAngvel));
  Serial.print(", reset: " + String(data->reset));
  Serial.print(", kp: " + String(data->kp));
  Serial.print(", ki: " + String(data->ki));
  Serial.println(", kd: " + String(data->kd));
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