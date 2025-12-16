// This file defines the format of the data packet shared between the two boards
// and utility functions to read data from the serial port.
#ifndef COMMS_CPP
#define COMMS_CPP

#include <Arduino.h>

struct AtoBPacket
{
  float setLeftAngvel;         // Angular velocity setpoint for the left PID controller, or open-loop effort if openLoop=true
  float setRightAngvel;        // Angular velocity setpoint for the right PID controller, or open-loop effort if openLoop=true
  bool reset;                  // If board A sets this to true, B will reset the motor controllers and encoder counts
  int gainChange;              // 1 to set the left controller's gains to newKp, newKi, newKd. 2 to set the right controller's gains, 0 to not change any of them
  float newKp;                 // Proportional gain of PID controller
  float newKi;                 // Integral gain of PID controller
  float newKd;                 // Derivative gain of PID controller
  bool openLoop;               // If true, board B will treat the setLeftAngvel and setRightAngvel as open-loop controls (1.0 for full forward, 0.0 for stop, -1.0 for reverse)
};

struct BtoAPacket
{
  float currLeftAngvel;
  float currRightAngvel;
};

// Read a float from the serial input and store it in f.
void serialReadFloat(float &f) 
{ 
  while (!Serial.available());
  f = Serial.parseFloat();
}

// Read an int from the serial input and store it in i.
void serialReadInt(int &i) 
{
  while (!Serial.available());
  i = Serial.parseInt();
}

#endif