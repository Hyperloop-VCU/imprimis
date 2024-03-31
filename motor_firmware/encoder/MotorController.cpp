#include <Arduino.h>
#include "config.h"

class MotorController {
  /* Object handling PID control and output for a single motor.
  Also calculates the velocity linearVel of each wheel using linearVel = angularVel*WHEEL_RADIUS.
  The PID proccess variable is "encoder counts per PID loop", AKA the derivative d(encoderCount)/dt.
  input: current encoder counts per dt, desired encoder count per dt
  output: PWM speed, digital direction */

private:
  const float KP, KI, KD;
  int SPEEDOUT, DIROUT; // pins
  int prevError;
  int *currCountPtr;

public:
  float linearVel, integral;
  int setpointCPL, speedOutput;
  bool dirOutput;
  bool doWriting;

  MotorController(float kp, float ki, float kd, int* currCountPtr, bool left) : 
    KP(kp), KI(ki), KD(kd), currCountPtr(currCountPtr) {
    setpointCPL = 0;
    speedOutput = 0;
    dirOutput = true;
    doWriting = true;
    prevError = 0;
    integral = 0;

    if (left) {
      SPEEDOUT = LSPEED;
      DIROUT = LDIR;
    }
    else {
      SPEEDOUT = RSPEED;
      DIROUT = RDIR;
    }
    pinMode(SPEEDOUT, OUTPUT);
    pinMode(DIROUT, OUTPUT);
  }

  void update() {
    /* updates the speed and direction outputs of the PID controller, 
    and calculates the current angular and linear velocities of the wheels.
    does not accumulate integral if the output is maxed.
    If doWriting is false, does not write to outputs but still keeps track of movement. */

    this->linearVel = CPL_2_LINVEL * *(this->currCountPtr);
    int currError = this->setpointCPL - *(this->currCountPtr);
    *(currCountPtr) = 0;

    if (!doWriting) return;

    float output = (KP * currError) + (KI * this->integral) + (KD * (currError - this->prevError) / DT);
    this->prevError = currError;
  
    if (output > 255) output = 255;
    else if (output < -255) output = -255;
    else this->integral += currError * DT;
    this->speedOutput = (output > 0) ? round(output) : -round(output);
    
    analogWrite(SPEEDOUT, this->speedOutput);
    digitalWrite(DIROUT, output >= 0);
  }

};
