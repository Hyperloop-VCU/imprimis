#include <Arduino.h>
#include "config.h"
#include "CytronMotorDriver.h"

class MotorController : public CytronMD {
  /* Object handling autonomous PID control, wheel information, and motor driving for a single motor.
  input: current and desired encoder count per dt
  output: (PWM) speed, (digital) direction.
  The PID proccess variable is "encoder counts per PID loop", AKA the derivative d(encoderCount)/dt. */

private:
  const float KP, KI, KD;
  long *currCount;
public:
  long prevCount, currCPL, setpointCPL, prevError;
  float integral, pidOutput;

  MotorController(int speedout, int dirout, float kp, float ki, float kd, long* currCount) :
    CytronMD(PWM_DIR, speedout, dirout), KP(kp), KI(ki), KD(kd), pidOutput(0), currCount(currCount),
    setpointCPL(0), prevError(0), integral(0), prevCount(0), currCPL(0) {}

  void update() {
    /* Updates the PID controller and the wheel encoder data.
    Does not accumulate integral if the output is maxed. */

    // calculate wheel info
    this->currCPL = *currCount - this->prevCount;
    this->prevCount = *currCount;

    // set output based on encoder feedback and PID computations if in autonomous mode
    int currError = this->setpointCPL - this->currCPL;
    this->pidOutput = (KP * currError) + (KI * this->integral); //+ (KD * (currError - this->prevError) / DT);
    if (abs(this->pidOutput) <= 255) { // don't accumulate integral if output is maxed
      this->integral += currError * DT;
    }
    this->prevError = currError;
    setSpeed(round(this->pidOutput));

  }

};