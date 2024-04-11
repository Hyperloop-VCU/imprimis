#include <Arduino.h>
#include "config.h"
#include "CytronMotorDriver.h"

class MotorController : public CytronMD {
  /* Object handling PID control, wheel information, motor driving, and manual/automatic mode multiplexing for a single motor.
  input: current and desired encoder count per dt, OR manual signal from RC receiver
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

  void update(int rawInput, bool manual) {
    /* Updates the PID controller and the wheel encoder data.
    Does not accumulate integral if the output is maxed. 
    Directly sets the speed to rawInput if manual is true. */

    // calculate wheel info
    this->currCPL = *currCount - this->prevCount;
    this->prevCount = *currCount;

    // bypass PID controller and directly set speed to the input if in manual mode
    if (manual) {
      setSpeed(rawInput);
      return;
    }

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