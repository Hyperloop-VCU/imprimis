#include <Arduino.h>
#include "config.h"

class MotorController {
  /*
  Bit 7 (Channel): 0 - motor LEFT, 1 - motor RIGHT.
Bit 6 - 0: Control motor direction and speed.

Example: Value in Decimal
0 or 64 - motor LEFT stop.
63 - motor LEFT full forward.
127 - motor LEFT full reverse.
128 or 192 - motor RIGHT stop.
191 - motor RIGHT full forward.
255 - motor RIGHT full reverse.

  */
private:
  float KP, KI, KD;
  int *currCount;
public:
  int prevCount, currCPL, setpointCPL, prevError, countsPerRev;
  float integral, pidOutput;
  int right;

  MotorController(int output, float kp, float ki, float kd, int* currCount, int Right, int countsPerRev) :
   KP(kp), KI(ki), KD(kd), pidOutput(0), currCount(currCount), countsPerRev(countsPerRev),
    setpointCPL(0), prevError(0), integral(0), prevCount(0), currCPL(0), right(Right) {}

  void update(bool openLoop) {
    /* Updates wheel encoder data.
    Updates the PID controller and output if openLoop = false.
    Does not accumulate integral if the output is maxed. */

    // calculate wheel info
    this->currCPL = *currCount - this->prevCount;
    this->prevCount = *currCount;

    if (openLoop) {
      this->integral = 0;
      return;
    }

    // do PID and set output
    int currError = this->setpointCPL - this->currCPL;
    this->pidOutput = (KP * currError) + (KI * this->integral) + (KD * (currError - this->prevError) / DT);
    if (abs(this->pidOutput) <= 255) { // don't accumulate integral if output is maxed
      this->integral += currError * DT;
    }
    this->prevError = currError;
    corrected_setSpeed((int)(this->pidOutput));

  }

  void setPID(float p, float i, float d) {
    this->KP = p;
    this->KI = i;
    this->KD = d;
  }

  void setSpeed(int spd) {
    // Interface with "serial simplified mode"

  }

  void corrected_setSpeed(int spd) {
    // for some reason, the right motor is reversed
    if (this->right) setSpeed(-spd);
    else setSpeed(spd);
  }

};
