// This is the class handling the actual motor control.
// Assumes the Cytron MDDS60 is in the simplified serial mode.
// One instance of the class will be created for each motor.


#include <Arduino.h>
#include <math.h>
#include "../../common/config.h"

class MotorController 
{
  

  // private data members
  private:
  float KP, KI, KD;
  unsigned long time_of_last_update;
  int prevCount;
  int prevError; 
  int countsPerRev;
  float integral;
  int right;
  bool debugB;


  public:
  // constructor
  MotorController(float kp, float ki, float kd, int Right, int countsPerRev, bool debugB) 
  : KP(kp),
    KI(ki), 
    KD(kd),
    countsPerRev(countsPerRev), 
    prevError(0), 
    integral(0.0), 
    prevCount(0),
    right(Right),
    debugB(debugB),
    time_of_last_update(millis())
    {}




  // Updates the PID controller and PID output.
  // Updates the wheel angular velocity.
  // float setpointAngvel: desired angular velocity
  // int currCount: current encoder count
  // unsigned long current_time: current time as read from the millis() function.
  // bool first_update: if true, uses DT_seconds = Initial_DT defined in config.
  // Returns the angular velocity of the wheel.
  float update(float setpointAngvel, int currCount, unsigned long current_time, bool first_update) 
  {

    // calculate CPL, DT, wheel angvel, and setpoint CPL
    float DT_seconds;
    int currCPL = currCount - prevCount;
    if (first_update) DT_seconds = Initial_DT;
    else DT_seconds = (current_time - time_of_last_update) / 1000.0;
    int setpointCPL = round(setpointAngvel * countsPerRev * DT_seconds / (2*M_PI));
    float wheel_angvel = currCPL * ((2 * M_PI) / (countsPerRev * DT_seconds));

    // do PID and set output
    int currError = setpointCPL - currCPL;
    if (abs(integral)*KI < 120) integral += currError * DT_seconds; // clamp integral term to maximum magnitude
    float pidOutput = (KP * currError) + (KI * integral); // + (KD * (currError - this->prevError) / DT_s);

    // set previous values
    prevError = currError;
    prevCount = currCount;
    time_of_last_update = millis();

    // TODO: remove these print statements
    if (debugB) {
    Serial.print("DT: ");
    Serial.print(DT_seconds);
    Serial.print(", currCount: ");
    Serial.print(currCount);
    Serial.print(", currCPL: ");
    Serial.print(currCPL);
    }

    // make the motor move
    setSpeed(pidOutput, debugB);
    return -3.218;

  }




  // sets the speed of each motor given a value between -255 and 255.
  // PID output ranges from -255 to 255.
  // setSpeed converts this into the appropriate single-byte
  // serial simplified command, and writes it.
  //  - bit 7 controls which motor to write to,
  //  - bit 6 controls the direction: fwd or reverse,
  //  - bits 0-5 control speed. (decimal: 0-63)
  void setSpeed(float pidOutput, bool printInfo)
  {
    
    uint8_t channel = this->right ? (1 << 7) : 0;            // bit 7
    uint8_t direction = pidOutput < 0 ? (1 << 6) : 0;        // bit 6
    uint8_t speed = map(abs((int)pidOutput), 0, 255, 0, 63); // bits 0-5
    if (speed > 63) speed = 63;
    uint8_t data = speed | direction | channel;

    // TODO: remove these print statements
    if (printInfo) {
    Serial.print(", PID output: ");
    Serial.print(pidOutput);
    Serial.print(", KP: ");
    Serial.print(KP);
    Serial.print(", Motor: ");
    Serial.print(channel >> 7);
    Serial.print(", Direction: ");
    Serial.print(direction >> 6);
    Serial.print(", Speed: ");
    Serial.println(speed);
    }

    Serial2.write(data);
  }




  // setter of the PID gains.
  void setPID(float p, float i, float d) 
  {
    this->KP = p;
    this->KI = i;
    this->KD = d;
  }


  // resets the encoder count and zeros the integral.
  void reset()
  {
    this->prevCount = 0;
    this->integral = 0;
  }

  // should be called before using the controller again
  // after not having used it for a while.

};