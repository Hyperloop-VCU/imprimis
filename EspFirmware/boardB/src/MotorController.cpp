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
  float pidOutput;
  int right;


  public:
  // constructor
  MotorController(float kp, float ki, float kd, int Right, int countsPerRev) 
  : KP(kp),
    KI(ki), 
    KD(kd),
    countsPerRev(countsPerRev), 
    pidOutput(0.0),  
    prevError(0), 
    integral(0.0), 
    prevCount(0),
    right(Right),
    time_of_last_update(millis())
    {}




  // Updates the PID controller and PID output.
  // Updates the wheel angular velocity.
  // int currCount: current encoder count
  // unsigned long current_time: current time as read from the millis() function.
  // Returns the angular velocity of the wheel.
  float update(float setpointAngvel, int currCount, unsigned long current_time) 
  {

    // calculate CPL, DT, wheel angvel, and setpoint CPL
    int currCPL = currCount - prevCount;
    float DT_seconds = (current_time - time_of_last_update) / 1000.0;
    int setpointCPL = round(setpointAngvel * countsPerRev * DT_seconds / (2*M_PI));
    float wheel_angvel = currCPL * ((2 * M_PI) / (countsPerRev * DT_seconds));

    // do PID and set output
    int currError = setpointCPL - currCPL;
    pidOutput = (KP * currError) + (KI * integral); // + (KD * (currError - this->prevError) / DT_s);
    if (abs(pidOutput) <= 255) integral += currError * DT_seconds;

    // set previous values
    prevError = currError;
    prevCount = currCount;

    // make the motor move
    setSpeed((this->pidOutput));
    return wheel_angvel;

  }




  // sets the speed of each motor given a value between -255 and 255.
  // PID output ranges from -255 to 255.
  // setSpeed converts this into the appropriate single-byte
  // serial simplified command, and writes it.
  //  - bit 7 controls which motor to write to,
  //  - bit 6 controls the direction: fwd or reverse,
  //  - bits 0-5 control speed. (decimal: 0-63)
  void setSpeed(float pidOutput)
  {
    
    uint8_t channel = this->right ? (1 << 7) : 0;            // bit 7
    uint8_t direction = pidOutput < 0 ? (1 << 6) : 0;        // bit 6
    uint8_t speed = map(abs((int)pidOutput), 0, 255, 0, 63); // bits 0-5
    uint8_t data = speed | direction | channel;

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

};