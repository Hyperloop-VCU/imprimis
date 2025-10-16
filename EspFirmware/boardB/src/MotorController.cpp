// This is the class handling the actual motor control.
// Assumes the Cytron MDDS60 is in the simplified serial mode.
// One instance of the class will be created for each motor.


#include <Arduino.h>
#include <math.h>
#include "../../common/config.h"
#include <atomic>

#define MAX_INTEGRAL 32

class MotorController 
{

  // private data members
  private:
  std::atomic<float> KP, KI, KD;
  std::atomic<float> setpointAngvel;
  std::atomic<float> currAngvel;
  unsigned long time_of_last_update;
  int prevError;
  int countsPerRev;
  float integral;
  int right;
  bool debugB;


  public:
  // constructor
  MotorController(float kp, float ki, float kd, int Right, int countsPerRev, bool debugB) 
  :  KP(kp)
  ,  KI(ki)
  ,  KD(kd)
  ,  countsPerRev(countsPerRev)
  ,  right(Right)
  ,  debugB(debugB)
    {
      reset();
    }



  // Updates the PID controller and PID output.
  // Updates the wheel angular velocity.
  // nominal PID output ranges from 0 to 63, negative or positive

  // count          : encoder counds since the last call to update. Should be zeroed after each update.
  // current_time   : current time as read from the millis() function.
  // first_update   : if true, does not calculate DT based on previous update calls; just uses 1 / update period.
void update(int count, unsigned long current_time, bool first_update) 
  {

    // calculate DT, CPL, error, and wheel angvel
    float DT_seconds;
    if (first_update) DT_seconds = 1 / (PID_UPDATE_PERIOD_MS / 1000.0);
    else DT_seconds = (current_time - time_of_last_update) / 1000.0;
    int setpointCPL = round(setpointAngvel * countsPerRev * DT_seconds / (2*M_PI));
    int currError = setpointCPL - count;
    float wheel_angvel = count * ((2 * M_PI) / (countsPerRev * DT_seconds));

    // do PID output
    integral += currError * DT_seconds;
    if (integral >= MAX_INTEGRAL) integral = MAX_INTEGRAL;
    else if (integral <= -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
    float pidOutput = (KP * currError) + (KI * integral); //+ (KD * (currError - prevError) / DT_seconds);

    // set previous values
    prevError = currError;
    time_of_last_update = millis();

    // make the motor move
    setSpeed(pidOutput, debugB);
    currAngvel = wheel_angvel;

  }




  // sets the speed of each motor given a float value between -63 and +63.
  // setSpeed converts this into the appropriate single-byte
  // serial simplified command, and writes it.
  //  - bit 7 controls which motor to write to,
  //  - bit 6 controls the direction: fwd or reverse,
  //  - bits 0-5 control speed. (decimal: 0-63)
  void setSpeed(float pidOutput, bool printInfo)
  {
    
    // get channel, direction, and speed sections of the data byte
    uint8_t channel = this->right ? (1 << 7) : 0;      // bit 7
    uint8_t direction = pidOutput < 0 ? (1 << 6) : 0;  // bit 6
    uint8_t speed = abs(pidOutput);                    // bits 0-5

    // ensure right motor is spinning properly
    if (right) direction ^= (1 << 6);

    // clamp speed
    if (speed > 63) speed = 63;

    // actually make the motor move
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



  // should be called before using the controller again
  // after not having used it for a while.
  void reset()
  {
    this->integral = 0.0;
    this->prevError = 0;
    this->setpointAngvel = 0.0;
    this->currAngvel = 0.0;
    this->time_of_last_update = millis();
  }

  // Setter for the angvelSetpoint
  void newSetpoint(float newAngvelSetpoint)
  {
    this->setpointAngvel = newAngvelSetpoint;
  }


  // getter of the angvel
  float getAngvel()
  {
    return this->currAngvel;
  }

};