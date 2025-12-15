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

  private:
  std::atomic<float> KP, KI, KD, setpointAngvel, currAngvel;
  std::atomic<int> prevError;
  std::atomic<unsigned long> time_of_last_update;
  bool first_update;
  float integral;
  const int countsPerRev;
  const bool debugB, right;

  public:
  MotorController(float kp, float ki, float kd, bool right, int countsPerRev, bool debugB) 
  :  KP(kp)
  ,  KI(ki)
  ,  KD(kd)
  ,  countsPerRev(countsPerRev)
  ,  right(right)
  ,  debugB(debugB)
    {
      reset();
    }


  // Updates the PID controller and PID output and the wheel angular velocity.
  // nominal PID output ranges from -63.0 to 63.0
  // count : encoder counts since the last call to update.
  // current_time : current time as read from the millis() function.
  void update(int count, unsigned long current_time) 
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
    setSpeed(pidOutput);
    currAngvel = wheel_angvel;

    first_update = false;
  }


    // sets the speed of the motor given a value between -63 and +63.
    // setSpeed converts this into the appropriate single-byte serial simplified command and writes it.
    // bit 7 controls which motor to write to,
    // bit 6 controls the direction: fwd or reverse,
    // bits 0-5 control speed. (decimal: 0-63)
    void setSpeed(float pidOutput)
    {
      // get channel, direction, and speed sections of the data byte
      uint8_t channel = right ? (1 << 7) : 0;      // bit 7
      uint8_t direction = pidOutput < 0 ? (1 << 6) : 0;  // bit 6
      uint8_t speed = abs(pidOutput);                    // bits 0-5

      // ensure right motor is spinning properly and clamp speed
      if (right) direction ^= (1 << 6);
      if (speed > 63) speed = 63;

      // actually make the motor move
      uint8_t data = speed | direction | channel;
      Serial2.write(data);

      // print debug info if needed
      if (debugB) {
        if (right) Serial.print("RIGHT: ");
        else Serial.print("LEFT: ");
        char info[15];
        sprintf(info, "%+2.2f   ", pidOutput);
        Serial.print(info);
        if (right) Serial.println();
      }
    }

    // should be called before calling update again after not using that method for a while.
    void reset()
    {
      this->integral = 0.0;
      this->prevError = 0;
      this->setpointAngvel = 0.0;
      this->currAngvel = 0.0;
      this->time_of_last_update = millis();
      this->first_update = true;
    }


    void newSetpoint(float newAngvelSetpoint)
    {
      this->setpointAngvel = newAngvelSetpoint;
    }


    float getAngvel()
    {
      return this->currAngvel;
    }


    void setPID(float p, float i, float d) 
    {
      this->KP = p;
      this->KI = i;
      this->KD = d;
    }

};