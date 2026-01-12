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
  std::atomic<bool> first_update;
  std::atomic<bool> updating;
  std::atomic<bool> enabled;
  float integral;
  const int countsPerRev;
  const bool debugB, right;


  public:
  MotorController(float kp, float ki, float kd, bool right, int countsPerRev, bool debugB) 
  :  KP(kp), KI(ki), KD(kd), countsPerRev(countsPerRev), right(right), debugB(debugB), 
     integral(0.0), setpointAngvel(0.0), prevError(0), enabled(true), time_of_last_update(millis()), currAngvel(0.0)
  {
    reset();
  }


  // Updates the motor speed using the PID controller and calculates the wheel angular velocity with encoder feedback. 
  // count : encoder counts since the last call to update.
  void update(int count) 
  {
    // calculate DT, CPL, error, and wheel angvel
    updating = true;
    float DT_seconds;
    if (first_update) DT_seconds = 1 / (PID_UPDATE_PERIOD_MS / 1000.0);
    else DT_seconds = (millis() - time_of_last_update) / 1000.0;
    int setpointCPL = round(setpointAngvel * countsPerRev * DT_seconds / (2*M_PI));
    int currError = setpointCPL - count;
    currAngvel = count * ((2 * M_PI) / (countsPerRev * DT_seconds));

    // do PID output
    integral += currError * DT_seconds;
    if (first_update) integral = 0;
    else if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
    else if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
    float pidOutput = (KP * currError) + (KI * integral); //+ (KD * (currError - prevError) / DT_seconds);
    
    // set previous values
    prevError = currError;
    time_of_last_update = millis();
    first_update = false;

    // print if needed, make the motor move
    if (debugB) {
      float vel = currAngvel;
      Serial.printf("%5s: CL %+.2f Int: %+.2f\n", (right ? "RIGHT" : "LEFT"), pidOutput, integral);
    }
    setSpeed(pidOutput);
    updating = false;
  }

  // Updates the motor speed based on the setpoint alone; does not use the PID controller. Also calculates the wheel angular velocity with encoder feedback.
  // When this function is called, setpointAngvel should be in [-1.0, 1.0]. -1.0 is full reverse, 0.0 is stop, and 1.0 is full forward.
  void update_openloop(int count)
  {
    // calculate DT and wheel angvel
    updating = true;
    float DT_seconds;
    if (first_update) DT_seconds = 1 / (PID_UPDATE_PERIOD_MS / 1000.0);
    else DT_seconds = (millis() - time_of_last_update) / 1000.0;
    currAngvel = count * ((2 * M_PI) / (countsPerRev * DT_seconds));

    // set previous values
    time_of_last_update = millis();
    first_update = false;

    // print if needed, make the motor move
    if (debugB) {
      float set = setpointAngvel;
      float vel = currAngvel;
      Serial.printf("%5s: OL %+.2f Vel: %+.3f\n", (right ? "RIGHT" : "LEFT"), set, vel);
    }
    setSpeed(setpointAngvel * 63.0);
    updating = false;
  }


    // Sets the speed of the motor from a float value by converting it into the appropriate single-byte serial simplified command and writing it.
    // Bit 7 controls which motor to write to, bit 6 controls the direction, bits 0-5 control speed.
    void setSpeed(float pidOutput)
    {
      if (!enabled) return;
      uint8_t channel = right ? (1 << 7) : 0;            // bit 7
      uint8_t direction = pidOutput < 0 ? (1 << 6) : 0;  // bit 6
      uint8_t speed = abs(pidOutput);                    // bits 0-5
      if (right) direction ^= (1 << 6);
      if (speed > 63) speed = 63;
      uint8_t data = speed | direction | channel;
      Serial2.write(data);
    }

    // should be called before calling update or update_openloop for the first time in a while.
    void reset()
    {
      while (updating);
      first_update = true;
    }

    void setPID(float p, float i, float d) 
    {
      KP = p;
      KI = i;
      KD = d;
    }

    void newSetpoint(float newAngvelSetpoint) {setpointAngvel = newAngvelSetpoint;}

    float getAngvel() {return currAngvel;}

};