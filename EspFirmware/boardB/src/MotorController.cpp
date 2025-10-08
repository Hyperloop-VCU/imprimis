// This is the class handling the actual motor control.
// Assumes the Cytron MDDS60 is in the simplified serial mode.
// One instance of the class will be created for each motor.


#include <Arduino.h>
#include <math.h>
#include "../../common/config.h"

#define MAX_INTEGRAL 32
#define OUTPUT_AVG_FILTER_LENGTH 1 // filter is disabled
#define ERROR_THRESHOLD 10
#define LOCK_UPDATES_REQUIRED 3

class MotorController 
{

  // private data members
  private:
  float KP, KI, KD;
  unsigned long time_of_last_update;
  int prevError;
  int lock_updates;
  bool locked;
  int prevSetpoint;
  int countsPerRev;
  float prevOutput;
  float pastOutputs[OUTPUT_AVG_FILTER_LENGTH];
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
    right(Right),
    debugB(debugB)
    {
      reset();
    }




  // Updates the PID controller and PID output.
  // Updates the wheel angular velocity.
  // nominal PID output ranges from 0 to 63, negative or positive

  // Will hold the PID output constant if the actual velocity has been "good" for some time.

  // setpointAngvel : desired angular velocity
  // count          : encoder counds since the last call to update. Should be zeroed after each update.
  // current_time   : current time as read from the millis() function.
  // first_update   : if true, uses DT_seconds = Initial_DT defined in config.
  // returns        : the angular velocity of the wheel.
  float update(float setpointAngvel, int count, unsigned long current_time, bool first_update) 
  {

    // calculate CPL, DT, wheel angvel, setpoint CPL, and error
    float DT_seconds;
    if (first_update) DT_seconds = Initial_DT;
    else DT_seconds = (current_time - time_of_last_update) / 1000.0;
    int setpointCPL = round(setpointAngvel * countsPerRev * DT_seconds / (2*M_PI));
    float wheel_angvel = count * ((2 * M_PI) / (countsPerRev * DT_seconds));
    int currError = setpointCPL - count;

    // figure out if we're locked
    if (count > setpointCPL - ERROR_THRESHOLD && count < setpointCPL + ERROR_THRESHOLD)
    {
      lock_updates++;
    }
    else lock_updates = 0;



    // do PID output
    float pidOutput;
    if (lock_updates >= LOCK_UPDATES_REQUIRED) // locked case
    {
      pidOutput = prevOutput;
      lock_updates = LOCK_UPDATES_REQUIRED;
    }
    else // unlocked case
    {
      integral += currError * DT_seconds;
      if (integral >= MAX_INTEGRAL) integral = MAX_INTEGRAL;
      else if (integral <= -MAX_INTEGRAL) integral = -MAX_INTEGRAL;
      pidOutput = (KP * currError) + (KI * integral) + (KD * (currError - prevError) / DT_seconds);
      lock_updates = 0;
    }

    // set previous values
    prevError = currError;
    prevSetpoint = setpointCPL;
    prevOutput = pidOutput;
    time_of_last_update = millis();

    // print information if needed
    if (debugB) 
    {
      Serial.print("DT: ");
      Serial.print(DT_seconds, 4);
      Serial.print(" I: ");
      Serial.print(integral);
      Serial.print(", currCPL: ");
      Serial.print(count);
    }

    // make the motor move
    float filteredOutput = filterPIDOutput(pidOutput);
    setSpeed(filteredOutput, debugB);
    return wheel_angvel;

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

    // Print information if necessary
    if (printInfo) {
    Serial.print(", PID output: ");
    Serial.print(pidOutput);
    Serial.print(", KI: ");
    Serial.print(KI);
    Serial.print(", Motor: ");
    Serial.print(channel >> 7);
    Serial.print(", Direction: ");
    Serial.print(direction >> 6);
    Serial.print(", Speed: ");
    Serial.println(speed);
    }

    // actually make the motor move
    uint8_t data = speed | direction | channel;
    Serial2.write(data);
  }



// Applies a simple moving average filter of length OUTPUT_AVG_FILTER_LENGTH.
// Takes the unfiltered count as an argument and returns the filtered count.
float filterPIDOutput(float PIDOutput)
{

  // add new count to the end
  pastOutputs[OUTPUT_AVG_FILTER_LENGTH-1] = PIDOutput;

  // get the output
  float avg = 0.0;
  for (int i = 0; i < OUTPUT_AVG_FILTER_LENGTH; i++) avg += pastOutputs[i];
  avg /= OUTPUT_AVG_FILTER_LENGTH;

  // make room for the next data point by shifting elements left
  for (int i = 1; i < OUTPUT_AVG_FILTER_LENGTH; i++) pastOutputs[i-1] = pastOutputs[i];


  return avg;
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
    this->integral = 0;
    this->prevError = 0;
    this->prevSetpoint = 0;
    this->prevOutput = 0.0;
    this->lock_updates = 0;
    this->locked = false;
    for (int i = 0; i < OUTPUT_AVG_FILTER_LENGTH; i++) pastOutputs[i] = 0;
    time_of_last_update = millis();
  }

};