#include <Arduino.h>
#include "../../common/config.h"

#define DEBUG 0

class MotorController {

    private:
      float KP, KI, KD;
      int *currCount;
      int debug;
    public:
      int prevCount, currCPL, setpointCPL, prevError, countsPerRev;
      float integral, pidOutput;
      int right;
    
      MotorController(int out, float kp, float ki, float kd, int* currCount, int Right, int countsPerRev, int debug) :
       KP(kp), KI(ki), KD(kd), pidOutput(0), currCount(currCount), countsPerRev(countsPerRev), debug(debug),
        setpointCPL(0), prevError(0), integral(0), prevCount(0), currCPL(0), right(Right) {}
    
      void update() 
      {
        /* Updates wheel encoder data.
        Updates the PID controller and output.
        Does not accumulate integral if the output is maxed. */
    
        // calculate wheel info
        this->currCPL = *currCount - this->prevCount;
        this->prevCount = *currCount;
    
        // do PID and set output
        int currError = this->setpointCPL - this->currCPL;
        this->pidOutput = (KP * currError) + (KI * this->integral) + (KD * (currError - this->prevError) / DT);
        if (abs(this->pidOutput) <= 255) { // don't accumulate integral if output is maxed
          this->integral += currError * DT;
        }
        this->prevError = currError;
        setSpeed((this->pidOutput));
    
      }
    
      void setPID(float p, float i, float d) 
      {
        this->KP = p;
        this->KI = i;
        this->KD = d;
      }
  
      void setSpeed(float pidOutput)
      {
        // PID output ranges from -255 to 255.
        // setSpeed converts this into the appropriate single-byte
        // serial simplified command, and writes it.
        // bit 7 is which motor
        // bit 6 is direction
        // bits 0-5 are speed. (decimal: 0-63)
        
        unsigned int channel = this->right ? (1 << 7) : 0;            // bit 7
        unsigned int direction = pidOutput < 0 ? (1 << 6) : 0;        // bit 6
        unsigned int speed = map(abs((int)pidOutput), 0, 255, 0, 63); // bits 0-5
        byte data = speed | direction | channel;

        Serial2.write(data);
        
        if (DEBUG) 
        {
           Serial.print("Motor value: ");
        }  Serial.println(data, BIN);
      }
  
      void newSetpoint(int sp)
      {
        this->integral = 0; // prevent error interference from prev setpoint
        this->setpointCPL = sp;
      }
    
    };