#include <math.h>
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"

// This is the firmware for ESP32 board "B" connected to the motor driver and encoders
unsigned long t0;


class MotorController {

  private:
    float KP, KI, KD;
    int *currCount;
  public:
    int prevCount, currCPL, setpointCPL, prevError, countsPerRev;
    float integral, pidOutput;
    int right;
  
    MotorController(int out, float kp, float ki, float kd, int* currCount, int Right, int countsPerRev) :
     KP(kp), KI(ki), KD(kd), pidOutput(0), currCount(currCount), countsPerRev(countsPerRev),
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
    }

    void newSetpoint(int sp)
    {
      this->integral = 0; // prevent error interference from prev setpoint
      this->setpointCPL = sp;
    }
  
  };

int leftEncoderCount = 0, rightEncoderCount = 0;
MotorController leftController(17, Initial_KP, Initial_KI, Initial_KD, &leftEncoderCount, 0, COUNTS_PER_REV);
MotorController rightController(17, Initial_KP, Initial_KI, Initial_KD, &rightEncoderCount, 1, COUNTS_PER_REV);

struct dataPacket { // data packet, shared between A and B
  int setLeftCPL;      // set by A, processed and reset by B
  int setRightCPL;     // set by A, processed and reset by B
  int currLeftCPL;     // set by B
  int currRightCPL;    // set by B
  int reset;           // set by A, reset by B
  float kp;            // set by A
  float ki;            // set by A
  float kd;            // set by A
};

dataPacket data;
esp_now_peer_info_t peerInfo;

void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  // copy received data for processing
  memcpy(&data, incomingData, sizeof(data));

  // disable encoder reading
  noInterrupts();

  // reset encoders if necessary
  if (data.reset != 0)
  {
    leftEncoderCount -= leftController.prevCount;
    leftController.prevCount = 0;
    rightEncoderCount -= rightController.prevCount;
    rightController.prevCount = 0;
    data.reset = 0;
  }
  
  // set left and right setpoints if necessary
  if (data.setLeftCPL != SETPOINT_RESET)
  {
    leftController.newSetpoint(data.setLeftCPL);
    data.setLeftCPL = SETPOINT_RESET;
  }
  if (data.setRightCPL != SETPOINT_RESET)
  {
    rightController.newSetpoint(data.setRightCPL);
    data.setRightCPL = SETPOINT_RESET;
  }

  // update PID controllers' outputs
  leftController.update();
  rightController.update();

  // update PID controllers' PID values
  leftController.setPID(data.kp, data.ki, data.kd);
  rightController.setPID(data.kp, data.ki, data.kd);

  // update CPL data from controllers
  data.currLeftCPL = leftController.currCPL;
  data.currRightCPL = rightController.currCPL;

  // re-enable encoder reading
  interrupts();

  // send processed data back to board A
  esp_err_t result = esp_now_send(A_MAC, (uint8_t *)&data, sizeof(data));

}

int ESPNowInit(char board)
{
  // Turn this board into a wifi station
  WiFi.mode(WIFI_STA);

  // initialize ESP now
  if (esp_now_init() != ESP_OK) return 1;

  // set the correct MAC address for peer
  if (board == 'b') memcpy(peerInfo.peer_addr, A_MAC, 6);
  if (board == 'a') memcpy(peerInfo.peer_addr, B_MAC, 6);

  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) return 2;

  // add received data callback
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));

  return 0;
}

void readRightEncoder() { 
  // Interrupt to update the right encoder count.
  if (!digitalRead(RB)) rightEncoderCount--;
  else rightEncoderCount++;
}

void readLeftEncoder() { 
  // Interrupt to update the left encoder count. (It's reversed because the motor is flipped)
  if(!digitalRead(LB)) leftEncoderCount++;
  else leftEncoderCount--;
}

void setup() 
{
  // wait for motor driver to turn on
  delay(3000);

  // setup pins and serial
  Serial2.begin(SERIAL_BAUD_RATE_B, SERIAL_8N1, 16, 17);
  pinMode(RA, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
  pinMode(LA, INPUT_PULLUP);
  pinMode(LB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RA), readRightEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(LA), readLeftEncoder, FALLING);
  pinMode(LV, OUTPUT);
  digitalWrite(LV, HIGH);

  // set initial data packet
  data.currLeftCPL = 0;
  data.currRightCPL = 0;
  data.setLeftCPL = SETPOINT_RESET;
  data.setRightCPL = SETPOINT_RESET;
  data.reset = 0;
  data.kp = Initial_KP;
  data.ki = Initial_KI;
  data.kd = Initial_KD;

  ESPNowInit('b');
}

void loop()
{
  // nothing happens in here. The motor control updating
  // happens when data is received. It is defined in
  // the receive data callback.
}