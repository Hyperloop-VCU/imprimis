#include <math.h>
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"
#include "MotorController.cpp"

#define DEBUG 0

// This is the firmware for ESP32 board "B" connected to the motor driver and encoders
unsigned long t0;

int leftEncoderCount = 0, rightEncoderCount = 0;
MotorController leftController(17, Initial_KP, Initial_KI, Initial_KD, &leftEncoderCount, 0, COUNTS_PER_REV, DEBUG);
MotorController rightController(17, Initial_KP, Initial_KI, Initial_KD, &rightEncoderCount, 1, COUNTS_PER_REV, DEBUG);

dataPacket data = {0, 0, 0, 0, 0, Initial_KP, Initial_KI, Initial_KD};
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

  ESPNowInit('b', &peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));

  if (DEBUG) Serial.begin(9600);
  Serial2.begin(SERIAL_BAUD_RATE_B, SERIAL_8N1, 16, 17);
}

void loop()
{
  // nothing happens in here. The motor control updating
  // happens when data is received. It is defined in
  // the receive data callback.
}