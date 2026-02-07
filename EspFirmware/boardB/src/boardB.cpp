// This is the firmware for ESP32 board "B" connected to the motor driver and encoders
// B recieves wheel setpoint data from A and runs a PID controller on each motor. The PID loop runs at a fixed rate.
// If we haven't received data for some time, we give both controllers a setpoint of '0' to stop the robot.


#include <math.h>
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include "../../common/config.h"
#include "MotorController.cpp"


// true to initialize the USB-C serial and print info to it
bool debugB = false;


// global variables
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
MotorController leftController(2.3 / 3, 6.2 / 3, 0.0, false, LEFT_COUNTS_PER_REV, debugB);
MotorController rightController(2.3 / 3, 6.2 / 3, 0.0, true, RIGHT_COUNTS_PER_REV, debugB);
esp_now_peer_info_t peerInfo;
std::atomic<unsigned long> time_of_last_data_receive{millis()};
unsigned long time_of_last_controller_update = millis();
unsigned long time_of_last_data_send = millis();
std::atomic<bool> openLoop{true};
std::atomic<bool> pauseUpdates{false};


// On-data-received callback, runs when board A sends data to board B.
void receiveDataCB(const uint8_t* mac, const uint8_t* incomingData, int len) 
{ 
  struct AtoBPacket received_data{};
  memcpy(&received_data, incomingData, sizeof(received_data));
  if (received_data.ignorePacket) return;
  time_of_last_data_receive = millis();
  
  if (received_data.reset || (openLoop != received_data.openLoop)) {
    pauseUpdates = true;
    leftController.reset();
    rightController.reset();
  }
  openLoop = received_data.openLoop;
  if (received_data.gainChange == 1) leftController.setPID(received_data.newKp, received_data.newKi, received_data.newKd);
  else if (received_data.gainChange == 2) rightController.setPID(received_data.newKp, received_data.newKi, received_data.newKd);
  leftController.newSetpoint(received_data.setLeftAngvel);
  rightController.newSetpoint(received_data.setRightAngvel);
  pauseUpdates = false;
}

// Interrupt to update the right encoder count.
void IRAM_ATTR readRightEncoder() 
{ 
  if (!digitalRead(RB)) rightEncoderCount--;
  else rightEncoderCount++;
}


// Interrupt to update the left encoder count. 
void IRAM_ATTR readLeftEncoder() 
{ 
  if(!digitalRead(LB)) leftEncoderCount++;
  else leftEncoderCount--; // reversed for some reason, I don't know why
}


void setup() 
{
  // setup serial
  Serial2.begin(SERIAL_BAUD_RATE_B, SERIAL_8N1, 16, 17);
  if (debugB) Serial.begin(DEBUG_BAUD_RATE_B);

  // initialize pins, attach interrputs to encoder pins
  pinMode(LA, INPUT_PULLUP);
  pinMode(RA, INPUT_PULLUP);
  pinMode(LB, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RA), readRightEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(LA), readLeftEncoder, FALLING);
  pinMode(LV, OUTPUT);
  digitalWrite(LV, HIGH);

  // setup ESP-now, register receive callback
  WiFi.mode(WIFI_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, A_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = true;
  esp_now_set_pmk((uint8_t*)PMK);
  for (uint8_t i = 0; i < 16; i++) peerInfo.lmk[i] = LMK[i];
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));
}


void loop() 
{
  // Stop the robot if we haven't received data for a while
  if (millis() - time_of_last_data_receive > TIMEOUT_MS) {
    if (debugB) Serial.println("Inactive - robot stopped.");
    leftController.reset();
    rightController.reset();
    leftController.setSpeed(0.0);
    rightController.setSpeed(0.0);
    delay(10);
    return;
  }

  // Update controllers at a fixed rate
  if (millis() - time_of_last_controller_update > PID_UPDATE_PERIOD_MS) {
    if (!pauseUpdates) {
      if (openLoop) {
        leftController.update_openloop(leftEncoderCount);
        rightController.update_openloop(rightEncoderCount);
      }
      else {
        leftController.update(leftEncoderCount);
        rightController.update(rightEncoderCount);
      }
      leftEncoderCount = 0;
      rightEncoderCount = 0;
    }
    time_of_last_controller_update = millis();
  }

  // Send data to board A at a fixed rate
  if (millis() - time_of_last_data_send > DATA_SEND_RATE_MS) {
    struct BtoAPacket data_to_send{};
    data_to_send.currLeftAngvel = leftController.getAngvel();
    data_to_send.currRightAngvel = rightController.getAngvel();
    (void*)esp_now_send(A_MAC, (uint8_t *)&data_to_send, sizeof(data_to_send));
    time_of_last_data_send = millis();
  }
}
