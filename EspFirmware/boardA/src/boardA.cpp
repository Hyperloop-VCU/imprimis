#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"

#define DEBUG 1

// This is the firmware for ESP32 board "A" connected to the PC

unsigned long t0;

dataPacket data = {0, 0, 0, 0, 0, Initial_KP, Initial_KI, Initial_KD};
esp_now_peer_info_t peerInfo;

void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));

  // pass along relevant data to PC
  if (DEBUG) {
    printAllData(&data);
  }
  else {
  Serial.write((byte*)(&data.currLeftCPL), sizeof(int));
  Serial.write((byte*)(&data.currRightCPL), sizeof(int));
  }
}

void updateSetpoints(float setLinearX, float setAngularZ) {
  /* Use a twist message to calculate the counts-per-loop values for the PID controllers, and set them.
  Zero PID integrals to prevent error interference. */
  data.setLeftCPL = round(LINVEL_2_CPL * (setLinearX + setAngularZ * HALF_WHEEL_TRACK_LENGTH));
  data.setRightCPL = round(LINVEL_2_CPL * (setLinearX - setAngularZ * HALF_WHEEL_TRACK_LENGTH));
}

void resetEncoders() {
  data.reset = 1;
}

void updatePIDgains(float kp, float ki, float kd) {
  data.kp = kp;
  data.ki = ki;
  data.kd = kd;
}

void doCommand() {
  // Read the first character from serial data, and execute the command associated with that character.

  if (!Serial.available()) return; // do nothing if there is no serial data to read

  char chr = Serial.read();
  switch(chr) {

    case RESET_ENCODERS:
      resetEncoders();
      break;

    case TWIST_SETPOINT: {
      float receivedLinX, receivedAngZ;
      serialReadFloat(receivedLinX);
      serialReadFloat(receivedAngZ);
      updateSetpoints(receivedLinX, receivedAngZ);
      break;
    }

    case SET_PID: {
      float p, i, d;
      serialReadFloat(p);
      serialReadFloat(i);
      serialReadFloat(d);
      updatePIDgains(p, i, d);
    }
  }

}

void setup() {  

  Serial.begin(SERIAL_BAUD_RATE_A);

  ESPNowInit('a', &peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));

  t0 = millis();
}

void loop() {
  doCommand();

  if (millis() - t0 >= DT_MILLIS) 
  {
    // send over the data every DT seconds.
    // when B receives the data,
    // it processes any new motor setpoints,
    // resets the encoders if necessary,
    // updates the left and right CPL values,
    // then sends it back.
    esp_err_t result = esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
    t0 = millis();
  }

}