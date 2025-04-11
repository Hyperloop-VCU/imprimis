#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"

#define DEBUG 1

// This is the firmware for ESP32 board "A" connected to the PC

unsigned long t0;

struct dataPacket { // data packet, shared between A and B
  int setLeftCPL;      // input to B
  int setRightCPL;     // input to B
  int currLeftCPL;     // readonly, updated by B
  int currRightCPL;    // readonly, updated by B
  int reset;           // set by A, then reset by B
  float kp;            // input to B
  float ki;            // input to B
  float kd;            // input to B
};

dataPacket data;
esp_now_peer_info_t peerInfo;

void printAllData() {
  Serial.println(data.setLeftCPL);
  Serial.print(" ");
  Serial.print(data.setRightCPL);
  Serial.print(" ");
  Serial.print(data.currLeftCPL);
  Serial.print(" ");
  Serial.print(data.currRightCPL);
  Serial.print(" ");
  Serial.print(data.reset);
  Serial.print(" ");
  Serial.print(data.kp);
  Serial.print(" ");
  Serial.print(data.ki);
  Serial.print(" ");
  Serial.print(data.kd);
  Serial.println();
}

void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));

  // pass along relevant data to PC
  if (DEBUG) {
    printAllData();
  }
  else {
  Serial.write((byte*)(&data.currLeftCPL), sizeof(int));
  Serial.write((byte*)(&data.currRightCPL), sizeof(int));
  }
}

void serialReadFloat(float &f) { 
  // Read a float from the serial input and store it in f.
  while (!Serial.available());
  f = Serial.parseFloat();
}

void serialReadInt(int &i) {
  // Read an int from the serial input and store it in i.
  while (!Serial.available());
  i = Serial.parseInt();
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

  // set the initial data packet
  data.currLeftCPL = 0;
  data.currRightCPL = 0;
  data.setLeftCPL = 0;
  data.setRightCPL = 0;
  data.reset = 0;
  data.kp = 0.057 / 2;
  data.ki = 0;
  data.kd = 0;

  // set up ESP now
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  memcpy(peerInfo.peer_addr, B_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));

  t0 = millis();
}

void loop() {
  doCommand();

  // exchange data every DT seconds. 
  if (millis() - t0 >= DT_MILLIS) {
    esp_err_t result = esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
    if (result != ESP_OK) Serial.println("Error sending data from A to B");
    t0 = millis();
  }

}

/*
void setManualMotorInput() { 
  // Get differential drive input from the RC receiver, and set the motor speeds.
  int linearInput = pulseIn(RC_MANUAL_LIN, HIGH, 30000); // 30 millisecond timeout
  int angularInput = pulseIn(RC_MANUAL_ANG, HIGH, 30000);
  if (linearInput < 950 || linearInput > 2050) { // ensure motor speeds are 0 if the RC signal is invalid
    linearInput = 1500;
  }
  if (angularInput < 950 || angularInput > 2050) {
    angularInput = 1500;
  }
  int linNormalized = map(linearInput, 1000, 2000, -255, 255);
  int angNormalized = map(angularInput, 1000, 2000, -255, 255);
  int leftManualInput = round(linNormalized + angNormalized * (1 - MANUAL_TURN_COEFF * linNormalized));
  int rightManualInput = round(linNormalized - angNormalized * (1 - MANUAL_TURN_COEFF * linNormalized));
  leftController.corrected_setSpeed(leftManualInput);
  rightController.corrected_setSpeed(rightManualInput);
}*/