#include <math.h>
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"

// This is the firmware for ESP32 board "B" connected to the motor driver and encoders
unsigned long t0;

struct dataPacket { // data packet, shared between A and B
  int setLeftCPL;      // readonly, updated by A
  int setRightCPL;     // readonly, updated by A
  int currLeftCPL;     // output to A
  int currRightCPL;    // output to A
  int reset;           // set by A, reset by B
  float kp;            // readonly, updated by A
  float ki;            // readonly, updated by A
  float kd;            // readonly, updated by 
};

dataPacket data;
esp_now_peer_info_t peerInfo;

void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&data, incomingData, sizeof(data));

  if (data.reset)
  {
    data.reset = 0;
  }
  if (data.currLeftCPL > 10)
  {
    data.currLeftCPL = 0;
  }
}

void setup() 
{

  // initial data packet
  data.currLeftCPL = 0;
  data.currRightCPL = 0;
  data.setLeftCPL = 0;
  data.setRightCPL = 0;
  data.reset = 0;
  data.kp = 0.057 / 2;
  data.ki = 0;
  data.kd = 0;

  // setup ESP now
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  memcpy(peerInfo.peer_addr, A_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));

  t0 = millis();
}

void loop()
{
  if (millis() - t0 >= DT_MILLIS)
  {
    esp_err_t result = esp_now_send(A_MAC, (uint8_t *)&data, sizeof(data));
  }
}