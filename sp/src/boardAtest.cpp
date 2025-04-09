#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"

#define DEBUG 1

// This is the firmware for ESP32 board "A" connected to the PC

unsigned long t0;

struct dataPacket // data packet, shared between A and B
{
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

void printAllData() 
{
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

void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&data, incomingData, sizeof(data));
  printAllData();
}

void setup() 
{  

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

void loop() 
{
  // exchange data every DT seconds. 
  if (millis() - t0 >= DT_MILLIS) 
  {
    data.currLeftCPL += 1;
    esp_err_t result = esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
    if (result != ESP_OK) Serial.println("Error sending data from A to B");
  }

}