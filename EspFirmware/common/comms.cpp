#ifndef COMMS_CPP
#define COMMS_CPP
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "config.h"

struct dataPacket 
{                        // data packet, shared between A and B
    int setLeftCPL;      // set by A, processed and reset by B
    int setRightCPL;     // set by A, processed and reset by B
    int currLeftCPL;     // set by B
    int currRightCPL;    // set by B
    int reset;           // set by A, reset by B
    float kp;            // set by A
    float ki;            // set by A
    float kd;            // set by A
};

void printAllData(dataPacket* data)
{
  Serial.print(data->currLeftCPL);
  Serial.print(data->currRightCPL);
  Serial.print(data->setLeftCPL);
  Serial.print(data->setRightCPL);
  Serial.print(data->reset);
  Serial.print(data->kp);
  Serial.print(data->ki);
  Serial.println(data->kd);
}

int ESPNowInit(char board, esp_now_peer_info* peerInfo)
{
    // Turn this board into a wifi station
    WiFi.mode(WIFI_STA);
  
    // initialize ESP now
    if (esp_now_init() != ESP_OK) return 1;
  
    // set the correct MAC address for peer
    if (board == 'b') memcpy(peerInfo->peer_addr, A_MAC, 6);
    if (board == 'a') memcpy(peerInfo->peer_addr, B_MAC, 6);
  
    // register peer
    peerInfo->channel = 0;  
    peerInfo->encrypt = false;
    if (esp_now_add_peer(peerInfo) != ESP_OK) return 2;
  
    // add received data callback
  
    return 0;
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

  #endif