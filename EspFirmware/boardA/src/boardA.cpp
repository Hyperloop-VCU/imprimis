
// This is the firmware for ESP32 board "A" connected to the PC.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"


// Global variables
const short status_greenPin = 25;
const short status_yellowPin = 26;
int status_YellowMilliseconds = 1000;
bool status_YellowLightOn = false;
unsigned long status_YellowLastSwitched = 0;
unsigned long time_of_last_command_recieve = millis();
unsigned long t0;
dataPacket data{}; // initial data
esp_now_peer_info_t peerInfo;
bool is_connected = false;


// Receive data callback: runs whenever board B sends data to board A (when a velocity command appears)
// Sets board A's local data to what board B sent it, then passes along
// the position and velocity of each motor to the PC.
// Whenever A sends data to B, B does its processing, updates the data if necessary,
// then sends the data back to A.
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&data, incomingData, sizeof(data));
  float x = 0.1;
  Serial.write((byte*)(&x), sizeof(float));
  Serial.write((byte*)(&x), sizeof(float));
}


// Send data callback: runs whenever board A sends data to board B.
// Ensures board B received the data. Handles retry and is_connected logic.
void sendDataCB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS) is_connected = false;
  else is_connected = true;
}


// Reads the first character from serial data and executes the command associated with that character.
// Does nothing if there is no serial data to read
// command RESET_ENCODERS: Resets the encoder counts and PID integral to 0.
// command ANGVEL_SETPOINT: Gives a new angular-velocity setpoint to the PID controllers.
// command SET_PID: Updates the pid gains of the controllers. Sets both the left and right wheel controllers to the same gain.
void doCommand() 
{

  if (!Serial.available()) return;
  time_of_last_command_recieve = millis();
  char chr = Serial.read();
  switch(chr) {

    case RESET_ENCODERS:
      data.reset = 1;
      break;

    case ANGVEL_SETPOINT: {
      serialReadFloat(data.setLeftAngvel);
      serialReadFloat(data.setRightAngvel);
      // send data to B
      esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
      break;
    }

    case SET_PID: {
      serialReadFloat(data.newKp);
      serialReadFloat(data.newKi);
      serialReadFloat(data.newKd);
      serialReadInt(data.gainChange);
      // send data to B
      esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
      break;
    }

    default:
      break;
  }
}


void setup() 
{  
  delay(100);
  Serial.begin(SERIAL_BAUD_RATE_A);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, B_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));
  esp_now_register_send_cb(esp_now_send_cb_t(sendDataCB));

  pinMode(status_greenPin, OUTPUT);
  pinMode(status_yellowPin, OUTPUT);
  // Get the current time
  status_YellowLastSwitched = millis();
}


void loop() 
{
  // Digital Write to pins 25-Green 26-Yellow
  if ((millis() - status_YellowLastSwitched) > status_YellowMilliseconds) {
    status_YellowLightOn = !status_YellowLightOn;
    digitalWrite(status_yellowPin, (status_YellowLightOn ? HIGH : LOW));
    status_YellowLastSwitched = millis();
  }
  
  doCommand();

  if (millis() - time_of_last_command_recieve > 500) digitalWrite(status_greenPin, LOW);
  else digitalWrite(status_greenPin, (is_connected ? HIGH : LOW));
}