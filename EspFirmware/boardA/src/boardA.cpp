
// This is the firmware for ESP32 board "A" connected to the PC.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include "../../common/config.h"
#include "../../common/comms.cpp"
#include <IBusBM.h>


// Global variables
bool status_YellowLightOn = false;
unsigned long status_YellowLastSwitched = 0;
unsigned long time_of_last_command_recieve = millis();
unsigned long t0;
dataPacket data{};
esp_now_peer_info_t peerInfo;
bool is_connected = false;
bool manual_mode = true;
IBusBM IBus;


// Receive data callback: runs whenever B sends data to A
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  memcpy(&data, incomingData, sizeof(data));
  Serial.write((byte*)(&data.currLeftAngvel), sizeof(float));
  Serial.write((byte*)(&data.currRightAngvel), sizeof(float));
}


// Send data callback: runs whenever A sends data to B
void sendDataCB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS) is_connected = false;
  else is_connected = true;
}


// Reads the first character from serial data and executes the command associated with that character.
// Does nothing if there is no serial data to read
// command RESET_ENCODERS: Set data.reset to true, does not send to board B right away
// command ANGVEL_SETPOINT: Send new angular velocity setpoints to board B
// command SET_PID: Send new PID gains to board B
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
      esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
      break;
    }

    case SET_PID: {
      serialReadFloat(data.newKp);
      serialReadFloat(data.newKi);
      serialReadFloat(data.newKd);
      serialReadInt(data.gainChange);
      esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
      break;
    }

    default:
      break;
  }
}

bool is_autonomous()
{
  return IBus.readChannel(4) == 2000; // SWA on controller
}

// reads from the appropriate channels on the RC reciever and sends the right data to board B.
void handle_manual_input()
{
  float x_normalized = (IBus.readChannel(0) - 1500.0) / 500.0;
  float y_normalized = (IBus.readChannel(1) - 1500.0) / 500.0;
  data.setLeftAngvel = y_normalized;
  data.setRightAngvel = y_normalized;
  data.setLeftAngvel += x_normalized;
  data.setRightAngvel -= x_normalized;
  esp_now_send(B_MAC, (uint8_t *)&data, sizeof(data));
}


void setup() 
{  
  // setup serial port and radio reciever data
  Serial.begin(SERIAL_BAUD_RATE_A);
  IBus.begin(Serial2, 1, RC_IBUS_RX, -1);

  // setup ESP-now and callbacks
  WiFi.mode(WIFI_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, B_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));
  esp_now_register_send_cb(esp_now_send_cb_t(sendDataCB));

  // setup status lights
  pinMode(GREEN_LIGHT, OUTPUT);
  pinMode(YELLOW_LIGHT, OUTPUT);
  status_YellowLastSwitched = millis();
}


void loop() 
{
  bool autonomous = is_autonomous();

  // handle yellow light
  if (!autonomous) {
    digitalWrite(YELLOW_LIGHT, HIGH);
    status_YellowLightOn = true;
  }
  else if ((millis() - status_YellowLastSwitched) > YELLOW_SWITCH_PERIOD_MS) {
    status_YellowLightOn = !status_YellowLightOn;
    digitalWrite(YELLOW_LIGHT, (status_YellowLightOn ? HIGH : LOW));
    status_YellowLastSwitched = millis();
  }
  
  // handle motor control
  if (autonomous) {
    data.openLoop = false;
    doCommand();
  }
  else {
    data.openLoop = true;
    handle_manual_input();
  }

  // handle green light
  digitalWrite(GREEN_LIGHT, (is_connected ? HIGH : LOW));
}