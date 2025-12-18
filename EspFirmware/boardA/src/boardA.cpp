
// This is the firmware for ESP32 board "A" connected to the PC.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include "../../common/config.h"
#include "../../common/comms.cpp"
#include <FlyskyIBUS.h>


// Global variables
bool status_YellowLightOn = false;
unsigned long status_YellowLastSwitched = 0;
esp_now_peer_info_t peerInfo;
bool is_connected = false;
FlyskyIBUS IBUS(Serial2, RC_IBUS_RX, 17);
std::atomic<float> leftAngvel{0.0};
std::atomic<float> rightAngvel{0.0};
std::atomic<unsigned long> time_of_last_data_receive{millis()};


// Receive data callback: runs whenever B sends wheel angvels to A
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  time_of_last_data_receive = millis();

  // copy recieved bytes into the struct so we can access/modify the data
  struct BtoAPacket received_data;
  if (len != sizeof(BtoAPacket)) return;
  memcpy(&received_data, incomingData, sizeof(BtoAPacket));

  // Clamp and update data
  if (received_data.currLeftAngvel > 99.99) received_data.currLeftAngvel = 99.99;
  else if (received_data.currLeftAngvel < -99.99) received_data.currLeftAngvel = -99.99;
  if (received_data.currRightAngvel > 99.99) received_data.currRightAngvel = 99.99;
  else if (received_data.currRightAngvel < -99.99) received_data.currRightAngvel = -99.99;
  leftAngvel = received_data.currLeftAngvel;
  rightAngvel = received_data.currRightAngvel;
}

// Send data callback: runs whenever A sends commands/setpoints to B
void sendDataCB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS) is_connected = false;
  else is_connected = true;
}


// Updates the data packet for autonomous mode and prints the latest wheel data to serial.
// Does nothing if there is no command, or the command is invalid.
void handle_ROS_command(struct AtoBPacket& data_to_send) 
{
  if (!Serial.available()) return;
  char chr = Serial.read();
  switch(chr) {

    case RESET_ENCODERS:
      data_to_send.reset = true;
      break;

    case ANGVEL_SETPOINT: {
      serialReadFloat(data_to_send.setLeftAngvel);
      serialReadFloat(data_to_send.setRightAngvel);
      break;
    }

    case SET_PID: {
      serialReadFloat(data_to_send.newKp);
      serialReadFloat(data_to_send.newKi);
      serialReadFloat(data_to_send.newKd);
      serialReadInt(data_to_send.gainChange);
      break;
    }
    default:
      return;
  }

  // print latest wheel data and mode if we still have connection to board B
  struct BtoAPacket received_data;
  if (millis() - time_of_last_data_receive < 500) {
    float left = leftAngvel;
    float right = rightAngvel;
    Serial.printf("%+06.2f %+06.2f %d\n", left, right, data_to_send.openLoop);
  }
  // lost connection
  else {
    leftAngvel = 0.0;
    rightAngvel = 0.0;
  }
}

// Updates the data packet for manual mode.
void handle_manual_commands(struct AtoBPacket& data_to_send)
{
  // get left and right efforts
  float x_normalized = (IBUS.getChannel(0) - 1500.0) / 500.0;
  float y_normalized = (IBUS.getChannel(1) - 1500.0) / 500.0;
  data_to_send.setLeftAngvel = y_normalized;
  data_to_send.setRightAngvel = y_normalized;
  data_to_send.setLeftAngvel += x_normalized;
  data_to_send.setRightAngvel -= x_normalized;

  // prevent ROS from doing anything
  data_to_send.gainChange = 0;
  data_to_send.reset = false;
}


void setup() 
{  
  // setup serial port and radio reciever data
  Serial.begin(SERIAL_BAUD_RATE_A);
  IBUS.begin();

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
  bool autonomous = (IBUS.getChannel(4) == 2000); // SWA on controller

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
  
  // handle motor control, ROS, and send data to board B
  struct AtoBPacket data_to_send{};
  data_to_send.openLoop = !autonomous;
  handle_ROS_command(data_to_send);
  if (data_to_send.openLoop) {
    handle_manual_commands(data_to_send);
    delay(10);
  }
  esp_now_send(B_MAC, (uint8_t*)&data_to_send, sizeof(data_to_send));

  

  // handle green light
  digitalWrite(GREEN_LIGHT, (is_connected ? HIGH : LOW));
}