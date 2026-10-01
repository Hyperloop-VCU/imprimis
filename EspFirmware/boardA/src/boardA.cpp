
// This is the firmware for ESP32 board "A" connected to the PC.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <esp_wifi.h>
#include "../../common/config.h"
//#include <PS4Controller.h>

esp_now_peer_info_t peerInfo;

// Shared between tasks
std::atomic<bool> boardBConnected{};
std::atomic<bool> manualEnabled{true};
std::atomic<float> leftAngvel{};
std::atomic<float> rightAngvel{};
std::atomic<float> espnow_latency{};

float serial_parse_latency = 0.0;


// Utilities
inline void serialReadFloat(float& f)
{
  while (!Serial.available());
  f = Serial.parseFloat();
}
inline void serialReadInt(int& i)
{
  while (!Serial.available());
  i = Serial.parseInt();
}

// Receive data callback: runs whenever B sends wheel angvels to A
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  boardBConnected = true; // assume we will always be able to send data if we can receive it
  // copy recieved bytes into the struct so we can access/modify the data
  struct AtoBPacket received_data;
  if (len != sizeof(AtoBPacket)) return;
  memcpy(&received_data, incomingData, sizeof(AtoBPacket));

  // update atomics
  leftAngvel = received_data.setLeftAngvel;
  rightAngvel = received_data.setRightAngvel;
  espnow_latency = (float)((int32_t)esp_timer_get_time() - received_data.t_sent) / 1000;

  //float lt = espnow_latency;
  //Serial.printf("%f\n", lt);

}

// Send data callback: runs whenever A sends commands/setpoints to B
void sendDataCB(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS) boardBConnected = false;
  else boardBConnected = true;
}


// Updates the data to send based on the command sent and prints data to serial.
// Does nothing if there is an invalid command or no command.
// Returns true if a valid command was received, false otherwise.
bool handle_ROS_command(struct AtoBPacket& dataToSend) 
{
  if (!Serial.available()) return false;

  int64_t time_of_last_command_receive = esp_timer_get_time();
  char chr = Serial.read();
  switch(chr) {
    case ANGVEL_SETPOINT:
      serialReadFloat(dataToSend.setLeftAngvel);
      serialReadFloat(dataToSend.setRightAngvel);
      break;
    case RESET_ENCODERS:
      break;
    default:
      return false;
  }

  float leftAngvel_tmp = leftAngvel;
  float rightAngvel_tmp = rightAngvel;
  bool boardBConnected_tmp = boardBConnected;
  float latency_tmp = espnow_latency;
  bool openLoop_tmp = manualEnabled;
  
  Serial.printf(
    "@%.2f %.2f %.2f %.2f %d %d\n", 
    leftAngvel_tmp, 
    rightAngvel_tmp,
    latency_tmp / 2,
    serial_parse_latency,
    openLoop_tmp,
    boardBConnected_tmp
  );

  serial_parse_latency = (float)(esp_timer_get_time() - time_of_last_command_receive) / 1000;
  return true;
}


void setup() 
{  
  Serial.begin(SERIAL_BAUD_RATE_A); // PC connection

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(84);
  esp_now_init();
  memcpy(peerInfo.peer_addr, B_MAC, 6);
  peerInfo.channel = 0;  
  //peerInfo.encrypt = true;
  //esp_now_set_pmk((uint8_t *)PMK);
  //for (uint8_t i = 0; i < 16; i++) peerInfo.lmk[i] = LMK[i];
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));
  esp_now_register_send_cb(esp_now_send_cb_t(sendDataCB));

  // ps4
  //PS4.begin();
}


void loop() 
{
  manualEnabled = false;

  /* handle status lights
  if (manualEnabled) {
    digitalWrite(YELLOW_LIGHT, HIGH);
    status_YellowLightOn = true;
  }
  else if ((millis() - status_YellowLastSwitched) > YELLOW_SWITCH_PERIOD_MS) {
    status_YellowLightOn = !status_YellowLightOn;
    digitalWrite(YELLOW_LIGHT, (status_YellowLightOn ? HIGH : LOW));
    status_YellowLastSwitched = millis();
  }
  
  digitalWrite(GREEN_LIGHT, (boardBConnected ? HIGH : LOW));
  */

  // Main board A logic
  // If a ROS command was received, send data back to ROS, regardless of mode
  // If manual, send command to board B regardless of what ROS is doing
  // If autonomous, send command to board B only if ROS sent one
  struct AtoBPacket dataToSend{};
  
  bool command_received = handle_ROS_command(dataToSend);

  if (!manualEnabled && command_received) {
    dataToSend.t_sent = (int32_t)esp_timer_get_time();
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
  }

  /*
  else if (manualEnabled) {
    float manualXInput = 0.0;
    float manualYInput = 0.0;
    if (PS4.isConnected()) {
      manualXInput = (PS4.RStickX() - 0.0) / 127.0;
      manualYInput = (PS4.RStickY() - 0.0) / 127.0;
    }

    dataToSend.setLeftAngvel = manualYInput + manualXInput;
    dataToSend.setRightAngvel = manualYInput - manualXInput;

    dataToSend.t_sent = esp_timer_get_time();
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
  }
  */
}