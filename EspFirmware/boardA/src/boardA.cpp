
// This is the firmware for ESP32 board "A" connected to the PC.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include "../../common/config.h"
#include <FlyskyIBUS.h>


// Local to loop()
bool status_YellowLightOn = false;
unsigned long status_YellowLastSwitched = 0;
esp_now_peer_info_t peerInfo;

// Shared between tasks
std::atomic<bool> boardBConnected{};
std::atomic<bool> manualEnabled{};
std::atomic<float> leftAngvel{};
std::atomic<float> rightAngvel{};

// RC receiver
FlyskyIBUS IBUS(Serial2, RX_RC_IBUS, TX_RC_IBUS); // TX is unused

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
  struct BtoAPacket received_data;
  if (len != sizeof(BtoAPacket)) return;
  memcpy(&received_data, incomingData, sizeof(BtoAPacket));

  // update atomics
  leftAngvel = received_data.currLeftAngvel;
  rightAngvel = received_data.currRightAngvel;
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

  char chr = Serial.read();
  switch(chr) {
    case RESET_ENCODERS:
      dataToSend.reset = true;
      break;
    case ANGVEL_SETPOINT:
      serialReadFloat(dataToSend.setLeftAngvel);
      serialReadFloat(dataToSend.setRightAngvel);
      break;
    case SET_PID:
      serialReadFloat(dataToSend.newKp);
      serialReadFloat(dataToSend.newKi);
      serialReadFloat(dataToSend.newKd);
      serialReadInt(dataToSend.gainChange);
      break;
    default:
      return false;
  }

  float leftAngvel_tmp = leftAngvel;
  float rightAngvel_tmp = rightAngvel;
  bool boardBConnected_tmp = boardBConnected;
  
  Serial.printf(
    "@%.2f %.2f %d %d %d %d %.7f %.7f %.2f %.2f\n", 
    leftAngvel_tmp, 
    rightAngvel_tmp, 
    0, // imu heading, removed
    dataToSend.openLoop, 
    boardBConnected_tmp, 
    0, 
    0, 
    0, 
    0, 
    0
  );
  return true;
}


void setup() 
{  
  Serial.begin(SERIAL_BAUD_RATE_A); // PC connection
  IBUS.begin(); // RC receiver connection

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  esp_now_init();
  memcpy(peerInfo.peer_addr, B_MAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = true;
  esp_now_set_pmk((uint8_t *)PMK);
  for (uint8_t i = 0; i < 16; i++) peerInfo.lmk[i] = LMK[i];
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receiveDataCB));
  esp_now_register_send_cb(esp_now_send_cb_t(sendDataCB));

  // Status lights
  pinMode(GREEN_LIGHT, OUTPUT);
  pinMode(YELLOW_LIGHT, OUTPUT);
  status_YellowLastSwitched = millis();
}


void loop() 
{
  manualEnabled = IBUS.getChannel(4) != 2000;

  // handle status lights
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

  // Main board A logic
  // If a ROS command was received, send data back to ROS, regardless of mode
  // If manual, send command to board B regardless of what ROS is doing
  // If autonomous, send command to board B only if ROS sent one
  struct AtoBPacket dataToSend{};
  dataToSend.openLoop = manualEnabled;
  bool command_received = handle_ROS_command(dataToSend);
  if (!manualEnabled && command_received) {
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
  }
  else if (manualEnabled) {
    float manualXInput = (IBUS.getChannel(0) - 1500.0) / 500.0;
    float manualYInput = (IBUS.getChannel(1) - 1500.0) / 500.0;
    dataToSend.setLeftAngvel = manualYInput - manualXInput;
    dataToSend.setRightAngvel = manualYInput + manualXInput;
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
  }
}