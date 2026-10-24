
// This is the firmware for ESP32 board "A" connected to the PC.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include "../../common/config.h"
#include <FlyskyIBUS.h>

#define SERIAL_BUFFER_SIZE 30
#define SWITCH_READ_PERIOD_MS 100
#define MANUAL_CONTROL_RATE_MS 30 // for some reason, latency gets WORSE as you lower this. I have no idea why, maybe it's sending faster than board B can process it...?

bool yellowLightOn = false;
unsigned long yellowLightLastSwitched = millis();
unsigned long manualSwitchLastRead = millis();
unsigned long manualCommandLastSent = millis();
//unsigned long rosDataLastSent = millis();

std::atomic<bool> boardBConnected{};
std::atomic<bool> manualMode{};
std::atomic<float> leftAngvel{};
std::atomic<float> rightAngvel{};

FlyskyIBUS IBUS(Serial2, RX_RC_IBUS, TX_RC_IBUS);
esp_now_peer_info_t peerInfo;
char rosBuffer[SERIAL_BUFFER_SIZE];
int bufferIndex = 0;


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

// non-blocking function
// repeatedly calling this will keep appending new characters sent by ROS to the serial buffer.
// once we see a \n, the command is done. We only send it to board B if we're autonomous, but we always send data back to ROS when the command is done.
void handleROS() 
{
  while (Serial.available() > 0) {

  char inChar = Serial.read();
    
  // message finished
  if (inChar == '\n') {
    if (!manualMode) {
      struct AtoBPacket dataToSend{};
      dataToSend.openLoop = 0;
      dataToSend.ignorePacket = false;
      int valsRead = sscanf(rosBuffer, "s %f %f", &dataToSend.setLeftAngvel, &dataToSend.setRightAngvel);
      if (valsRead == 2) {
        esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
      }
      else {
        // TODO do something if the read command isn't valid
      }
    }
    float leftAngvel_tmp = leftAngvel;
    float rightAngvel_tmp = rightAngvel;
    bool boardBConnected_tmp = boardBConnected;
    bool manual_tmp = manualMode;
    Serial.printf("@%.2f %.2f %d %d\n", leftAngvel_tmp, rightAngvel_tmp, manual_tmp, boardBConnected_tmp);
    bufferIndex = 0;
    //rosDataLastSent = millis();
  } 

  // message in-progress
  else if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
    rosBuffer[bufferIndex] = inChar;
    bufferIndex++;
  }

  // message still in-progress but we overflowed the buffer. this should never happen
  else {
    bufferIndex = 0;

  }

}
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
  yellowLightLastSwitched = millis();
}


void loop() 
{
  // read from RC receiver
  // these lines MUST run every iteration of the loop; if they don't, the board sends data to ROS at irregular rates. 
  // I have no idea why.
  manualMode = IBUS.getChannel(4) != 2000;
  float manualXInput = (IBUS.getChannel(0) - 1500.0) / 500.0;
  float manualYInput = (IBUS.getChannel(1) - 1500.0) / 500.0;

  // handle status lights
  if (manualMode) {
    digitalWrite(YELLOW_LIGHT, HIGH);
    yellowLightOn = true;
  }
  else if ((millis() - yellowLightLastSwitched) > YELLOW_SWITCH_PERIOD_MS) {
    yellowLightOn = !yellowLightOn;
    digitalWrite(YELLOW_LIGHT, (yellowLightOn ? HIGH : LOW));
    yellowLightLastSwitched = millis();
  }
  digitalWrite(GREEN_LIGHT, (boardBConnected ? HIGH : LOW));

  // serial parsing and autonomous control
  handleROS();

  // manual control
  if (manualMode && (millis() - manualCommandLastSent > MANUAL_CONTROL_RATE_MS)) {
    struct AtoBPacket dataToSend{};
    dataToSend.openLoop = 1;
    dataToSend.setLeftAngvel = manualYInput + manualXInput;
    dataToSend.setRightAngvel = manualYInput - manualXInput;
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
    manualCommandLastSent = millis();
  }
}