
// This is the firmware for ESP32 board "A" connected to the PC.

#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include "../../common/config.h"
#include <FlyskyIBUS.h>
#include <BNO055_support.h>
#include <Wire.h>
#include <Adafruit_GPS.h>


// Local to loop()
bool status_YellowLightOn = false;
unsigned long status_YellowLastSwitched = 0;
esp_now_peer_info_t peerInfo;

// Shared between tasks
std::atomic<bool> boardBConnected{};
std::atomic<bool> manualEnabled{};
std::atomic<float> imuHeading{};
std::atomic<float> leftAngvel{};
std::atomic<float> rightAngvel{};
std::atomic<bool> gpsFix{};
std::atomic<float> gpsLong{};
std::atomic<float> gpsLat{};
std::atomic<float> gpsAlt{};
std::atomic<float> gpsHdop{};

SemaphoreHandle_t sendDataMutex;

// Sensors
FlyskyIBUS IBUS(Serial2, RX_RC_IBUS, TX_RC_IBUS); // 15 is the unused TX pin
Adafruit_GPS GPS(&Serial1);
struct bno055_t IMU;

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
inline void send_pmtk(const char* cmd) {
  GPS.sendCommand(cmd);
  delay(100);
}

// Receive data callback: runs whenever B sends wheel angvels to A
void receiveDataCB(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  boardBConnected = true; // assume we will always be able to send data if we can receive it
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
  float imuHeading_tmp = imuHeading;
  bool gpsFix_tmp = gpsFix;
  float gpsLat_tmp = gpsLat;
  float gpsLong_tmp = gpsLong;
  float gpsAlt_tmp = gpsAlt;
  float gpsHdop_tmp = gpsHdop;
  
  Serial.printf(
    "%+06.2f %+06.2f %+07.2f %d %d %d %+011.7f %+011.7f %+06.2f %+06.2f\n", 
    leftAngvel_tmp, 
    rightAngvel_tmp, 
    imuHeading_tmp, 
    dataToSend.openLoop, 
    boardBConnected_tmp, 
    gpsFix_tmp, 
    gpsLat_tmp, 
    gpsLong_tmp, 
    gpsAlt_tmp, 
    gpsHdop_tmp
  );
  return true;
}


void readIMU(void* pvParameters)
{
  while (1) {
    BNO055_S16 readIMUHeading;
    bno055_read_euler_h(&readIMUHeading);
    imuHeading = static_cast<float>(readIMUHeading) / 16.00;
    vTaskDelay(pdMS_TO_TICKS(IMU_READ_PERIOD_MS));
  }
}


void readGPS(void* pvParameters)
{
  while (1) {
    (void)GPS.read();
    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) {
      gpsFix = GPS.fix;
      gpsLat = GPS.latitudeDegrees;
      gpsLong = GPS.longitudeDegrees;
      gpsAlt = GPS.altitude;   
      gpsHdop = GPS.HDOP;
    }
    vTaskDelay(pdMS_TO_TICKS(GPS_READ_PERIOD_MS));
  }
}

void confirmConnection(void* pvParameters) 
{
  while (1) {
    struct AtoBPacket testPacket{};
    testPacket.ignorePacket = true;
    (void)xSemaphoreTake(sendDataMutex, portMAX_DELAY);
    esp_now_send(B_MAC, (uint8_t*)&testPacket, 1);
    xSemaphoreGive(sendDataMutex);
    vTaskDelay(pdMS_TO_TICKS(CONNECTION_CHECK_PERIOD_MS));
  }
}


void setup() 
{  
  sendDataMutex = xSemaphoreCreateMutex(); // Mutex for sending data to board B
  Serial.begin(SERIAL_BAUD_RATE_A); // PC connection
  IBUS.begin(); // RC receiver connection

  // GPS
  Serial1.setPins(RX_GPS, TX_GPS);
  GPS.begin(SERIAL_BAUD_RATE_GPS);
  send_pmtk(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  send_pmtk(PMTK_SET_NMEA_UPDATE_10HZ);
  send_pmtk(PMTK_SET_FIX_CTL_10HZ);
  GPS.sendCommand(PGCMD_NOANTENNA); // TODO use antenna

  // IMU
  Wire.begin();
  BNO_Init(&IMU);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);

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

  xTaskCreate(
    readIMU, // function
    "readIMU_task", // task name
    8192, // stack size
    NULL, // params
    0, // priority
    NULL // task handle
  );
  xTaskCreate(
    readGPS,
    "readGPS_task",
    16384,
    NULL,
    0,
    NULL
  );
  xTaskCreate(
    confirmConnection,
    "confirmConnection_task",
    8192,
    NULL,
    0,
    NULL
  );
}


void loop() 
{
  bool controllerConnected = true; // TODO
  manualEnabled = controllerConnected ? (IBUS.getChannel(4) != 2000) : true; // SWA on controller

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
  // If manual, send manual commands to board B regardless of ROS command
  // If autonomous, send ROS command to board B only if it was received
  struct AtoBPacket dataToSend{};
  dataToSend.openLoop = manualEnabled;
  bool command_received = handle_ROS_command(dataToSend);
  if (!manualEnabled && command_received) {
    (void)xSemaphoreTake(sendDataMutex, portMAX_DELAY);
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
    xSemaphoreGive(sendDataMutex);
  }
  else if (manualEnabled) {
    float manualXInput = (IBUS.getChannel(0) - 1500.0) / 500.0;
    float manualYInput = (IBUS.getChannel(1) - 1500.0) / 500.0;
    dataToSend.setLeftAngvel = manualYInput - manualXInput;
    dataToSend.setRightAngvel = manualYInput + manualXInput;
    (void)xSemaphoreTake(sendDataMutex, portMAX_DELAY);
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));
    xSemaphoreGive(sendDataMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(1));
}