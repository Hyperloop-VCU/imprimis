
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <esp_wifi.h>
#include "../../common/config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// screen config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// visualization: plot size
#define PLOT_X_END 110
#define PLOT_Y_END 62
#define PLOT_X_START 18
#define PLOT_Y_START 10

// visualization: connection / average report positions
#define AVG_REPORT_YOFF 5

// visualization: plot size
#define PLOT_X_END 110
#define PLOT_Y_END 62
#define PLOT_X_START 18
#define PLOT_Y_START 10

// visualization: y-axis ticks, labels, and scaling
#define PLOT_YLIMIT_MS 30
#define PLOT_NUM_TICKS 2
#define PLOT_TICK_LABEL_XOFF 15

// visualization: x-axis scaling
#define PLOT_XAXIS_NUM_DATAPOINTS 20
#define PLOT_XAXIS_TIME_BETWEEN_POINTS_MS 100

// pins
#define SCL 12
#define SDA 14
#define VRX 27
#define VRY 26

// display stuff
float plotBuffer[PLOT_XAXIS_NUM_DATAPOINTS + 1];
float* startPtr = plotBuffer;
float* endPtr = startPtr;

// struct telling us who board B is
esp_now_peer_info_t peerInfo;

// variables updated when wifi callbacks run
std::atomic<bool> boardBConnected{};
std::atomic<float> leftAngvel{};
std::atomic<float> rightAngvel{};
std::atomic<float> avgLatency{};

// ----------------------------------

void receive_data_cb(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  boardBConnected = true;
  struct AtoBPacket received_data;
  if (len != sizeof(AtoBPacket)) {
    return;
  }
  memcpy(&received_data, incomingData, sizeof(AtoBPacket));
  leftAngvel = received_data.setLeftAngvel;
  rightAngvel = received_data.setRightAngvel;
  //espnowLatency = (float)((int32_t)esp_timer_get_time() - received_data.t_sent) / 1000;
}

void send_data_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS) boardBConnected = false;
  else boardBConnected = true;
}

// ----------------------------------

int get_normalized_joy_x() {

  return analogRead(VRX);
}

int get_normalized_joy_y() {
  return analogRead(VRY);
}

// ----------------------------------

void draw_plot_axes_lines_and_ticks() {
  display.drawLine(
    PLOT_X_START, PLOT_Y_START, 
    PLOT_X_START, PLOT_Y_END, 
    WHITE
  );
  display.drawLine(
    PLOT_X_START, PLOT_Y_END, 
    PLOT_X_END, PLOT_Y_END, 
    WHITE
  );
  int pixelTickInterval = (int)round((PLOT_Y_END - PLOT_Y_START) / PLOT_NUM_TICKS);

  for (int i = 1; i <= PLOT_NUM_TICKS; i++) {
    int tickY = PLOT_Y_END - i * pixelTickInterval;
    display.setCursor(PLOT_X_START - PLOT_TICK_LABEL_XOFF, tickY);
    int msValue = (int)round(PLOT_YLIMIT_MS * i / PLOT_NUM_TICKS);
    display.printf("%d", msValue);
  }
}

void draw_avg_report(int val) {
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(
    (PLOT_X_END - PLOT_X_START) / 2, 
    PLOT_Y_START - AVG_REPORT_YOFF
  );
  float tempLatency = avgLatency;
  if (!boardBConnected) {
    display.printf("Average latency: %.2fms", val);
  }
  else {
    display.printf("NOT CONNECTED");
  }
}

// -----------------------

void draw_plot_curve() {
  if (!boardBConnected) {
    // zero buffer
    return;
  }

  return;
}

// -----------------------

void setup() 
{
  // joystick pins
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);
  
  // SSD1306 Display
  Wire.begin(SDA, SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) {
    while (1) {
      delay(1000);
    }
  }
  display.setRotation(0);
  display.clearDisplay();
  draw_avg_report(0);
  draw_plot_axes_lines_and_ticks();
  display.display();

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(11, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(84);
  esp_now_init();
  memcpy(peerInfo.peer_addr, B_MAC, 6);
  peerInfo.channel = 0;  
  esp_now_add_peer(&peerInfo);
  esp_now_register_recv_cb(esp_now_recv_cb_t(receive_data_cb));
  esp_now_register_send_cb(esp_now_send_cb_t(send_data_cb));
}


void loop() 
{

  // read stick and send command to B
  struct AtoBPacket dataToSend{};
  float manualXInput = get_normalized_joy_x();
  float manualYInput = get_normalized_joy_y();
  dataToSend.setLeftAngvel = manualYInput + manualXInput;
  dataToSend.setRightAngvel = manualYInput - manualXInput;
  dataToSend.t_sent = (int32_t)esp_timer_get_time();
  esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));

  // update display
  display.clearDisplay();
  draw_plot_axes_lines_and_ticks();
  draw_avg_report(manualXInput);
  draw_plot_curve();
  display.display();
}