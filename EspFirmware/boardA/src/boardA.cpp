
#include <esp_now.h>
#include <Arduino.h>
#include <WiFi.h>
#include <atomic>
#include <esp_wifi.h>
#include "../../common/config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// screen config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// visualization: connection / average report positions
#define AVG_REPORT_YOFF 5

// visualization: plot size and positioning
#define PLOT_X_END 114
#define PLOT_Y_END 62
#define PLOT_X_START 22
#define PLOT_Y_START 10
#define PLOT_DATA_XOFF 0
#define LOST_CONNECTION_XOFF 10
#define LOST_CONNECTION_YOFF 20

// visualization: y-axis ticks, labels, and scaling
const int PLOT_SCALES_ARR[3] = {5, 10, 30};
const int NUM_PLOT_SCALES = 3;
#define PLOT_NUM_TICKS 2
#define PLOT_TICK_LABEL_XOFF 20
#define TICK_TEXT_YOFF 4
#define TICK_LENGTH 4

// sample buffer size
#define PLOT_XAXIS_NUM_DATAPOINTS 20

// deadzones for stick, 0 -> 4095
#define X_LEFT_START 1760
#define X_RIGHT_START 1900
#define Y_BOTTOM_START 1900
#define Y_TOP_START 1740
#define STICK_MAX 4095

// display modes
#define LATENCY_PLOT 0
#define WHEELS 1

#define AVG_MAX 4 

// pins
#define SCL 18
#define SDA 19
#define SW 25
#define VRX 33
#define VRY 32

// rates
#define DISPLAY_UPDATE_PERIOD_MS 30
#define ESP_NOW_SEND_PERIOD_MS 20
#define JOY_SWITCH_READ_PERIOD_MS 100 // don't care about interrupts

// ratekeeping
unsigned long displayLastUpdated = millis();
unsigned long dataLastSent = millis();
unsigned long switchLastRead = millis();

// board B information (so we can connect to it)
esp_now_peer_info_t peerInfo;

// variables updated when wifi callbacks run
std::atomic<bool> boardBConnected{};
std::atomic<float> leftAngvel{};
std::atomic<float> rightAngvel{};
std::atomic<float> espnowLatency{};

// data buffer for plotting latency measurements
float latencies[PLOT_XAXIS_NUM_DATAPOINTS]{};
int latenciesPtr = 0;

// switched by the joystick
int displayMode = LATENCY_PLOT;
int PLOT_YLIMIT_MS = PLOT_SCALES_ARR[0];
int plotScalePtr = 0;
int switchHeld = 0;

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
  espnowLatency = ((float)esp_timer_get_time() - (float)received_data.t_sent) / 1000;
}

void send_data_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if (status != ESP_NOW_SEND_SUCCESS) boardBConnected = false;
  else boardBConnected = true;
}

// ----------------------------------

float get_normalized_joy_x() 
{
  float value = analogRead(VRX);
  if ((value > X_LEFT_START && value < X_RIGHT_START) || (value < 0) || (value > STICK_MAX)) {
    return 0;
  }
  else if (value < X_LEFT_START) {
    return (value - X_LEFT_START) / X_LEFT_START;
  }
  else if (value > X_RIGHT_START) {
    float range = STICK_MAX - X_RIGHT_START;
    return (value - X_RIGHT_START) / range;
  }
  return 0; // should never happen
}

float get_normalized_joy_y() 
{
  float value = analogRead(VRY);
  if ((value > Y_TOP_START && value < Y_BOTTOM_START) || (value < 0) || (value > STICK_MAX)) {
    return 0;
  }
  else if (value < Y_TOP_START) {
    return (value - Y_TOP_START) / Y_TOP_START;
  }
  else if (value > Y_BOTTOM_START) {
    float range = STICK_MAX - Y_BOTTOM_START;
    return (value - Y_BOTTOM_START) / range;
  }
  return 0; // should never happen
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
    display.drawRect(PLOT_X_START + TICK_LENGTH/2 - 1, tickY, TICK_LENGTH, 1, WHITE);
    display.setTextSize(1);
    display.setCursor(PLOT_X_START - PLOT_TICK_LABEL_XOFF, tickY - TICK_TEXT_YOFF);
    float msValue = (float)PLOT_YLIMIT_MS * (float)i / PLOT_NUM_TICKS;
    if (msValue < 3.0) { // yeah yeah I know
      display.printf("%.1f", msValue);
    }
    else {
      display.printf("%d", (int)round(msValue));
    }
  }
}

int index_to_X(int index) {
  int d = (PLOT_X_END - (PLOT_X_START + PLOT_DATA_XOFF)) / PLOT_XAXIS_NUM_DATAPOINTS;
  return PLOT_DATA_XOFF + PLOT_X_START + d*index;
}

int latency_to_Y(float latency) {
  if (latency < 0) {
    return PLOT_Y_END;
  }
  if (latency > PLOT_YLIMIT_MS) {
    return PLOT_Y_START;
  }
  float value = latency / PLOT_YLIMIT_MS; // 0-1
  return PLOT_Y_START + round((1 - value) * (PLOT_Y_END - PLOT_Y_START));
}

void draw_avg_report_and_curve(bool connected) {
  
  // cursor formatting for printing text
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  // not connected case: don't draw the curve, just indicate the lost connection
  if (!connected) {
    display.setCursor(
      PLOT_X_START + LOST_CONNECTION_XOFF, 
      PLOT_Y_START + LOST_CONNECTION_YOFF
    );
    display.printf("LOST CONNECTION");
    return;
  }

  // latency curve
  int lastX = PLOT_X_START + PLOT_DATA_XOFF;
  int lastY = PLOT_Y_END;
  float avg = 0;
  int i;

  for (i = 0; i <= latenciesPtr; i++) {
    int newX = index_to_X(i);
    int newY = latency_to_Y(latencies[i]);

    display.drawLine(
      lastX, lastY,
      newX, newY,
      WHITE
    );

    if (i > latenciesPtr + 1 - AVG_MAX) {
      avg += latencies[i];
    }
    lastX = newX;
    lastY = newY;
  }
  
  // average
  avg /= AVG_MAX; // I know this isn't technically correct for the first few data points
  display.setCursor(
    (PLOT_X_END - PLOT_X_START) / 2, 
    PLOT_Y_START - AVG_REPORT_YOFF
  );
  display.printf("Avg: %.2f", avg);
}

// -----------------------

void sample_latency(bool connected) {

  // if connected, add a sample to the buffer and increment the pointer
  if (connected) {
    latenciesPtr = (latenciesPtr + 1) % PLOT_XAXIS_NUM_DATAPOINTS;
    latencies[latenciesPtr] = espnowLatency; // don't care about all the other measurements. I'm only sampling at this rate
  }
  // if disconencted, zero the latency buffer and reset the pointer
  else {
    for (int i = 0; i < PLOT_XAXIS_NUM_DATAPOINTS; i++) {
      latencies[i] = 0;
    }
    latenciesPtr = 0;
  }
}

void setup() 
{

  //Serial.begin(115200); // please remove

  // joystick pins
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);
  pinMode(SW, INPUT);
  
  // SSD1306 Display, enter infinite loop if it fails
  Wire.begin(SDA, SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) {
    Serial.begin(115200);
    while (1) {
      delay(1000);
      Serial.printf("Error initializing display");
    }
  }
  display.setRotation(0);
  display.clearDisplay();
  draw_plot_axes_lines_and_ticks();
  display.display();

  //ESP-NOW
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
  if (millis() - dataLastSent > ESP_NOW_SEND_PERIOD_MS) {
    struct AtoBPacket dataToSend{};
    float manualYInput = get_normalized_joy_x();
    float manualXInput = get_normalized_joy_y();
    dataToSend.setLeftAngvel = (manualYInput + manualXInput) * 2 * PI;
    dataToSend.setRightAngvel = (manualYInput - manualXInput) * 2 * PI;
    dataToSend.t_sent = (int32_t)esp_timer_get_time();
    esp_now_send(B_MAC, (uint8_t*)&dataToSend, sizeof(AtoBPacket));

    dataLastSent = millis();
  }
  
  // sample latency and update display
  if (millis() - displayLastUpdated > DISPLAY_UPDATE_PERIOD_MS) {
    display.clearDisplay();
    draw_plot_axes_lines_and_ticks();

    bool connected = boardBConnected;
    sample_latency(connected);
    draw_avg_report_and_curve(connected);
    
    display.display();
    displayLastUpdated = millis();
    //float l = espnowLatency;
    //Serial.printf("%f\n", l);
  }

  // read switch
  if (millis() - switchLastRead > JOY_SWITCH_READ_PERIOD_MS) {
    int switchPressed = digitalRead(SW);
    
    if (switchPressed && !switchHeld) {
      plotScalePtr = (plotScalePtr + 1) % NUM_PLOT_SCALES;
      PLOT_YLIMIT_MS = PLOT_SCALES_ARR[plotScalePtr];
    }

    switchHeld = switchPressed;
    switchLastRead = millis();
  }
}