#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

static const uint32_t USB_BAUD = 115200;
static const int GPS_RX_PIN = 4;
static const int GPS_TX_PIN = 3;
static const uint32_t GPS_BAUD = 9600;
static const int GPS_UPDATE_HZ = 1; // Rate at which the GPS module outputs data
static const int IMU_SAMPLE_RATE_MS = 10; // Rate at which to read data from the IMU
static const int MSG_PUBLISH_RATE_MS = 200; // Rate at which to send JSON data over serial

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
Adafruit_GPS GPS(&gpsSerial);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

uint32_t lastFixMs = millis();
uint32_t lastHeadingMs = millis();
sensors_event_t imuOrientationData;

// Sends GPS and heading data over serial as a JSON message. Will be parsed by the gps_nav_bridge package.
static void print_json_fix() {
  float lat = GPS.latitudeDegrees;
  float lon = GPS.longitudeDegrees;
  float alt_m = GPS.altitude;
  float speed_ms = GPS.speed * 0.514444f; // speed in knots converted to m/s
  float track_deg = GPS.angle; // course over ground in degrees
  int sats = GPS.satellites;
  float hdop = GPS.HDOP; // hdop (below 1 is best but 1-2 is okay)
  int fix = (int)GPS.fix; // fix quality, 0 = invalid, 1 = gps fix, 2 = dgps
  int fixquality = (int)GPS.fixquality; // gps time if available
  int hour = GPS.hour;
  int minute = GPS.minute;
  int seconds = GPS.seconds;
  int milliseconds = GPS.milliseconds;
  float heading = imuOrientationData.orientation.heading;

  // json
  Serial.print("{\"fix\":"); Serial.print(fix);
  Serial.print(",\"fixquality\":"); Serial.print(fixquality);
  Serial.print(",\"lat\":"); Serial.print(lat, 7);
  Serial.print(",\"lon\":"); Serial.print(lon, 7);
  Serial.print(",\"alt\":"); Serial.print(alt_m, 2);
  Serial.print(",\"speed_ms\":"); Serial.print(speed_ms, 3);
  Serial.print(",\"track_deg\":"); Serial.print(track_deg, 2);
  Serial.print(",\"sats\":"); Serial.print(sats);
  Serial.print(",\"hdop\":"); Serial.print(hdop, 2);
  Serial.print(",\"heading\":"); Serial.print(heading, 2);
  Serial.print(",\"gps_time\":\"");

  if (GPS.fix) {
    if (hour < 10) Serial.print("0"); Serial.print(hour); Serial.print(":");
    if (minute < 10) Serial.print("0"); Serial.print(minute); Serial.print(":");
    if (seconds < 10) Serial.print("0"); Serial.print(seconds); Serial.print(".");
    if (milliseconds < 100) Serial.print("0");
    if (milliseconds < 10) Serial.print("0");
    Serial.print(milliseconds);
  } else {
    Serial.print("00:00:00.000");
  }

  Serial.print("\"");
  Serial.println("}");
}

void setup() {
  Serial.begin(USB_BAUD);
  while (!Serial) { delay(10); }

  if (!bno.begin()) {
    // Handle bno initialization failure
  }

  gpsSerial.begin(GPS_BAUD);
  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  if (GPS_UPDATE_HZ == 1) GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  else if (GPS_UPDATE_HZ == 5) GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  else GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA); // attenna for future

  delay(1000);
  Serial.println("{\"status\":\"gps_bridge_ready\"}");
}

void loop() {

  // Read IMU
  if (millis() - lastHeadingMs > IMU_SAMPLE_RATE_MS) {
    bno.getEvent(&imuOrientationData, Adafruit_BNO055::VECTOR_EULER);
    lastHeadingMs = millis();
  }

  // Read GPS and send JSON
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {  
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }

    uint32_t now = millis();
    if (GPS.fix && (now - lastFixMs) > MSG_PUBLISH_RATE_MS) {
      lastFixMs = now;
      print_json_fix();
    }
  }
}
