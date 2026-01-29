#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

static const uint32_t USB_BAUD = 115200;

static const int GPS_RX_PIN = 4; // arduino rx
static const int GPS_TX_PIN = 3; // arduino tx 

static const uint32_t GPS_BAUD = 9600;

static const int GPS_UPDATE_HZ = 1; //publish rate, 1hz is good for now

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
Adafruit_GPS GPS(&gpsSerial);

uint32_t lastFixMs = 0;

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
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {  
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }

    uint32_t now = millis();
    if (GPS.fix && (now - lastFixMs) > 200) {
      lastFixMs = now;
      print_json_fix();
    }
  }
}
