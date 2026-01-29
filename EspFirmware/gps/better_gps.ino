#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

static const uint32_t USB_BAUD = 115200;
static const int GPS_RX_PIN = 4;
static const int GPS_TX_PIN = 2;
static const uint32_t GPS_BAUD = 9600;
static const int GPS_OUTPUT_HZ = 10;

#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F"
#define PMTK_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_FIX_CTL_10HZ "$PMTK300,100,0,0,0,0*2C"

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
Adafruit_GPS GPS(&gpsSerial);

uint32_t lastPrintMs = 0;

static void send_pmtk(const char* cmd) {
  GPS.sendCommand(cmd);
  delay(100);
}

void setup() {
  Serial.begin(USB_BAUD);
  gpsSerial.begin(GPS_BAUD);

  GPS.begin(GPS_BAUD);

  send_pmtk(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  if (GPS_OUTPUT_HZ == 10) {
    send_pmtk(PMTK_SET_NMEA_UPDATE_10HZ);
    send_pmtk(PMTK_SET_FIX_CTL_10HZ);
  } else {
    send_pmtk(PMTK_SET_NMEA_UPDATE_5HZ);
    send_pmtk(PMTK_SET_FIX_CTL_5HZ);
  }

  GPS.sendCommand(PGCMD_NOANTENNA);

  Serial.println("{\"status\":\"gps_bridge_ready\"}");
}

void loop() {
  char c = GPS.read();
  (void)c;

  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }

  const uint32_t now = millis();
  const uint32_t periodMs = (GPS_OUTPUT_HZ == 10) ? 100 : 200;

  if (now - lastPrintMs < periodMs) return;
  lastPrintMs = now;

  int fix = GPS.fix ? 1 : 0;

  if (!fix) {
    Serial.println("{\"fix\":0}");
    return;
  }

  float lat = GPS.latitudeDegrees;
  float lon = GPS.longitudeDegrees;
  float alt = GPS.altitude;   
  float hdop = GPS.HDOP;      

  Serial.print("{\"fix\":1,\"lat\":");
  Serial.print(lat, 7);
  Serial.print(",\"lon\":");
  Serial.print(lon, 7);
  Serial.print(",\"alt\":");
  Serial.print(alt, 2);
  Serial.print(",\"hdop\":");
  Serial.print(hdop, 2);
  Serial.println("}");
}
