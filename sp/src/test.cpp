#include <Arduino.h>


/*
Example: Value in Decimal
0 or 64 - motor LEFT stop.
63 - motor LEFT full forward.
127 - motor LEFT full reverse.
128 or 192 - motor RIGHT stop.
191 - motor RIGHT full forward.
255 - motor RIGHT full reverse.

*/


void ramp(byte a, byte b, int delayMs) {

  for (byte i = a; i <= b; i++) {
    Serial2.write(i);
    delay(delayMs);
  }
  Serial2.write((byte)0);
  Serial2.write((byte)192);
}

void setup() {
   Serial.begin(115200);
   Serial2.begin(9600, SERIAL_8N1, 16, 17);  // UART2 on GPIO 16 (RX) and 17 (TX)
   delay(3000);
}

void loop() {
  
  ramp(128, 191, 50);
  delay(10000);
}

