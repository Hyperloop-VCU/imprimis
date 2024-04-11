/* Reads the RC mode select channel and outputs it as a digital 1 or 0 to the main board.
This is done on a separate board to prevent delays from pulseIn messing with encoder counts. */

#define MODE_OUTPUT 13
#define RC_INPUT A0

void setup() {
  pinMode(MODE_OUTPUT, OUTPUT);
  pinMode(RC_INPUT, INPUT);
  Serial.begin(9600);
}

void loop() {
  int pulse = pulseIn(RC_INPUT, HIGH, 30000);
  Serial.println(pulse);
  if (pulse > 1500) digitalWrite(MODE_OUTPUT, HIGH);
  else digitalWrite(MODE_OUTPUT, LOW);
}