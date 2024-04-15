/* Reads the RC mode select channel and outputs it as a digital 1 or 0 to the main board.
Also deals with controlling the yellow and red lights.
If in autonomous mode, flash the yellow LED repeatedly. Otherwise, keep it solid.
Red light is controlled by a receiver switch.
This is done on a separate board to prevent delays from pulseIn messing with encoder counts. */

// pins
#define MODE_OUTPUT 13
#define MODE_INPUT A0
#define RED_LIGHT_INPUT A1
#define RED_LIGHT 3
#define YELLOW_LIGHT 1

unsigned long t0;
int pulse;

void setup() {
  pinMode(MODE_OUTPUT, OUTPUT);
  pinMode(MODE_INPUT, INPUT);
  pinMode(YELLOW_LIGHT, OUTPUT);
  pinMode(RED_LIGHT, OUTPUT);
  pinMode(RED_LIGHT_INPUT, INPUT);
  t0 = millis();
}

void flashYellow(int flashPeriod, int pulseLength) {
  // flash the yellow light.
  digitalWrite(YELLOW_LIGHT, LOW);
  if (millis() - t0 > flashPeriod) {
    digitalWrite(YELLOW_LIGHT, HIGH);
    delay(pulseLength);
    t0 = millis();
  }
  digitalWrite(YELLOW_LIGHT, LOW);
}

bool readSwitch(int pin) { 
  // read a digital switch channel from the receiver.
  int pulse = pulseIn(pin, HIGH, 30000);
  if (pulse > 1500) {
    return true;
  }
  return false;
}

void loop() {

  // control mode and yellow light
  if (readSwitch(MODE_INPUT)) {
    digitalWrite(MODE_OUTPUT, HIGH);
    digitalWrite(YELLOW_LIGHT, HIGH);
  }
  else {
    digitalWrite(MODE_OUTPUT, LOW);
    flashYellow(1000, 50);
  }
  
  // control red light
  if (readSwitch(RED_LIGHT_INPUT)) {
    digitalWrite(RED_LIGHT, HIGH);
  }
  else {
    digitalWrite(RED_LIGHT, LOW);
  }
}