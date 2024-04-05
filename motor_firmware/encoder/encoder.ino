#include "MotorController.cpp"
#include "config.h"

int rightEncoderCount, leftEncoderCount;
unsigned long t0;
MotorController leftController(0.02, 0.01, 0, &leftEncoderCount, true);
MotorController rightController(0.02, 0.01, 0, &rightEncoderCount, false);

void doCommand() {
  // reads the first character from serial data, and executes the command associated with that character.

  if (!Serial.available()) return; // do nothing if there is no serial data to read
  char chr = Serial.read();
  switch(chr) {

    case RESET_ANGPOS: // Set the angular position of both wheels to 0.
      leftController.angPos = 0;
      rightController.angPos = 0;
      
    case DISABLE_MOTORS: // Set the motor speeds to 0; don't change them again until re-enabled.
      analogWrite(LSPEED, 0);
      analogWrite(RSPEED, 0);
      leftController.doWriting = false;
      rightController.doWriting = false;
      digitalWrite(13, LOW); // turn off onboard LED
      break;

    case ENABLE_MOTORS: // Allow PID controllers to change the motor speeds.
      leftController.doWriting = true;
      rightController.doWriting = true;
      digitalWrite(13, HIGH); // turn on onboard LED
      break;

    case GET_DATA: // Send current joint-state data over serial.
      Serial.write((byte*)(&leftController.angPos), sizeof(float));
      Serial.write((byte*)(&leftController.angVel), sizeof(float));
      Serial.write((byte*)(&rightController.angPos), sizeof(float));
      Serial.write((byte*)(&rightController.angVel), sizeof(float));
      break;

    case 'p': // debug
      Serial.write((byte*)(&leftEncoderCount), sizeof(int));
      Serial.write((byte*)(&rightEncoderCount), sizeof(int));
      break;

    case TWIST_SETPOINT: { // Update the PID controllers with new velocity setpoints.
      float receivedLinX, receivedAngZ;
      serialReadFloat(receivedLinX);
      serialReadFloat(receivedAngZ);
      getNewSetpoints(receivedLinX, receivedAngZ);
    }
  }
}

inline void getNewSetpoints(float setLinearX, float setAngularZ) {
  /* uses a twist message to calculate the counts-per-loop values for the PID controllers, and sets them.
  Zeroes PID integrals to prevent error interference. */

  leftController.setpointCPL = round(ANGVEL_2_CPL * (setLinearX - setAngularZ * HALF_WHEEL_TRACK_LENGTH));
  leftController.integral = 0;
  rightController.setpointCPL = round(ANGVEL_2_CPL * (setLinearX + setAngularZ * HALF_WHEEL_TRACK_LENGTH));
  rightController.integral = 0;
}

float serialReadFloat(float &f1) {
  // reads a float from the serial input and stores it in f1.

  byte receivedBytes[sizeof(float)];
  Serial.readBytes(receivedBytes, sizeof(float));
  memcpy(&f1, &receivedBytes[0], sizeof(float));
}

void readRightEncoder() { // Interrupt that updates the right encoder count.
  if (digitalRead(RB)) rightEncoderCount--;
  else rightEncoderCount++;
}

void readLeftEncoder() { // Interrupt that updates the left encoder count.
  if(digitalRead(LB)) leftEncoderCount--;
  else leftEncoderCount++;
}

void setup() {  
  pinMode(13, OUTPUT); // Onboard LED pin
  digitalWrite(13, HIGH); // motors are enabled by default, communicate that via LED
  pinMode(RA, INPUT);
  pinMode(RB, INPUT);
  pinMode(RINDEX, INPUT);
  pinMode(LA, INPUT);
  pinMode(LB, INPUT);
  pinMode(LINDEX, INPUT);
  attachInterrupt(digitalPinToInterrupt(RA), readRightEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(LA), readLeftEncoder, RISING);
  Serial.begin(SERIAL_BAUD_RATE);

  t0 = millis();
}

void loop() {
  doCommand();

  if (millis() - t0 > DT_MILLIS) {
    noInterrupts();
    rightController.update();
    leftController.update();
    interrupts();
    t0 = millis();
  }

}
