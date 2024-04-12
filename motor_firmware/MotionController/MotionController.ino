#include "MotorController.cpp"
#include "config.h"

long leftEncoderCount = 0, rightEncoderCount = 0;
int leftManualInput = 0, rightManualInput = 0;
float receivedLinX = 0, receivedAngZ = 0, total_dt = 0;
bool manual; // true if manual mode, false if autonomous mode
unsigned long t0, t1;
MotorController leftController(LSPEED, LDIR, 0.02, 0.01, 0, &leftEncoderCount);
MotorController rightController(RSPEED, RDIR, 0.02, 0.01, 0, &rightEncoderCount);
// chsnrd
void doCommand() {
  // Read the first character from serial data, and execute the command associated with that character.

  if (!Serial.available()) return; // do nothing if there is no serial data to read

  char chr = Serial.read();
  switch(chr) {

    case RESET_ENCODERS: // Reset encoder counts.
      leftEncoderCount -= leftController.prevCount;
      leftController.prevCount = 0;
      rightEncoderCount -= rightController.prevCount;
      rightController.prevCount = 0;
      break;

    case GET_DATA: // Send current mode and encoder data over serial.
      Serial.write((byte*)(&leftEncoderCount), sizeof(long));
      Serial.write((byte*)(&leftController.currCPL), sizeof(long));
      Serial.write((byte*)(&rightEncoderCount), sizeof(long));
      Serial.write((byte*)(&rightController.currCPL), sizeof(long));
      break;

    case DEBUG: {// debug
      break;
    }

    case TWIST_SETPOINT: // Update the PID controllers with new velocity setpoints.
      serialReadFloat(receivedLinX);
      serialReadFloat(receivedAngZ);
      updateSetpoints(receivedLinX, receivedAngZ);
  }
}

void updateSetpoints(float setLinearX, float setAngularZ) {
  /* Use a twist message to calculate the counts-per-loop values for the PID controllers, and set them.
  Zero PID integrals to prevent error interference. */
  leftController.setpointCPL = round(ANGVEL_2_CPL * (setLinearX - setAngularZ * HALF_WHEEL_TRACK_LENGTH));
  leftController.integral = 0;
  rightController.setpointCPL = round(ANGVEL_2_CPL * (setLinearX + setAngularZ * HALF_WHEEL_TRACK_LENGTH));
  rightController.integral = 0;
}

void updateManualMotorInput() { 
  // Get differential drive input from the RC receiver.
  int linearInput = pulseIn(RC_MANUAL_LIN, HIGH, 30000); // 30 millisecond timeout
  int angularInput = pulseIn(RC_MANUAL_ANG, HIGH, 30000);
  int linNormalized = map(linearInput, 1000, 2000, -255, 255);
  int angNormalized = map(angularInput, 1000, 2000, -255, 255);
  if (abs(linNormalized) > 255 || abs(angNormalized) > 255) {
    leftManualInput = 0;
    rightManualInput = 0;
    return;
  }
  leftManualInput = linNormalized + angNormalized * (1 - MANUAL_TURN_COEFF * linNormalized);
  rightManualInput = linNormalized - angNormalized * (1 - MANUAL_TURN_COEFF * linNormalized);
}

void serialReadFloat(float &f) { 
  // Read a float from the serial input and store it in f.
  byte receivedBytes[sizeof(float)];
  Serial.readBytes(receivedBytes, sizeof(float));
  memcpy(&f, &receivedBytes[0], sizeof(float));
}

void readRightEncoder() { 
  // Interrupt to update the right encoder count.
  if (!digitalRead(RB)) rightEncoderCount--;
  else rightEncoderCount++;
}

void readLeftEncoder() { 
  // Interrupt to update the left encoder count.
  if(!digitalRead(LB)) leftEncoderCount--;
  else leftEncoderCount++;
}

void setup() {  
  pinMode(MANUAL_ENABLE, INPUT_PULLUP);
  pinMode(RA, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);
  pinMode(LA, INPUT_PULLUP);
  pinMode(LB, INPUT_PULLUP);
  pinMode(RC_MANUAL_LIN, INPUT);
  pinMode(RC_MANUAL_ANG, INPUT);
  attachInterrupt(digitalPinToInterrupt(RA), readRightEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(LA), readLeftEncoder, FALLING);
  Serial.begin(SERIAL_BAUD_RATE);
  t0 = millis();
}

void loop() {
  doCommand();

  // update control mode and motors every DT seconds
  if (millis() - t0 >= DT_MILLIS) {
    noInterrupts();

    // set current mode
    if (!digitalRead(MANUAL_ENABLE)) {
      manual = true;
      updateManualMotorInput();
    }
    else {
      manual = false;
    }
    // set motor speeds
    leftController.update(leftManualInput, manual);
    rightController.update(rightManualInput, manual);
    

    interrupts();
    t0 = millis();
  }

}
