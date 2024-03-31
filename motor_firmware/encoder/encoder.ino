#include "MotorController.cpp"
#include "config.h"

int rightEncoderCount, leftEncoderCount;
unsigned long t0;
double odom;
float robotLinearX, robotAngularZ;
MotorController leftController(1, 0.5, 0, &leftEncoderCount, true);
MotorController rightController(1, 0.5, 0, &rightEncoderCount, false);

void doCommand() {
  // reads the first character from serial data, and executes the command associated with that character.

  if (!Serial.available()) return; // do nothing if there is no serial data to read
  char chr = Serial.read();

  switch(chr) {

    // Resets the odometer to 0.
    case RESET_ODOM:
      odom = 0;
      break;

    // Set the motor speeds to 0; don't change them again until re-enabled.
    case DISABLE_MOTORS:
      analogWrite(LSPEED, 0);
      analogWrite(RSPEED, 0);
      leftController.doWriting = false;
      rightController.doWriting = false;
      digitalWrite(13, LOW); // turn off onboard LED
      break;

    // Allow PID controllers to change the motor speeds.
    case ENABLE_MOTORS:
      leftController.doWriting = true;
      rightController.doWriting = true;
      digitalWrite(13, HIGH); // turn on onboard LED
      break;

    // Send current twist message data over serial.
    case GET_DATA:
      Serial.write((byte*)&robotLinearX, sizeof(float));
      Serial.write((byte*)&robotAngularZ, sizeof(float));
      Serial.write((byte*)&odom, sizeof(float));
      break;

    // Update the PID controllers with new counts-per-loop setpoints.
    case TWIST_SETPOINT: {
      byte receivedBytes[2 * sizeof(float)]; // serial byte buffer
      float receivedLinX, receivedAngZ;
      Serial.readBytes(receivedBytes, 2 * sizeof(float)); // fill the buffer
      memcpy(&receivedLinX, &receivedBytes[0], sizeof(float)); // copy the read values
      memcpy(&receivedAngZ, &receivedBytes[sizeof(float)], sizeof(float));
      getNewSetpoints(receivedLinX, receivedAngZ); // set new PID setpoints
    }
  }
  while (Serial.available()) Serial.read(); // clear serial input after command is processed
}

inline void getNewSetpoints(float setLinearX, float setAngularZ) {
  /* uses a twist message to calculate the counts-per-loop values for the PID controllers, and sets them.
  Zeroes PID integrals to prevent error interference. */

  leftController.setpointCPL = ANGVEL_2_CPL * (setLinearX - setAngularZ * HALF_WHEEL_TRACK_LENGTH);
  leftController.integral = 0;
  rightController.setpointCPL = ANGVEL_2_CPL * (setLinearX + setAngularZ * HALF_WHEEL_TRACK_LENGTH);
  rightController.integral = 0;
}

void readRightEncoder() {
  // Interrupt that updates the right encoder count.

  if (digitalRead(RB)) rightEncoderCount--;
  else rightEncoderCount++;
}

void readLeftEncoder() {
  // Interrupt that updates the left encoder count.

  if(digitalRead(LB)) leftEncoderCount--;
  else leftEncoderCount++;
}

inline void updateState() {
  // Updates both PID controllers, current linear velocity (linearX) turning rate (angularZ), and odometer.

  rightController.update();
  leftController.update();
  robotLinearX = (rightController.linearVel + leftController.linearVel) * 0.5;
  robotAngularZ = (leftController.linearVel - robotLinearX) / HALF_WHEEL_TRACK_LENGTH;
  odom += robotLinearX * DT;
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
  odom = 0;
  t0 = millis();
}

void loop() {
  doCommand();

  if (millis() - t0 > DT_MILLIS) {
    noInterrupts();
    updateState();
    interrupts();
    t0 = millis();
  }

}
