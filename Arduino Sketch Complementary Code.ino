#include <Servo.h>

// Pins
const int MOTION_SENSOR_PIN = 2;
const int SERVO_PIN = 9;
const int LOCK_POSITION = 0;
const int UNLOCK_POSITION = 90;

Servo gateServo;
bool isLocked = false;

void setup() {
  Serial.begin(9600);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  gateServo.attach(SERVO_PIN);
  gateServo.write(UNLOCK_POSITION);
  isLocked = false;
}

void loop() {
  if (Serial.available() >= 4) {
    char command[5] = {0};
    Serial.readBytes(command, 4);
    
    if (strcmp(command, "LOCK") == 0) {
      gateServo.write(LOCK_POSITION);
      isLocked = true;
      Serial.write("ACK", 3);
    } 
    else if (strcmp(command, "UNLK") == 0) {
      gateServo.write(UNLOCK_POSITION);
      isLocked = false;
      Serial.write("ACK", 3);
    }
    else if (strcmp(command, "SENS") == 0) {
      int motion = digitalRead(MOTION_SENSOR_PIN);
      Serial.write(motion ? "MOT1" : "MOT0", 4);
    }
    else if (strcmp(command, "STOP") == 0) {
      // Emergency stop - maintain current position
      Serial.write("ACK", 3);
    }
  }
  
  delay(100);
}
