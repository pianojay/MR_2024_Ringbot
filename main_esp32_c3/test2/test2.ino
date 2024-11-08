#include "ESP32Servo.h"

#define SERVO_PIN 8

Servo srv;

void setup() {
  Serial.begin(9600);
  srv.setPeriodHertz(50);  // Standard 50hz servo
  srv.attach(SERVO_PIN, 500, 2500); // [500, 2500] -> [0, 270] 
  srv.writeMicroseconds(1500); // Center
  delay(1000);
}

int i = 1500;

void loop() {
  for (; i < 2500; i += 5) {
    srv.writeMicroseconds(i);
    Serial.println(i);
    delay(10);
  }
  for (; i > 500; i -= 5) {
    srv.writeMicroseconds(i);
    Serial.println(i);
    delay(10);
  }
}