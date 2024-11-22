#include "ESP32Servo.h"

#define arm1Pin 4  // left
#define arm2Pin 8  // right
Servo arm1, arm2;

void setup() {
  Serial.begin(9600);
  arm1.setPeriodHertz(50);
  arm2.setPeriodHertz(50);
  arm1.attach(arm1Pin, 500, 2500);
  arm2.attach(arm2Pin, 500, 2500);
  arm1.write(75);
  arm2.write(180-75);
}

int i = 0;

void loop() {
  delay(2000);
  for (; i <= 45; i += 1) {
    arm1.write(i);
    arm2.write(180 - i);
    Serial.println(i);
    delay(100);
  }
  delay(2000);
  for (; i >= 25; i -= 1) {
    arm1.write(i);
    arm2.write(180 - i);
    Serial.println(i);
    delay(100);
  }
}