#include "ESP32Servo.h"

#define arm1Pin 4
#define arm2Pin 8
Servo arm1, arm2;

void setup() {
  Serial.begin(9600);
  arm1.setPeriodHertz(50);
  arm2.setPeriodHertz(50);
  arm1.attach(arm1Pin, 500, 2500);
  arm2.attach(arm2Pin, 500, 2500);
  arm1.write(0);
  arm2.write(0);
}

int i = 0;

void loop() {
  delay(2000);
  for (; i <= 180; i += 1) {
    srv.write(i);
    Serial.println(i);
  }
  delay(1000);
  for (; i >= 0; i -= 1) {
    srv.write(i);
    Serial.println(i);
  }
}