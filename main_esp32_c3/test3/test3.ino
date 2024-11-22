#include "ESP32Servo.h"

#define arm1Pin 4  // left
#define arm2Pin 8  // right
Servo arm1, arm2;

#define DIR_PIN 3
#define PWM_PIN 7

int ringv = 0;
int ringvs = 0;
int factor = 0.1;

void ringvsmoother() {  // exponential moving average, for smooth control
  ringv = ringv * (1.0 - factor) + ringvs * factor;
}

void ringwrite() {  // ringv is an integer; write sign and magnitude
  digitalWrite(DIR_PIN, (ringv < 0) ? LOW : HIGH);
  Serial.println(ringv);
  analogWrite(PWM_PIN, abs(ringv));
}

void setup() {
  Serial.begin(115200);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  arm1.setPeriodHertz(50);
  arm2.setPeriodHertz(50);
  arm1.attach(arm1Pin, 500, 2500);
  arm2.attach(arm2Pin, 500, 2500);
  arm1.write(40);
  arm2.write(180-35);
}

void loop() {
  for (; ringv < 25; ringv += 5) {
    ringwrite();
    delay(200);
  }
  ringwrite();
  delay(5000);
  for (; ringv > 0; ringv -= 5) {
    ringwrite();
    delay(200);
  }
  ringwrite();
  delay(5000);
  for (; ringv > -25; ringv -= 5) {
    ringwrite();
    delay(200);
  }
  ringwrite();
  delay(5000);
  for (; ringv < 0; ringv += 5) {
    ringwrite();
    delay(200);
  }
  ringwrite();
  delay(5000);
}
