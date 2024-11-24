#include "ESP32Servo.h"

#define arm1Pin 4  // 4 = left
#define arm2Pin 8  // 8 = right
Servo arm1, arm2;
double rolli, armao = 0, rolls = 0;             // current roll (sensor), servo PWM, desired roll
double armkp = 30.0, armki = 0.0, armkd = 0.0;  // unit: percent. TODO: Manual Tuning
#define Res 100
#define ArmaDev 90    // Deviation; Determine the actual servo PWM from armao
#define ArmWidth 10   // To compensate each arm's thickness
#define MinRolls -45  // To prevent excessive tilting
#define MaxRolls 45
#define Compensate 0   // angle exceeding of arm1 from arm2
#define Compensate2 4  // angle to stand upright when symmetric

void setup() {
  Serial.begin(9600);
  arm1.setPeriodHertz(50);
  arm2.setPeriodHertz(50);
  arm1.attach(arm1Pin, 500, 2500);
  arm2.attach(arm2Pin, 500, 2500);
  arm1.write(75);
  arm2.write(180 - 75);
}

int i = 0;

void loop() {
  arm1.write(90 - Compensate - rolls + Compensate2);
  arm2.write(180 - 90 - rolls + Compensate2);
}