#include <SoftwareSerial.h>  // for Bluetooth
#include <Servo.h>           // for BLDC and servo
#include <Wire.h>            // for MPU6050
#include <MPU6050.h>         // for MPU6050; https://github.com/ElectronicCats/mpu6050

// Based on Uno but surely compatible with Nano.

// HC-O6 BlueTooth Serial connections; Using PuTTy
const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial BTS(rxPin, txPin);
int IByte = 0;

// BLDC motor; Create PWM signal via Servo.h for ESC
const byte ringPin = 4;
Servo ring;
int ringinit = 0;
int kv = 1000;  // change this per motor spec

// Servo motor: DS3218MG
const byte armPin = 5;
Servo arm;
int arminit = 0;

// MPU6050; Check MPU6050BasicExample.ino
const byte mpuPin = 6;
MPU6050lib mpu;

void setup() {
  BTS.begin(9600);
  while (!BTS)
    ;
  BTS.println("Connected!");

  ring.attach(ringPin);
  ring.write(ringinit);

  arm.attach(armPin);
  arm.write(arminit);
}

void loop() {
  if (BTS.available()) {
    IByte = BTS.read();
    if (IByte == 'w') {
      BTS.println("Forward");
    } else if (IByte == 's') {
      BTS.println("Backward");
    } else if (IByte == 'a') {
      BTS.println("Turn Left");
    } else if (IByte == 'd') {
      BTS.println("Turn Right");
    } else if (IByte == ' ') {
      BTS.println("Reset");
    }
  }
}
