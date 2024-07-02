#include <SoftwareSerial.h>  // for Bluetooth
#include <PID_v1.h>
#include <Servo.h>    // for BLDC and servo
#include <Wire.h>     // for MPU6050
#include <MPU6050.h>  // for MPU6050; https://github.com/ElectronicCats/mpu6050

// Based on Uno but surely compatible with Nano.

// HC-O6 BlueTooth Serial connections; Using PuTTy
const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial BTS(rxPin, txPin);
int IByte;

// BLDC motor; Create PWM signal via Servo.h for ESC (https://joyonclear.tistory.com/9)
const byte ringPin = 4;
Servo ring;
float ringv = 1500, ringvs = 1500;  // current, desired
float factor = 0.1;                 // Manual Tuning
#define MinRingv 1000               // based on PWM signal
#define MaxRingv 2000

void ringvsmoother() {  // exponential moving average
  ringv = ringv * (1.0 - factor) + ringvs * factor;
}

// Servo motor: DS3218MG
const byte armPin = 5;
Servo arm;
double rolli, armao = 90, rolls = 0;           // current roll, servo angle out, desired roll
double armkp = 1, armki = 0.05, armkd = 0.25;  // Manual Tuning
PID armPid(&rolli, &armao, &rolls, armkp, armki, armkd, DIRECT);
#define MinArma 0  // based on servo
#define MaxArma 180
#define Minrolls -45  // manual tuning
#define Maxrolls 45

// MPU6050; Check MPU6050IMU.ino
MPU6050lib mpu;
// Not implemented here

bool isfallen = false;

void checkfall() {
  // if roll from IMU over threshold then isfallen = true
  // -> try to stand up at next cycle
}

void standup() {
  // Do something
}


void setup() {
  BTS.begin(9600);
  while (!BTS)
    ;
  BTS.println("Connected!");

  ring.writeMicroseconds(ringv);
  ring.attach(ringPin);

  arm.write(armao);
  armPid.SetMode(AUTOMATIC);
  armPid.SetOutputLimits(MinArma, MaxArma);
  arm.attach(armPin);

  // And then MPU6050 setup
}

void loop() {
  // MPU6050 stuff; -> get roll
  // rolli = roll;

  checkfall();
  if (isfallen) {
    BTS.println("");
    BTS.println("I have fallen.");
    BTS.println("Initiating standing up protocol");
    standup();
  }

  if (BTS.available()) {
    IByte = BTS.read();
    switch (IByte) {
      case ' ':
        BTS.println("Reset");
        ringvs = 0;
        rolls = 0;
        break;
      case 'w':
        BTS.println("Forward");
        ringvs += 5;
        break;
      case 's':
        BTS.println("Backward");
        ringvs -= 5;
        break;
      case 'a':
        BTS.println("Roll Left");
        rolls -= 5;
        break;
      case 'd':
        BTS.println("Roll Right");
        rolls += 5;
        break;
      default:
        break;
    }
    ringvs = constrain(ringvs, MinRingv, MaxRingv);
    ringvsmoother();
    ring.writeMicroseconds(ringv);
    rolls = constrain(rolls, Minrolls, Maxrolls);
    // armPid.SetTunings(,,);
    // for interactive tuning?
    armPid.Compute();
    arm.write(armao);
  }
}
