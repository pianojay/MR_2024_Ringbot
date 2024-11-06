/*
  #################################################################
          main.ino: setup() and loop() is here.

          Needs RingIMU.ino in a same folder for RingIMU::roll
          No more routines, but halt when fell.

          TODO:
            - MPU6050 roll value problem (Oh no!)
            - refactor/rewrite Motors -> very important
              - Servo: angle problem
              - Motor: speed control ()
            - fix HTML: Weird reactance
            - Erase all Serial and change to server
              - Only use them for debugging!!!
            - Parameter calibrations for actual hardware
  #################################################################
*/

#include <WiFi.h>        // for WiFi connection
#include <WebServer.h>   // for hosting
#include <PID_v1.h>      // for PID control
#include <ESP32Servo.h>  // ESP32Servo: for BLDC and servo
#include <Wire.h>        // for MPU6050
#include <MPU6050.h>     // for MPU6050; https://github.com/ElectronicCats/mpu6050


// namespace forward declaration:
namespace RingWiFi {
void RingWiFisetup();
void RingWiFiloop();
}


// BLDC motor; Create PWM signal via Servo.h for ESC (https://joyonclear.tistory.com/9)
#define ringPin 4
Servo ring;
float ringv = 1500, ringvs = 1500;  // current, desired
float factor = 0.1;                 // Manual Tuning (EMA alpha factor)
#define MinRingv 1000               // based on PWM signal
#define MaxRingv 2000

void ringvsmoother() {  // exponential moving average, for smooth control
  ringv = ringv * (1.0 - factor) + ringvs * factor;
}


// Servo motor: DS3218MG
#define arm1Pin 2  // only one arm
Servo arm1;
const double deviation = 0;            // Determine the actual servo angle from armao
double rolli, armao = 145, rolls = 0;  // current roll (sensor), servo angle out, desired roll
// 145 was the imperical value to go.
double armkp = 100.0, armki = 5.0, armkd = 25.0;  // TODO: Manual Tuning
PID armPid(&rolli, &armao, &rolls, armkp / 100, armki / 100, armkd / 100, DIRECT);
#define MinArma 50  // based on imperical servo range
#define MaxArma 240
#define Minrolls -45  // TODO: manual tuning (but probably suitable)
#define Maxrolls 45


// MPU6050; Check RingIMU.ino, which should be in a same file.
// We use RingIMU::roll for roll data.
// namespace forward declaration:
namespace RingIMU {
void RingIMUsetup();
void RingIMUloop();
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);
extern float roll;
}


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Connected!");

  // WiFi-server setup
  RingWiFi::RingWiFisetup();

  ring.writeMicroseconds(ringv);
  ring.attach(ringPin);

  arm1.write(armao);
  armPid.SetMode(AUTOMATIC);
  armPid.SetOutputLimits(MinArma + deviation, MaxArma - deviation);
  arm1.attach(arm1Pin);

  // And then MPU6050 setup
  RingIMU::RingIMUsetup();
}


void loop() {
  // MPU6050 stuff; -> get roll data
  RingIMU::RingIMUloop();
  rolli = RingIMU::roll;
  Serial.println(rolli);



  if (abs(rolli) > 45) {  // Detect fall -> stop motor
    Serial.println("I fell: Stopping");
    ringvs = 1500;
    rolls = 0;
  } else {
    RingWiFi::RingWiFiloop();  // Run server; handle commands and responses
  }

  // Update BLDC
  ringvs = constrain(ringvs, MinRingv, MaxRingv);
  ringvsmoother();
  ring.writeMicroseconds(ringv);

  // Update servo
  rolls = constrain(rolls, Minrolls, Maxrolls);

  // for interactive tuning, if applicable
  armkp = constrain(armkp, 0, 10000);
  armki = constrain(armki, 0, 10000);
  armkd = constrain(armkd, 0, 10000);
  armPid.SetTunings(armkp, armki, armkd);

  armPid.Compute();
  arm1.write(armao - deviation);
}
