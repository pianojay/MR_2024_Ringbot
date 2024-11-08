/*
  #################################################################
          main.ino: setup() and loop() is here.

          Needs these in a same folder:
            RingIMU.ino for RingIMU::roll
            RingWiFi.ino for RingWiFi

          TODO:
            - MPU6050 roll value problem (Oh no!)
            - 알고리즘 다시 구상하기: 넘어지려고 하면 균형 잡기...
              - 근데 정확한 내용이 무엇인지?
            - Parameter calibrations for actual hardware
  #################################################################
*/

#include <WiFi.h>        // for WiFi connection
#include <WebServer.h>   // for hosting
#include <PID_v1.h>      // for PID control
#include <ESP32Servo.h>  // ESP32Servo: for servo
#include <Wire.h>        // for MPU6050
#include <MPU6050.h>     // for MPU6050; https://github.com/ElectronicCats/mpu6050


// WiFi and HTTP server; Check RingWiFi.ino, which should be in a same file.
// namespace forward declaration:
namespace RingWiFi {
void RingWiFisetup();
void RingWiFiloop();
extern String response;
}


// BLDC motor
#define ring_DIR_Pin 4
#define ring_PWM_Pin 5
float ringv = 0, ringvs = 0;  // current, desired
float factor = 0.1;           // Manual Tuning (EMA alpha factor)
#define MinRingv -70          // based on educated guess
#define MaxRingv 70

void ringvsmoother() {  // exponential moving average, for smooth control
  ringv = ringv * (1.0 - factor) + ringvs * factor;
}

void ringwrite() {  // ringv is an integer; write sign and magnitude
  digitalWrite(ring_DIR_Pin, (ringv < 0) ? LOW : HIGH);
  analogWrite(ring_PWM_Pin, abs(ringv));
}


// Servo motor: DS3218MG; Based on PWM specifications!
#define arm1Pin 2
#define arm2Pin 3
Servo arm1, arm2;
const double deviation = 300;           // Determine the actual servo PWM from armao
double rolli, armao = 1500, rolls = 0;  // current roll (sensor), servo PWM, desired roll
// 145 was the imperical value to go.
double armkp = 100.0, armki = 5.0, armkd = 25.0;  // TODO: Manual Tuning
PID armPid(&rolli, &armao, &rolls, armkp / 100, armki / 100, armkd / 100, DIRECT);
#define MinArma 500  // based on PWM spec
#define MaxArma 2500
#define Minrolls -45  // TODO: manual tuning (but probably suitable)
#define Maxrolls 45


// MPU6050; Check RingIMU.ino, which should be in a same file.
// namespace forward declaration:
namespace RingIMU {
void RingIMUsetup();
void RingIMUloop();
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);
extern float roll;
}


void setup() {
  // Serial is only used for debugging while connected to Arduino IDE terminal.
  Serial.begin(115200);
  unsigned long waiting = millis();
  while (!Serial && millis() - waiting < 1000)
    ;
  if (Serial) Serial.println("Serial Connected!");

  // WiFi server setup
  RingWiFi::RingWiFisetup();

  // BLDC setup
  pinMode(ring_DIR_Pin, OUTPUT);
  pinMode(ring_PWM_Pin, OUTPUT);
  ringwrite();

  // Servo setup
  arm1.setPeriodHertz(50);
  arm2.setPeriodHertz(50);
  arm1.writeMicroseconds(armao - deviation);
  arm2.writeMicroseconds(armao + deviation);
  arm1.attach(arm1Pin, 500, 2500);
  arm2.attach(arm2Pin, 500, 2500);

  // PID setup
  armPid.SetMode(AUTOMATIC);
  armPid.SetOutputLimits(MinArma + deviation, MaxArma - deviation);

  // MPU6050 setup
  RingIMU::RingIMUsetup();
}


void loop() {
  RingIMU::RingIMUloop();  // get roll data
  rolli = RingIMU::roll;

  RingWiFi::RingWiFiloop();  // Run server; handle commands and responses
  if (Serial) Serial.println(RingWiFi::response);

  if (abs(rolli) > 45) {  // Detect fall -> set speed to zero to stop motor
    if (Serial) Serial.println("I fell: Stopping");
    ringvs = 0;
    rolls = 0;
    // TODO
    // arm1.writeMicroseconds(500);
    // arm2.writeMicroseconds(2500);
  }

  // Update BLDC
  ringvs = constrain(ringvs, MinRingv, MaxRingv);
  ringvsmoother();
  ringwrite();

  // Update servo
  rolls = constrain(rolls, Minrolls, Maxrolls);

  // for interactive tuning, if applicable
  armkp = constrain(armkp, 0, 10000);
  armki = constrain(armki, 0, 10000);
  armkd = constrain(armkd, 0, 10000);
  armPid.SetTunings(armkp, armki, armkd);

  // PID control update
  armPid.Compute();
  arm1.writeMicroseconds(armao - deviation);
  arm2.writeMicroseconds(armao + deviation);
}
