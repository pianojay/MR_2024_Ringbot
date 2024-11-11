/*
  #################################################################
          main.ino: setup() and loop() is here.

          Needs these in a same folder:
            RingIMU.ino for RingIMU::roll
            RingWiFi.ino for RingWiFi

          TODO:
            - Servo angle problem:
              Each arm has a range of 0~180.
              However, they should cover each semicircle. (left and right)
              Need to convert the angles into their respective angle.
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
#define ring_DIR_Pin 3
#define ring_PWM_Pin 7
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


// Servo motor: DS3120MG; Based on PWM specifications!
// which is which arm ?? considering arm1 as left, arm2 as right, for now.
// if opposite, just change pin number.
#define arm1Pin 4
#define arm2Pin 8
Servo arm1, arm2;
double rolli, armao = 0, rolls = 0;            // current roll (sensor), servo PWM, desired roll
double armkp = 0.0, armki = 0.0, armkd = 0.0;  // unit: Per mile. TODO: Manual Tuning
PID armPid(&rolli, &armao, &rolls, armkp / 1000, armki / 1000, armkd / 1000, DIRECT);
#define ArmaDev 60    // Deviation; Determine the actual servo PWM from armao
#define ArmWidth 10   // To compensate each arm's width
#define MinRolls -45  // To prevent excessive tilting
#define MaxRolls 45


// MPU6050; Check RingIMU.ino, which should be in a same file.
// namespace forward declaration:
namespace RingIMU {
void RingIMUsetup();
void RingIMUloop();
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);
extern float roll;
}


// Fall control
#define FallRoll 45
#define ResetRoll 2.5
#define ResetV 2.5
#define FreeV 5
bool isFall = false;
bool isReset = true;  // initial state is considred as a reset.

#define FallArma 30  // TODO: speculation... we need to measure correct angle.
void catchfall() {
  // The two arm touches the ground symmetrically;
  // the ring should become perpendicular after this, assuming the ground is flat.
  // For this to work on any uneven plane, we need more modelling and calculations...
  arm1.write(FallArma);
  arm2.write(180 - FallArma);
}


void setup() {
  // Serial is only used for debugging while connected to Arduino IDE terminal.
  Serial.begin(115200);
  unsigned long waiting = millis();
  while (!Serial && millis() - waiting < 1000)  // waiting connection for a second
    ;
  if (Serial) Serial.println("Serial Connected!");

  // WiFi server setup
  RingWiFi::RingWiFisetup();

  // BLDC setup
  pinMode(ring_DIR_Pin, OUTPUT);
  pinMode(ring_PWM_Pin, OUTPUT);
  ringwrite();

  // Servo setup
  arm1.setPeriodHertz(50);  // based on PWM spec
  arm2.setPeriodHertz(50);
  catchfall();
  arm1.attach(arm1Pin, 500, 2500);
  arm2.attach(arm2Pin, 500, 2500);

  // PID setup
  armPid.SetMode(AUTOMATIC);
  armPid.SetOutputLimits(-ArmaDev + ArmWidth, ArmaDev - ArmWidth);

  // MPU6050 setup
  RingIMU::RingIMUsetup();

  // IMU stabilizing loop (5 seconds)
  waiting = millis();
  while (millis() - waiting < 5000) RingIMU::RingIMUloop();
}


void loop() {
  // get roll data
  RingIMU::RingIMUloop();
  rolli = RingIMU::roll;
  // We need to rotate 180 degrees by x-axis
  // because "SOMEBODY" assembled MPU6050 in wrong orientation.
  rolli += (rolli < 0) ? 180.0 : -180.0;

  // Run server; handle commands and responses
  RingWiFi::RingWiFiloop();
  if (Serial) Serial.println(RingWiFi::response);
  if (Serial) Serial.println(rolli);

  // (isFall, isReset)
  // (false, true): catchfall() -> (false, false): PID
  // (false, false) -> (true, false) -> (false, true) -> (false, false)

  // Detect fall; if tilted more than 45 deg
  if (abs(rolli) > FallRoll) {
    isFall = true;
  }

  // If falling: set target velocity to 0, catch falling with arms.
  if (isFall) {
    if (Serial) Serial.println("I am Fall");
    ringvs = 0;
    catchfall();
    isReset = false;

    // Detect reset; break from falling
    if ((abs(rolli) < ResetRoll) && abs(ringv) < ResetV) {
      // if tilted less than 2.5 deg and speed less than 2.5
      isReset = true;
      isFall = false;
    }
  }

  // If reset:
  if (isReset) {
    if (Serial) Serial.println("I am Reset");
    catchfall();
    // Detect enough velocity; break from reset, continue with PID control.
    if (abs(ringv) >= FreeV) {  // if speed more or equal to 5
      isReset = false;
    }
  }

  // Update BLDC
  ringvs = constrain(ringvs, MinRingv, MaxRingv);
  ringvsmoother();
  ringwrite();

  // Update servo
  rolls = constrain(rolls, MinRolls, MaxRolls);

  // for interactive tuning, if applicable
  armkp = constrain(armkp, 0, 10000);
  armki = constrain(armki, 0, 10000);
  armkd = constrain(armkd, 0, 10000);
  armPid.SetTunings(armkp, armki, armkd);

  // PID control update
  if (!isFall && !isReset) {
    armPid.Compute();
    arm1.write(armao - ArmaDev + 180);
    arm2.write(armao + ArmaDev);
  }
}
