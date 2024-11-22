/*
  #################################################################
          main.ino: setup() and loop() is here.

          Needs these in a same folder:
            RingIMU.ino for RingIMU::roll
            RingWiFi.ino for RingWiFi

          TODO:
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
#define MinRingv -100          // based on educated guess
#define MaxRingv 100

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
#define arm1Pin 4  // 4 = left
#define arm2Pin 8  // 8 = right
Servo arm1, arm2;
double rolli, armao = 0, rolls = 0;             // current roll (sensor), servo PWM, desired roll
double armkp = 30.0, armki = 0.0, armkd = 0.0;  // unit: percent. TODO: Manual Tuning
#define Res 100
PID armPid(&rolli, &armao, &rolls, armkp / Res, armki / Res, armkd / Res, REVERSE);
#define ArmaDev 90    // Deviation; Determine the actual servo PWM from armao
#define ArmWidth 10   // To compensate each arm's thickness
#define MinRolls -45  // To prevent excessive tilting
#define MaxRolls 45
#define Compensate 0   // angle exceeding of arm1 from arm2
#define Compensate2 4  // angle to stand upright when symmetric


// MPU6050; Check RingIMU.ino, which should be in a same file.
// namespace forward declaration:
namespace RingIMU {
void RingIMUsetup();
void RingIMUloop();
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);
extern float roll;
}


// Fall control
#define FallRoll 50
#define ResetRoll 5
#define ResetV 15
#define FreeV 65
bool isFall = false;
bool isReset = true;  // initial state is considred as a reset.

#define FallArma 40  // TODO: speculation... we need to measure correct angle.
void catchfall() {
  // The two arm touches the ground symmetrically;
  // the ring should become perpendicular after this, assuming the ground is flat.
  // For this to work on any uneven plane, we need more modelling and calculations...
  arm1.write(FallArma - Compensate - rolls + Compensate2);
  arm2.write(180 - FallArma - rolls + Compensate2);
}


void setup() {
  // Serial is only used for debugging while connected to Arduino IDE terminal.
  Serial.begin(115200);
  unsigned long waiting = millis();
  while (!Serial && millis() - waiting < 100)  // waiting connection for 0.1 sec
    ;
  if (Serial) Serial.println("Serial Connected!");

  // WiFi server setup
  RingWiFi::RingWiFisetup();

  // BLDC setup
  pinMode(ring_DIR_Pin, OUTPUT);
  pinMode(ring_PWM_Pin, OUTPUT);

  // Servo setup
  arm1.setPeriodHertz(50);  // based on PWM spec
  arm2.setPeriodHertz(50);
  catchfall();
  arm1.attach(arm1Pin, 500, 2500);
  arm2.attach(arm2Pin, 500, 2500);

  // PID setup
  armPid.SetMode(AUTOMATIC);
  armPid.SetOutputLimits(max(ArmWidth - ArmaDev, ArmaDev - 180), min(ArmaDev - ArmWidth, 180 - ArmaDev));

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
    rolls = 0;
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
      rolls = 2.5 * (int)(rolli / 2.5); // round toward zero, unit of 2.5
    }
  }

  // Update BLDC
  ringvs = constrain(ringvs, MinRingv, MaxRingv);
  ringvsmoother();
  ringwrite();

  // Update servo
  rolls = constrain(rolls, MinRolls, MaxRolls);

  // for interactive tuning, if applicable
  armkp = constrain(armkp, 0, 100);
  armki = constrain(armki, 0, 100);
  armkd = constrain(armkd, 0, 100);
  armPid.SetTunings(armkp / Res, armki / Res, armkd / Res);

  // PID control update
  if (!isFall && !isReset) {
    if (ringv < FreeV) {
      isReset = true;
    }
    armPid.Compute();
    arm1.write(armao - ArmaDev + 180 + Compensate2);
    arm2.write(armao + ArmaDev + Compensate2);
  }
}
