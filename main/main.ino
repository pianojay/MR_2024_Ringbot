/*
  #################################################################
          main.ino: setup() and loop() is here.

          Needs RingIMU.ino in a same folder for RingIMU::roll

          TODO:
            - standup(), turnleft(), turnright() routine
            - Parameter calibrations for actual hardware
  #################################################################
*/

#include <SoftwareSerial.h>  // for Bluetooth
#include <PID_v1.h>          // for PID control
#include <Servo.h>           // for BLDC and servo
#include <Wire.h>            // for MPU6050
#include <MPU6050.h>         // for MPU6050; https://github.com/ElectronicCats/mpu6050


// HC-O6 BlueTooth Serial connections; Using PuTTy
#define rxPin 2
#define txPin 3
SoftwareSerial BTS(rxPin, txPin);
int IByte;


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
#define arm1Pin 5
#define arm2Pin 6
Servo arm1, arm2;                              // left and right
const double deviation = 45;                   // Determine the actual servo angle from armao
double rolli, armao = 90, rolls = 0;           // current roll (sensor), servo angle out, desired roll
double armkp = 1, armki = 0.05, armkd = 0.25;  // Manual Tuning
PID armPid(&rolli, &armao, &rolls, armkp, armki, armkd, DIRECT);
#define MinArma 0  // based on servo
#define MaxArma 180
#define Minrolls -45  // manual tuning
#define Maxrolls 45


// MPU6050; Check RingIMU.ino, which should be in a same file.
// We use RingIMU::roll for roll data.
// namespace forward declaration:
namespace RingIMU {
void RingIMUsetup();
void RingIMUloop();
void ComplementaryUpdate(float ay, float az, float gyroy);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);
extern float roll, roll_C;
extern bool verbose;
}


// Detecting falling and standing up
bool isfallen = false;

void standup() {
  // Stand up from fallen state
}


// Holonomic Turning; When fully stopped
int turn = 0;  // 1 for turnleft(), 2 for turnright()

void turnleft() {
  // Leg motion for turning left from standing state
}

void turnright() {
  // Leg motion for turning right from standing state
}


// This is for data reporting (temporary)
char buffer[128];
void format2f(float x, float y, int width = 8, int precision = 2) {
  char x_str[10], y_str[10];
  dtostrf(x, width, precision, x_str);
  dtostrf(y, width, precision, y_str);
  sprintf(buffer, "%s, %s", x_str, y_str);
  return;
}


void setup() {
  BTS.begin(9600);
  while (!BTS)
    ;
  BTS.println("Connected!");

  ring.writeMicroseconds(ringv);
  ring.attach(ringPin);

  arm1.write(armao - deviation);
  arm2.write(armao + deviation);
  armPid.SetMode(AUTOMATIC);
  armPid.SetOutputLimits(MinArma + deviation, MaxArma - deviation);
  arm1.attach(arm1Pin);
  arm2.attach(arm2Pin);

  // And then MPU6050 setup
  RingIMU::RingIMUsetup();
}


void loop() {
  // MPU6050 stuff; -> get roll data
  RingIMU::RingIMUloop();
  rolli = RingIMU::roll;

  format2f(RingIMU::roll, RingIMU::roll_C);
  BTS.println(buffer);  // Compare roll value between Madgwick and Complementary

  if (isfallen) {  // Detect fall, stop, and then stand up
    if (abs(ringv - 1500) < 5) {
      ringv = 1500;
      BTS.println("Stopped for standing up");
      standup();
      isfallen = !isfallen;
    }
  } else if (abs(rolli) > 45) {
    BTS.println("I fell: Stopping");
    ringvs = 1500;
    rolls = 0;
    isfallen = !isfallen;
  } else if (turn) {                                  // holonomic turning; stop the robot first and then turn
    if (abs(ringv - 1500) < 5 && abs(rolli) < 0.5) {  // check if stationary while standing
      ringv = 1500;
      BTS.println("Stopped for turning");
      if (turn == 1) turnleft();
      if (turn == 2) turnright();
      turn = 0;  // turning done; reset turning state
    }
  } else if (BTS.available()) {  // Read Bluetooth input for control
    IByte = BTS.read();
    switch (IByte) {
      case ' ':
        BTS.println("Reset: Stopping");
        ringvs = 1500;
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
      case 'z':
        BTS.println("Turn Left: Stopping");
        ringvs = 1500;
        rolls = 0;
        turn = 1;
        break;
      case 'x':
        BTS.println("Turn Right: Stopping");
        ringvs = 1500;
        rolls = 0;
        turn = 2;
        break;
      case 'v':
        if (IByte == 'v') {
          if (!RingIMU::verbose) {
            BTS.println("");
            BTS.println("verbose");
          } else {
            BTS.println("");
            BTS.println("quiet");
          }
          IByte = 0;
          RingIMU::verbose = !RingIMU::verbose;
        }
        break;
      default:
        break;
    }

    // Update BLDC
    ringvs = constrain(ringvs, MinRingv, MaxRingv);
    ringvsmoother();
    ring.writeMicroseconds(ringv);

    // Update servo
    rolls = constrain(rolls, Minrolls, Maxrolls);
    // armPid.SetTunings(,,);
    // ^ for interactive tuning, if applicable
    armPid.Compute();
    arm1.write(armao - deviation);
    arm2.write(armao + deviation);
  }
}
