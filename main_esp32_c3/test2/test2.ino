#include "ESP32Servo.h"

#define SERVO_PIN 8

Servo myservo;

void setup() {
    myservo.setPeriodHertz(50); // Standard 50hz servo
    myservo.attach(SERVO_PIN, 500, 2400);
    myservo.write(10);
    Serial.begin(9600);
}

void loop() {
    for(int i = 10; i < 180; i += 5) {
        myservo.write(i);
        Serial.println(i);
        delay(1000);
    }
    for(int j = 175; j > 5; j -= 5) {
        myservo.write(j);
        Serial.println(j);
        delay(1000);
    }
}
