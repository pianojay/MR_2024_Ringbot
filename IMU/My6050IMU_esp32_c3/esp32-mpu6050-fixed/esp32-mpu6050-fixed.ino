#include <Wire.h>

// LED control
// const int BUILTIN_LED = 12;  // ESP32-C3's built-in LED pin

void setup() {
  Serial.begin(115200);
  Wire.begin(5, 6);  // SDA = 5, SCL = 6
  
  // Turn off the built-in LED
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
  
  // Wait for serial connection
  while(!Serial) delay(100);
  
  Serial.println("\nI2C Scanner and MPU6050 Diagnostic");
  
  // Scan for I2C devices
  byte error, address;
  int deviceCount = 0;
  
  Serial.println("Scanning I2C bus...");
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found\n");
  }
  
  // Try to initialize MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // wake up MPU6050
  error = Wire.endTransmission(true);
  
  if (error == 0) {
    Serial.println("MPU6050 initialized successfully");
  } else {
    Serial.println("Failed to initialize MPU6050");
    Serial.print("Error code: ");
    Serial.println(error);
  }
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);  // request 6 registers
  
  if(Wire.available() == 6) {
    int16_t AcX = Wire.read() << 8 | Wire.read();
    int16_t AcY = Wire.read() << 8 | Wire.read();
    int16_t AcZ = Wire.read() << 8 | Wire.read();
    
    Serial.print("Accelerometer: ");
    Serial.print("X = "); Serial.print(AcX);
    Serial.print(" | Y = "); Serial.print(AcY);
    Serial.print(" | Z = "); Serial.println(AcZ);
  }
  
  delay(1000);
}