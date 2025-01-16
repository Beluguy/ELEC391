#include "Arduino_BMI270_BMM150.h"

float SampleRate;
float currentAngle = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();
}

void loop() {
  float accX, accY, accZ, gyroX, gyroY, gyroZ, kAcc, kGyro, accAngle, gyroAngle;

  kAcc = 0.05;
  kGyro = 1.0 - kAcc;

  if (IMU.gyroscopeAvailable()&&IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    accAngle = RAD_TO_DEGatan(accY/accZ);

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    gyroAngle = (1.00/SampleRate)*gyroX;

    currentAngle = kGyro(gyroAngle + currentAngle) + kAcc(accAngle);
    Serial.println(currentAngle);
  }
}