#include "Arduino_BMI270_BMM150.h"

float SampleRate;
float angle = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();
  Serial.println(angle);
}

void loop() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    angle += (1.00/SampleRate)*x;
    Serial.println(angle);
  }
}
