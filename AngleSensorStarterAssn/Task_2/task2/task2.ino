#include "Arduino_BMI270_BMM150.h"

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  float x, y, z, angle;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    angle = RAD_TO_DEG*atan(y/z);z
    Serial.println(angle);
  }
}




