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
  float x, y, z, anglex, angley;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    angley = RAD_TO_DEG*atan(y/z);
    anglex = RAD_TO_DEG*atan(x/z);
    Serial.print(angley);
    Serial.print("\t");
    Serial.println(anglex);

  }
}




