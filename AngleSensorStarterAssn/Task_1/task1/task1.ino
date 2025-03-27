#include "Arduino_BMI270_BMM150.h"
double gyrox = 0.0, gyroy = 0.0, gyroz = 0.0;


void setup() {
  Serial.begin(9600);
  while (!Serial);
 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    static double lastTime = millis();
    double dt = (millis() - lastTime) / 1000.0;
    lastTime = millis();
    gyrox = x * dt + gyrox;
    gyroy = y * dt + gyroy;
    gyroz = z * dt + gyroz;
    Serial.print(gyrox);
    Serial.print("\t");
    Serial.print(gyroy);
    Serial.print("\t");
    Serial.println(gyroz);
  }
}
