#include "Arduino_BMI270_BMM150.h"

int SampleRate;

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
  delay (100);
  float x, y, z, angle;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    //Serial.print(x);
    //Serial.print('\t');
    angle = SampleRate*x + 0;
    Serial.println(angle);
  }
}
