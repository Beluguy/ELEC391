#include "Arduino_BMI270_BMM150.h"

#define PWM_1 D10
#define PWM_2 D9
#define PWM_3 D8
#define PWM_4 D7

float SampleRate;
float currentAngle = 0;

float accX, accY, accZ, gyroX, gyroY, gyroZ, kAcc, kGyro, accAngle, gyroAngle, errorAngle, targetAngle;

void setup() {

  Serial.begin(9600);
  while (!Serial);
 if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();

  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);
}

void loop() {

  kAcc = 0.05;
  kGyro = 1.0 - kAcc;

  targetAngle = 0;

  if (IMU.gyroscopeAvailable()&&IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    accAngle = RAD_TO_DEG*(atan(accY/accZ));

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    gyroAngle = (1.00/SampleRate)*gyroX;

    currentAngle = kGyro*(gyroAngle + currentAngle) + kAcc*(accAngle);
    Serial.println(currentAngle);
  }

  errorAngle = targetAngle - currentAngle;

  if(errorAngle=0){
    //don't turn
  } elseif(errorAngle < 0) {
    // turn in one dir
    //(255*abs(errorAngle)/90)
  } else {
    //go other dir
  }

}