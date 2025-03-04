#include "Arduino_BMI270_BMM150.h"
#include "mbed.h"

//MOTOR 1
#define M1F D7 //BIN1 - GREEN
#define M1B D8 //BIN2 - BLUE
//MOTOR 2
#define M2F D10 //AIN1 - BLUE
#define M2B D9 //AIN2 - GREEN

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

  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
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
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
  }

  errorAngle = targetAngle - currentAngle;

  if(errorAngle < -2) {

    analogWrite(M1F, 255);
    analogWrite(M1B, 255-round(255*abs(errorAngle)/60));

    analogWrite(M2F, 255);
    analogWrite(M2B, 255-round(255*abs(errorAngle)/60));
    Serial.println("first case");

  } else if(errorAngle > 2) {

    analogWrite(M1F, 255-round(255*abs(errorAngle)/60));
    analogWrite(M1B, 255);

    analogWrite(M2F, 255-round(255*abs(errorAngle)/60));
    analogWrite(M2B, 255);
<<<<<<< HEAD:MotorControl/PoC_Demo/PoC_Demo/angleChanging/angleChanging.ino
    Serial.println("second case");
=======
>>>>>>> 1326da0d2bba8603f41926bffcc499ccef94c22d:MotorControl/PoC_Demo/angleChanging/angleChanging.ino

  } else {
    analogWrite(M1F, 0);
    analogWrite(M1B, 0);

    analogWrite(M2F, 0);
    analogWrite(M2B, 0);
<<<<<<< HEAD:MotorControl/PoC_Demo/PoC_Demo/angleChanging/angleChanging.ino
    Serial.println("last case");
=======
>>>>>>> 1326da0d2bba8603f41926bffcc499ccef94c22d:MotorControl/PoC_Demo/angleChanging/angleChanging.ino
  }
}