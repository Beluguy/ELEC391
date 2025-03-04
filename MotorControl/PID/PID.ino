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

float accX, accY, accZ, gyroX, gyroY, gyroZ, kAcc, kGyro, accAngle, gyroAngle, errorAngle, targetAngle, pError, iError, dError, prevError;

int motorControlVar, motorControlVarAbs;

float Kp = 5.0;
float Ki = 2.0;
float Kd = 0.5;

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

  //Complementary filter constants
  kAcc = 0.05;
  kGyro = 0.95;

  targetAngle = 0;

  if (IMU.gyroscopeAvailable()&&IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    accAngle = RAD_TO_DEG*(atan(accY/accZ));

    IMU.readGyroscope(gyroX, gyroY, gyroZ);

    gyroAngle = (1.00/SampleRate)*gyroX;

    currentAngle = kGyro*(gyroAngle + currentAngle) + kAcc*(accAngle);
    //Serial.print(currentAngle);
    //Serial.print("    ");
  }

  //PID

  pError = targetAngle - currentAngle;
  iError += pError*(1.0/SampleRate); 
  dError = (pError - prevError)/(1.0/SampleRate);

  Serial.print(pError);
  Serial.print("    ");
  Serial.print(iError);
  Serial.print("    ");
  Serial.print(dError);
  Serial.print("    ");
  

  prevError = pError;

  motorControlVar = int(Kp*pError + Ki*iError + Kd*dError);
  motorControlVarAbs = abs(motorControlVar);

  if(motorControlVarAbs < 40){
    motorControlVar = 0;
  } else if(motorControlVarAbs> 255 && motorControlVar > 0){
    motorControlVar = 255; 
  } else if(motorControlVarAbs > 255 && motorControlVar < 0){
    motorControlVar = -255;
  } else if(motorControlVar < 0){
    analogWrite(M1F, 255);
    analogWrite(M1B, 255 - motorControlVarAbs);

    analogWrite(M2F, 255);
    analogWrite(M2B, 255 - motorControlVarAbs);
  } else if(motorControlVar > 0){
    analogWrite(M1F, 255 - motorControlVar);
    analogWrite(M1B, 255);

    analogWrite(M2F, 255 - motorControlVar);
    analogWrite(M2B, 255);    
  } else {
    analogWrite(M1F, 0);
    analogWrite(M1B, 0);

    analogWrite(M2F, 0);
    analogWrite(M2B, 0);
  }

  Serial.print(motorControlVar);
  Serial.print("\n");

  //delay(100);
}