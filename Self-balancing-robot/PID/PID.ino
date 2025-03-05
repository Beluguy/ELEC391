#include "Arduino_BMI270_BMM150.h"
#include <PID_v1.h>

//MOTOR 1
#define M1F D7 //BIN1 - GREEN
#define M1B D8 //BIN2 - BLUE

//MOTOR 2
#define M2F D10 //AIN1 - BLUE
#define M2B D9 //AIN2 - GREEN

float Kp = 10.0, Ki = 0.0, Kd = 0.0;
double currentAngle = 0, targetAngle = 0, PWM;
float kAcc = 0.05, kGyro = 1 - kAcc;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

//Specify the links and initial tuning parameters
PID myPID(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);

  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
}

void loop() {
  //----------------complementary filter------------------------
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    accAngle = RAD_TO_DEG*(atan(accY/accZ));

    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    gyroAngle = (1.0/SampleRate)*gyroX;

    currentAngle = kGyro*(gyroAngle + currentAngle) + kAcc*(accAngle);
    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.print("\tPWM: ");
    }
  //-----------------------------------------------------------

  //----------------------PID---------------------------------
  myPID.Compute();

  if (currentAngle > 2.0) {
    int speed = abs(PWM);
    if (speed < 80) speed = 80;
    analogWrite(M1F, 255);  
    analogWrite(M1B, 255-speed);   
    analogWrite(M2F, 255);  
    analogWrite(M2B, 255-speed);
  } else if (currentAngle < -2.0)  {
    if (PWM < 80) PWM = 80;
    analogWrite(M1F, int(255-PWM));    
    analogWrite(M1B, 255);   
    analogWrite(M2F, int(255-PWM));   
    analogWrite(M2B, 255);
  } else {
    analogWrite(M1F, 0);    
    analogWrite(M1B, 0);   
    analogWrite(M2F, 0);   
    analogWrite(M2B, 0);
  }
  //----------------------------------------------------------
  Serial.println(PWM);
}