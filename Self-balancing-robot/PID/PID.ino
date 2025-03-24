#include "Arduino_BMI270_BMM150.h"
#include <PID_v1.h>

#define M2B D10 //yellow: motor 2
#define M1F D9  //white:  motor 1
#define M1B D8  //green:  motor 1
#define M2F D7  //blue:   motor 2

float Kp = 7.0, Ki = 0.0, Kd = 0.12;
//float aggKp = 15.0, aggKi = 0.0, aggKd = 0.0;
// float Kp = Ku*0.6, Ki = 1.3*Ku/Tu, Kd = 0.075*Ku*Tu;
double currentAngle = 0.0, targetAngle = 0.0, PWM;
float kAcc = 0.01, kGyro = 0.99;

float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

//Specify the links and initial tuning parameters
PID myPID(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  //while (!Serial);
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
    accAngle = RAD_TO_DEG*(accY/accZ);

    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    gyroAngle = (1.0/SampleRate)*gyroX;

    currentAngle = kGyro*(gyroAngle + currentAngle) + kAcc*(accAngle);
    Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.print("  gyro: ");
    Serial.print(gyroAngle);
    Serial.print("  acc angle: ");
    Serial.println(accAngle);

    //Serial.print("\tSpeed: ");
    }
  //-----------------------------------------------------------

  //----------------------PID---------------------------------
  myPID.Compute();
  int speed = abs(PWM);
  //if (speed > 180) speed = 255;

  if (currentAngle > targetAngle + 0.5) {
    analogWrite(M1F, 255);  
    analogWrite(M1B, 255-speed);   
    analogWrite(M2F, 255);  
    analogWrite(M2B, 255-speed);
  } else if (currentAngle < targetAngle - 0.5)  {
    analogWrite(M1F, 255-speed);    
    analogWrite(M1B, 255);   
    analogWrite(M2F, 255-speed);   
    analogWrite(M2B, 255);
  } else {
    analogWrite(M1F, 255);    
    analogWrite(M1B, 255);   
    analogWrite(M2F, 255);   
    analogWrite(M2B, 255);
  }
  //----------------------------------------------------------
  //Serial.println(speed);
}