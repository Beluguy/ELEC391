#include "Arduino_BMI270_BMM150.h"
#include <PID_v1.h>
#include "mbed.h"

#define M2B D10 //yellow: motor 2
#define M1F D9  //white:  motor 1
#define M1B D8  //green:  motor 1
#define M2F D7  //blue:   motor 2
#define PWM_FREQ 10000.0

mbed::PwmOut M2BPin( digitalPinToPinName( M2B ) );
mbed::PwmOut M1FPin ( digitalPinToPinName( M1F ) );
mbed::PwmOut M1BPin( digitalPinToPinName( M1B ) );
mbed::PwmOut M2FPin( digitalPinToPinName( M2F ) );

float Kp = 26.0, Ki = 0.0, Kd = 0.05;
double currentAngle = 0.0, targetAngle = 0.0, PWM;
float kAcc = 0.1, kGyro = 0.9;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

//Specify the links and initial tuning parameters
PID myPID(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd, DIRECT);

// unsigned long loopStartTime;
// unsigned long loopTime;
// unsigned long maxTime = 0;
// unsigned long minTime = 1000000; // Initialize with a large value

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(1);
  myPID.SetMode(AUTOMATIC);

  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
  M2BPin.period(1.0/PWM_FREQ);
  M1FPin.period(1.0/PWM_FREQ);
  M1BPin.period(1.0/PWM_FREQ);
  M2FPin.period(1.0/PWM_FREQ);
}

void loop() {
  //loopStartTime = micros();

  //----------------complementary filter------------------------
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    accAngle = RAD_TO_DEG*(accY/accZ);

    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    gyroAngle = (1.0/SampleRate)*gyroZ;

    currentAngle = kGyro*(gyroAngle + currentAngle) + kAcc*(accAngle);
    // Serial.print("Current Angle: ");
    // Serial.print(currentAngle);
    // Serial.print("  gyro: ");
    // Serial.print(gyroAngle);
    // Serial.print("  acc angle: ");
    // Serial.println(accAngle);

    //Serial.print("\tSpeed: ");
    }
  //-----------------------------------------------------------

  //----------------------PID---------------------------------
  myPID.Compute();
  float speed = abs(PWM)/255.0;
  
  if (currentAngle > targetAngle) {
    M1FPin.write(speed);
    M1BPin.write(0.0);
    M2FPin.write(speed);
    M2BPin.write(0.0);
  } else if (currentAngle < targetAngle)  {
    M1FPin.write(0.0);
    M1BPin.write(speed);
    M2FPin.write(0.0);
    M2BPin.write(speed);
  } else {
    M1FPin.write(0.0);
    M1BPin.write(0.0);
    M2FPin.write(0.0);
    M2BPin.write(0.0);
  }
  //----------------------------------------------------------
  //Serial.println(speed);
  //loopTime = micros() - loopStartTime;

  //if (loopTime > maxTime) maxTime = loopTime;
  //if (loopTime < minTime) minTime = loopTime;

  //Serial.println(loopTime);
  // Serial.print("\t");
  // Serial.print(minTime);
  // Serial.print("\t");
  // Serial.println(maxTime);
}