#include "Arduino_BMI270_BMM150.h"
#include "mbed.h"
#include <Wire.h>
#include <AS5600.h>
#include "ArduPID.h"
ArduPID myController;

#define M2B D10 //yellow: motor 2
#define M1F D9  //white:  motor 1
#define M1B D8  //green:  motor 1
#define M2F D7  //blue:   motor 2
#define PWM_FREQ 10000.0

mbed::PwmOut M2BPin( digitalPinToPinName( M2B ) );
mbed::PwmOut M1FPin ( digitalPinToPinName( M1F ) );
mbed::PwmOut M1BPin( digitalPinToPinName( M1B ) );
mbed::PwmOut M2FPin( digitalPinToPinName( M2F ) );

float Kp = 20.0, Ki = 0.0, Kd = 0.0;
double currentAngle = 0.0, targetAngle = 0.0, PWM, accAngle, gyroAngle;
float kAcc = 0.1, kGyro = 0.9;
float accX, accY, accZ, gyroX, gyroY, gyroZ, SampleRate;

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();
  myController.begin(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd);   //Specify the links and initial tuning parameters
  myController.setOutputLimits(-255.0, 255.0);
  myController.setSampleTime(10);
  myController.start();

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
  //----------------complementary filter------------------------
  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readAcceleration(accX, accY, accZ);
    accAngle = RAD_TO_DEG*(atan(accY/accZ));

    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    static double lastTime = millis();
    double dt = (millis() - lastTime) / 1000.0;
    lastTime = millis();
    gyroAngle = gyroZ * dt + currentAngle;

    currentAngle = kGyro*(gyroAngle) + kAcc*(accAngle);
    // Serial.print("Current Angle: ");
    Serial.print(currentAngle);
    Serial.print("\t");
    Serial.print(gyroAngle);
    Serial.print("\t");
    Serial.print(accAngle);
    }
  //-----------------------------------------------------------

  //----------------------PID---------------------------------
    myController.compute();
    static float speed = abs(PWM)/255.0;
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
  Serial.print("\t");
  Serial.println(speed);
}