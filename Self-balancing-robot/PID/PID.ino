#include "Arduino_BMI270_BMM150.h"
#include <PID_v1.h>
#include "mbed.h"

#define M1F D10 //Green: motor 1
#define M1B D9  //Blue:  motor 1
#define M2B D8  //Green: motor 2
#define M2F D7  //Blue:  motor 2
#define PWM_FREQ 10000.0

mbed::PwmOut M2BPin(digitalPinToPinName(M1B));
mbed::PwmOut M1FPin(digitalPinToPinName(M1F));
mbed::PwmOut M1BPin(digitalPinToPinName(M2B));
mbed::PwmOut M2FPin(digitalPinToPinName(M2F));

// float Kp = 10.0, Ki = 0.0, Kd = 0.0;
// double currentAngle = 0.0, targetAngle = 0.0, PWM;
// float kAcc = 0.15, kGyro = 0.85;
// float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

// //Specify the links and initial tuning parameters
// PID myPID(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd, DIRECT);

void setup() {
  // Serial.begin(9600);
  // //while (!Serial);
  // if (!IMU.begin()) {
  //   Serial.println("Failed to initialize IMU!");
  //   while (1);
  // }
  // SampleRate = IMU.gyroscopeSampleRate();
  // myPID.SetOutputLimits(-255, 255);
  // myPID.SetSampleTime(1);
  // myPID.SetMode(AUTOMATIC);

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
  // //----------------complementary filter------------------------
  // if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
  //   IMU.readAcceleration(accX, accY, accZ);
  //   accAngle = RAD_TO_DEG*atan(accY/accZ);

  //   IMU.readGyroscope(gyroX, gyroY, gyroZ);
  //   static double lastTime = millis();
  //   double dt = (millis() - lastTime) / 1000.0;
  //   lastTime = millis();
  //   gyroAngle = gyroZ * dt + currentAngle;
  //   Serial.print(dt);

  //   currentAngle = kGyro*(gyroAngle) + kAcc*(accAngle);
  //   Serial.print("Current Angle: ");
  //   Serial.print(currentAngle);
  //   Serial.print("\tgyroAngle: ");
  //   Serial.print(gyroAngle);
  //   Serial.print("\taccAngle: ");
  //   Serial.print(accAngle);
  //   }
  // //-----------------------------------------------------------

  // //----------------------PID---------------------------------
  // myPID.Compute();
  // float speed = abs(PWM)/255.0;

  // if (currentAngle > (targetAngle)) {
  //   M1FPin.write(1.0);
  //   M1BPin.write(1.0 - speed);
  //   M2FPin.write(1.0);
  //   M2BPin.write(1.0 - speed);
  // } else if (currentAngle < (targetAngle))  {
  //   M1FPin.write(1.0 - speed);
  //   M1BPin.write(1.0);
  //   M2FPin.write(1.0 - speed);
  //   M2BPin.write(1.0);
  // } else {
    M1FPin.write(1.0 - 0.1);
    M1BPin.write(1.0);
    M2FPin.write(1.0 - 0.1);
    M2BPin.write(1.0);
    // M1FPin.write(1.0);
    // M1BPin.write(1.0 - 0.1);
    // M2FPin.write(1.0);
    // M2BPin.write(1.0 - 0.1);
  //}
  //----------------------------------------------------------
  // Serial.print("\tSpeed: ");
  // Serial.println(speed);
}