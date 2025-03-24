#include "Arduino_BMI270_BMM150.h"
#include "ArduPID.h"
ArduPID myController;

#define M2B D10 //yellow: motor 2
#define M1F D9  //white:  motor 1
#define M1B D8  //green:  motor 1
#define M2F D7  //blue:   motor 2

float Kp = 22.67, Ki = 0.0, Kd = 1.43;
double currentAngle = 0.0, targetAngle = 0.0, PWM = 0.0;
float kAcc = 0.1, kGyro = 0.9;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

void setup() {
  //Serial.begin(9600);
  //while (!Serial);
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();
  myController.begin(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd);   //Specify the links and initial tuning parameters
  myController.setOutputLimits(-255, 255);
  myController.setSampleTime(20);
  myController.start();

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
    // Serial.print("Current Angle: ");
    // Serial.print(currentAngle);
    // Serial.print("\tSpeed: ");
    }
  //-----------------------------------------------------------

  //----------------------PID---------------------------------
    myController.compute();
    int speed = abs(PWM);
    //if (speed < 25) speed = 25;

    if (currentAngle > (targetAngle)) {
      analogWrite(M1F, 255);  
      analogWrite(M1B, 255-speed);   
      analogWrite(M2F, 255);  
      analogWrite(M2B, 255-speed);
    } else if (currentAngle < (targetAngle)) {
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