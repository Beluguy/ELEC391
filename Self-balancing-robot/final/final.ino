#include "Arduino_BMI270_BMM150.h"
#include <ArduinoBLE.h>
#include <PID_v1.h>
#include "mbed.h"
#include <Wire.h>
#include <AS5600.h>
#include "speed.h"

#define BUFFER_SIZE 1
#define PWM_FREQ 10000.0

#define M1B D10 //Green: motor 1 
#define M1F D9  //Blue:  motor 1
#define M2B D8  //Green: motor 2
#define M2F D7  //Blue:  motor 2
#define EF A0   //From ESP
#define EB A1   //From ESP

mbed::PwmOut M2BPin(digitalPinToPinName(M1B));
mbed::PwmOut M1FPin(digitalPinToPinName(M1F));
mbed::PwmOut M1BPin(digitalPinToPinName(M2B));
mbed::PwmOut M2FPin(digitalPinToPinName(M2F));

float Kp = 0.0, Ki = 0.0, Kd = 0.0;
double currentAngle = 0.0, targetAngle = 0.0, PWM;
float kAcc = 0.1, kGyro = 0.9;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle;
int turn = 0; // 0 = stationary, 1 = forward, -1 = backward, 2 = left, 3 = right

//Specify the links and initial tuning parameters
PID myPID(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd, DIRECT);

// Define a custom BLE service and characteristic
BLEService customService("fc096266-ad93-482d-928c-c2560ea93a4e");
BLECharacteristic customCharacteristic("9ff0183d-6d83-4d05-a10e-55c142bee2d1", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);


void setup() {
  Serial.begin(9600);
  //while (!Serial);
  //---------------------ble-----------------------------------
  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    //Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("BLE-DEVICE-A19");
  BLE.setDeviceName("BLE-DEVICE-A19");

  // Add the characteristic to the service
  customService.addCharacteristic(customCharacteristic);

  // Add the service
  BLE.addService(customService);

  // Set an initial value for the characteristic
  customCharacteristic.writeValue("Waiting for data");

  // Start advertising the service
  BLE.advertise();

  //Serial.println("Bluetooth® device active, waiting for connections...");
  //----------------------------------------------------------------------

  //Set up gyroscope and pid
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(1);
  myPID.SetMode(AUTOMATIC);

  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);

  pinMode(EF, INPUT);
  pinMode(EB, INPUT); 

  M2BPin.period(1.0/PWM_FREQ);
  M1FPin.period(1.0/PWM_FREQ);
  M1BPin.period(1.0/PWM_FREQ);
  M2FPin.period(1.0/PWM_FREQ);
}

void loop() {
  //----------------------------ble----------------------------------------
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    //Serial.print("Connected to central: ");
    //Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      // Check if the characteristic was written
      if (customCharacteristic.written()) {
        int length = customCharacteristic.valueLength();
        
        if (length == 1) { // Expecting 20 bytes (5 floats)
          static uint8_t data[1];
          customCharacteristic.readValue(data, length);
          memcpy(&turn, data, 1);       // Extract first float
          Serial.print(turn);
        }
      }
      //----------------------------------------------------------------------------------
      
      //----------------complementary filter------------------------
      if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
        accAngle = RAD_TO_DEG*atan(accY/accZ);

        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        static double lastTime = millis();
        double dt = (millis() - lastTime) / 1000.0;
        lastTime = millis();
        gyroAngle = gyroZ * dt + currentAngle;
        //Serial.println(dt);

        currentAngle = kGyro*(gyroAngle) + kAcc*(accAngle);
        // Serial.print("Current Angle: ");
        // Serial.print(currentAngle);
        // Serial.print("\tgyroAngle: ");
        // Serial.print(gyroAngle);
        // Serial.print("\taccAngle: ");
        // Serial.print(accAngle);
        // Serial.print("\t");
        }
      //-----------------------------------------------------------

      //----------------------PID---------------------------------
      myPID.Compute();
      static float speed, leftSpeed, rightSpeed;
      speed = abs(PWM)/255.0;

      if (speed <= 0.1){
        speed = 0.1;
      } else if (speed >= 0.9){
        speed = 0.9;
      }

      //----------------------------------------------------------

      //-------------------comm b/w esp & arduino-------------------
      if (digitalRead(EF)) turn = 1;
      if (digitalRead(EB)) turn = 4;
      //-----------------------------------------------------------

      //------------------------directions-------------------------
      if (turn = 2){
        leftSpeed = speed - 0.05;
        rightSpeed = speed + 0.05;
      } else if (turn = 3){
        leftSpeed = speed - 0.05;
        rightSpeed = speed + 0.05;
      } else {
        leftSpeed = speed;
        rightSpeed = speed;
      }

      if(leftSpeed <= 0.0){
        leftSpeed = 0.0;
      } else if (leftSpeed >= 1.0){
        leftSpeed = 1.0;
      }

      if(rightSpeed <= 0.0){
        rightSpeed = 0.0;
      } else if (rightSpeed >= 1.0){
        rightSpeed = 1.0;
      }


      if (turn = 1){ // forward 
        if (currentAngle > (targetAngle + 1.0)) {
          M1FPin.write(1.0);
          M1BPin.write(1.0 - leftSpeed);
          M2FPin.write(1.0);
          M2BPin.write(1.0 - rightSpeed);
        } else if (currentAngle < (targetAngle + 1.0)) {
          M1FPin.write(1.0 - leftSpeed);
          M1BPin.write(1.0);
          M2FPin.write(1.0 - rightSpeed);
          M2BPin.write(1.0);
        } else {
          M1FPin.write(1.0);
          M1BPin.write(1.0);
          M2FPin.write(1.0);
          M2BPin.write(1.0);
        }
      } else if (turn = 4){ //backward 
        if (currentAngle > (targetAngle - 1.0)) {
          M1FPin.write(1.0);
          M1BPin.write(1.0 - leftSpeed);
          M2FPin.write(1.0);
          M2BPin.write(1.0 - rightSpeed);
        } else if (currentAngle < (targetAngle - 1.0)) {
          M1FPin.write(1.0 - leftSpeed);
          M1BPin.write(1.0);
          M2FPin.write(1.0 - rightSpeed);
          M2BPin.write(1.0);
        } else {
          M1FPin.write(1.0);
          M1BPin.write(1.0);
          M2FPin.write(1.0);
          M2BPin.write(1.0);
        }
      } else if (turn = 2) { //left
          if (currentAngle > targetAngle) {
            M1FPin.write(1.0);
            M1BPin.write(1.0 - leftSpeed);
            M2FPin.write(1.0);
            M2BPin.write(1.0 - rightSpeed);
          } else if (currentAngle < targetAngle) {
            M1FPin.write(1.0 - leftSpeed);
            M1BPin.write(1.0);
            M2FPin.write(1.0 - rightSpeed);
            M2BPin.write(1.0);
          } else {
            M1FPin.write(1.0);
            M1BPin.write(1.0);
            M2FPin.write(1.0);
            M2BPin.write(1.0);
          }
      } else if (turn = 3) { // right
        if (currentAngle > targetAngle){
          M1FPin.write(1.0);
          M1BPin.write(1.0 - leftSpeed);
          M2FPin.write(1.0);
          M2BPin.write(1.0 - speed);
        } else if (currentAngle < targetAngle){
          M1FPin.write(1.0 - leftSpeed);
          M1BPin.write(1.0);
          M2FPin.write(1.0 - rightSpeed);
          M2BPin.write(1.0);
        } else {
          M1FPin.write(1.0);
          M1BPin.write(1.0);
          M2FPin.write(1.0);
          M2BPin.write(1.0);
        }
      } else { // stationary
        if (currentAngle > (targetAngle)){
          M1FPin.write(1.0);
          M1BPin.write(1.0 - leftSpeed);
          M2FPin.write(1.0);
          M2BPin.write(1.0 - rightSpeed);
        } else if (currentAngle < (targetAngle))  {
          M1FPin.write(1.0 - leftSpeed);
          M1BPin.write(1.0);
          M2FPin.write(1.0 - rightSpeed);
          M2BPin.write(1.0);
        } else {
          M1FPin.write(1.0);
          M1BPin.write(1.0);
          M2FPin.write(1.0);
          M2BPin.write(1.0);
        }
      }
      //-----------------------------------------------------------
      // Serial.print("Kp: ");
      // Serial.print(Kp);
      // Serial.print("\tKi: ");
      // Serial.print(Ki);
      // Serial.print("\tkd: ");
      // Serial.print(Kd);
      // Serial.println("\t");
      // Serial.println(speed);
      
    }
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    //Serial.println("Disconnected from central.");
  }
}