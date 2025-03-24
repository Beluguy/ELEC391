#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include "ArduPID.h"
ArduPID myController;

#define BUFFER_SIZE 20
#define M2B D10 //yellow: motor 2
#define M1F D9  //white:  motor 1
#define M1B D8  //green:  motor 1
#define M2F D7  //blue:   motor 2

float Kp = 0.0, Ki = 0.0, Kd = 0.0;
double currentAngle = 0.0, targetAngle = 0.0, PWM = 0.0;
float kAcc = 0.3, kGyro = 0.7;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

// Define a custom BLE service and characteristic
BLEService customService("fc096266-ad93-482d-928c-c2560ea93a4e");
BLECharacteristic customCharacteristic("9ff0183d-6d83-4d05-a10e-55c142bee2d1", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

float turnCoeff, driveCoeff;

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
  //Serial.println("before advertise");
  BLE.advertise();

  //Serial.println("Bluetooth® device active, waiting for connections...");
  //----------------------------------------------------------------------

  //Set up gyroscope and pid
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  SampleRate = IMU.gyroscopeSampleRate();
  myController.begin(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd);   //Specify the links and initial tuning parameters
  myController.setOutputLimits(-255, 255);
  myController.start();

  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
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
        
        if (length == 12) { // Expecting 20 bytes (5 floats)
          uint8_t data[12];
          customCharacteristic.readValue(data, length);

          //memcpy(&turnCoeff, data, 4);  // Extract first float
          //memcpy(&driveCoeff, data + 4, 4); // Extract second float
          memcpy(&Kp, data, 4); // Extract third float
          memcpy(&Ki, data + 4, 4); // Extract fourth float
          memcpy(&Kd, data + 8, 4); // Extract fifth float
          
          myController.stop();
          myController.start();
          }
      //----------------------------------------------------------------------------------
      
      //----------------complementary filter------------------------
      if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
        accAngle = RAD_TO_DEG*(atan(accY/accZ));

        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        gyroAngle = (1.0/SampleRate)*gyroX;

        currentAngle = kGyro*(gyroAngle + currentAngle) + kAcc*(accAngle);
        }
      //-----------------------------------------------------------

      //----------------------PID---------------------------------
      myController.compute();
      int speed = abs(PWM);
      if (speed < 25) speed = 25;
      if (currentAngle > (targetAngle + 2.0)) {
        analogWrite(M1F, 255);  
        analogWrite(M1B, 255-speed);   
        analogWrite(M2F, 255);  
        analogWrite(M2B, 255-speed);
      } else if (currentAngle < (targetAngle - 2.0)) {
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

      if (currentAngle > 30.0) myController.stop();
      else myController.start();

      myController.debug(&Serial, "myController", PRINT_INPUT    | // Can include or comment out any of these terms to print
                                              PRINT_OUTPUT   | // in the Serial plotter
                                              PRINT_SETPOINT |
                                              PRINT_BIAS     |
                                              PRINT_P        |
                                              PRINT_I        |
                                              PRINT_D);
      //----------------------------------------------------------
      }
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    //Serial.println("Disconnected from central.");
    }
  }
}