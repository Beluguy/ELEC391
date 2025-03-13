#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <PID_v1.h>

#define BUFFER_SIZE 20

//MOTOR 1
#define M1F D7 //BIN1 - GREEN
#define M1B D8 //BIN2 - BLUE

//MOTOR 2
#define M2F D10 //AIN1 - BLUE
#define M2B D9 //AIN2 - GREEN

float Kp = 10.0, Ki = 0.0, Kd = 0.0;
double currentAngle = 0, targetAngle = 0, PWM;
float kAcc = 0.05, kGyro = 0.95;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

//Specify the links and initial tuning parameters
PID myPID(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd, DIRECT);

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

float turnCoeff, driveCoeff, KpNew, KiNew, KdNew;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
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

  Serial.println("Bluetooth® device active, waiting for connections...");

  //Set up gyroscope and pid
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
  // Wait for a BLE central to connect
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

    // Keep running while connected
    while (central.connected()) {
      // Check if the characteristic was written
      if (customCharacteristic.written()) {
        int length = customCharacteristic.valueLength();
        
        if (length == 20) { // Expecting 20 bytes (5 floats)
          uint8_t data[20];
          customCharacteristic.readValue(data, length);

          memcpy(&turnCoeff, data, 4);  // Extract first float
          memcpy(&driveCoeff, data + 4, 4); // Extract second float
          memcpy(&KpNew, data + 8, 4); // Extract third float
          memcpy(&KiNew, data + 12, 4); // Extract fourth float
          memcpy(&KdNew, data + 16, 4); // Extract fifth float
          
          Serial.print("Turn Value: "); Serial.print(turnCoeff);
          Serial.print(" | Forward Drive Val: "); Serial.println(driveCoeff);
          Serial.print("P: "); Serial.print(KpNew);
          Serial.print(" | I: "); Serial.print(KiNew);
          Serial.print(" | D: "); Serial.println(KdNew);

          Kp = KpNew;
          Ki = KiNew;
          Kd = KdNew;

        }
      
       /*
       // Get the length of the received data
        int length = customCharacteristic.valueLength();

        // Read the received data
        const unsigned char* receivedData = customCharacteristic.value();

        // Create a properly terminated string
        char receivedString[length + 1]; // +1 for null terminator
        memcpy(receivedString, receivedData, length);
        receivedString[length] = '\0'; // Null-terminate the string

        // Print the received data to the Serial Monitor
        Serial.print("Received data: ");
        Serial.println(receivedString);


        // Optionally, respond by updating the characteristic's value
        customCharacteristic.writeValue("Data received");
        */
      }

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

    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");
  }
}