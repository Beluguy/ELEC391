#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"
#include <PID_v1.h>
#include "mbed.h"

#define BUFFER_SIZE 20
#define PWM_FREQ 10000.0

#define M2B D10 //yellow: motor 2
#define M1F D9  //white:  motor 1
#define M1B D8  //green:  motor 1
#define M2F D7  //blue:   motor 2

mbed::PwmOut M2BPin( digitalPinToPinName( M2B ) );
mbed::PwmOut M1FPin ( digitalPinToPinName( M1F ) );
mbed::PwmOut M1BPin( digitalPinToPinName( M1B ) );
mbed::PwmOut M2FPin( digitalPinToPinName( M2F ) );

float Kp = 0.0, Ki = 0.0, Kd = 0.0;
double currentAngle = 0.0, targetAngle = 0.3, PWM;
float kAcc = 0.05, kGyro = 0.95;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate;

//Specify the links and initial tuning parameters
PID myPID(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd, DIRECT);

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
  BLE.advertise();

  //Serial.println("Bluetooth® device active, waiting for connections...");
  //----------------------------------------------------------------------

  //Set up gyroscope and pid
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
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
          static uint8_t data[12];
          customCharacteristic.readValue(data, length);

          //memcpy(&turnCoeff, data, 4);  // Extract first float
          //memcpy(&driveCoeff, data + 4, 4); // Extract second float
          memcpy(&Kp, data, 4); // Extract third float
          memcpy(&Ki, data + 4, 4); // Extract fourth float
          memcpy(&Kd, data + 8, 4); // Extract fifth float
          
          myPID.SetTunings(Kp, Ki, Kd);
        }
      }
      //----------------------------------------------------------------------------------
      
      //----------------complementary filter------------------------
      if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
        accAngle = RAD_TO_DEG*(accY/accZ);

        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        static double lastTime = millis();
        double dt = (millis() - lastTime) / 1000.0;
        lastTime = millis();
        gyroAngle = gyroZ * dt + currentAngle;
        Serial.println(dt);

        currentAngle = kGyro*(gyroAngle) + kAcc*(accAngle);
        //Serial.print("Current Angle: ");
        //Serial.print(currentAngle);
        //Serial.print("\t");
        //Serial.print(gyroAngle);
        //Serial.print("\t");
        //Serial.println(accAngle);
        // Serial.println("\t");
        }
      //-----------------------------------------------------------

      //----------------------PID---------------------------------
      myPID.Compute();
      static float speed;
      speed = abs(PWM)/255.0;
      /*
      static float speedNew;
      static bool speedFactorOver;

      
      speedNew = 1.05*speed;

      if (speedNew >= 1.0){
        speedFactorOver = 1;
      }
      */

      //Serial.println(speed);
      if (currentAngle > (targetAngle)) {
        /*
        analogWrite(M1F, 255);  
        analogWrite(M1B, 255-speed);   
        analogWrite(M2F, 255);  
        analogWrite(M2B, 255-speed);
        */
        // M1FPin.write(1.0);
        // M1BPin.write(1.0 - speed);
        // M2FPin.write(1.0);
        // M2BPin.write(1.0 - speed);
        M1FPin.write(speed);
        M1BPin.write(0.0);
        M2FPin.write(speed);
        M2BPin.write(0.0);
        
      } else if (currentAngle < (targetAngle))  {
        /*
        analogWrite(M1F, 255-speed);    
        analogWrite(M1B, 255);   
        analogWrite(M2F, 255-speed);   
        analogWrite(M2B, 255);
        */

        // M1FPin.write(1.0 - speed);
        // M1BPin.write(1.0);
        // M2FPin.write(1.0 - speed);
        // M2BPin.write(1.0);

        M1FPin.write(0.0);
        M1BPin.write(speed);
        M2FPin.write(0.0);
        M2BPin.write(speed);
      } else {
        /*
        analogWrite(M1F, 255);    
        analogWrite(M1B, 255);   
        analogWrite(M2F, 255);   
        analogWrite(M2B, 255);
        */

          // M1FPin.write(1.0);
          // M1BPin.write(1.0);
          // M2FPin.write(1.0);
          // M2BPin.write(1.0);

        M1FPin.write(0.0);
        M1BPin.write(0.0);
        M2FPin.write(0.0);
        M2BPin.write(0.0);
      }
      //----------------------------------------------------------
      /*
      static unsigned long loopStartTime = micros();
      static unsigned long loopTime = micros();
      static unsigned long maxTime = 0;
      static unsigned long minTime = 1000000; // Initialize with a large value
      loopTime = micros() - loopStartTime;

      if (loopTime > maxTime) maxTime = loopTime;
      if (loopTime < minTime) minTime = loopTime;

      Serial.println(maxTime);
      Serial.print("\t");
      Serial.println(minTime);
      */
    }
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    //Serial.println("Disconnected from central.");
  }
}