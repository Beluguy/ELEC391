#include <ArduinoBLE.h>             // Bluetooth
#include "Arduino_BMI270_BMM150.h"  // IMU
#include "ArduPID.h"                // PID
//#include <Wire.h>                 // I2C
//#include <AS5600.h>               // Encoder
#include "mbed.h"                   // Customer PWM freq

#define BUFFER_SIZE 20
#define PWM_FREQ 10000.0

#define M1F D10 //Green: motor 1
#define M1B D9  //Blue:  motor 1
#define M2B D8  //Green: motor 2
#define M2F D7  //Blue:  motor 2

mbed::PwmOut M2BPin(digitalPinToPinName(M1B));
mbed::PwmOut M1FPin(digitalPinToPinName(M1F));
mbed::PwmOut M1BPin(digitalPinToPinName(M2B));
mbed::PwmOut M2FPin(digitalPinToPinName(M2F));

ArduPID myController;

float Kp = 0.0, Ki = 0.0, Kd = 0.0;
int turn = 0;
double currentAngle = 0.0, targetAngle = 0.0, PWM;
float kAcc = 0.1, kGyro = 0.9;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle;
unsigned long dt, loopTime;
unsigned long lastBLECheck = 0;
const int BLE_CHECK_INTERVAL = 200;

bool isConnected, newDataReceived;

// Define a custom BLE service and characteristic
BLEService customService("fc096266-ad93-482d-928c-c2560ea93a4e");
BLECharacteristic customCharacteristic("9ff0183d-6d83-4d05-a10e-55c142bee2d1", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);


void setup() {
  Serial.begin(19200);
  //while (!Serial);
  //---------------------ble-----------------------------------
  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);

  if (!BLE.begin()) {
    //Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the device name and local name
  BLE.setLocalName("BLE-enjoyer");
  BLE.setDeviceName("BLE-enjoyer");

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
  myController.begin(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd);

  myController.setOutputLimits(-255, 255);
  //myController.setWindUpLimits(-100, 100); // Groth bounds for the integral term to prevent integral wind-up
  //myController.setSampleTime(10);
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
  // static unsigned long lastLoopTime = millis();
  // loopTime = (millis() - lastLoopTime);
  // lastLoopTime = millis();
  // Serial.print("LOOP TIME: ");
  // Serial.print(loopTime);
  // Serial.print("\t");


  unsigned long currentMillis = millis();

  // ---------------- BLE Handling ----------------
  if (currentMillis - lastBLECheck >= BLE_CHECK_INTERVAL) {
    lastBLECheck = currentMillis;
    BLE.poll();  // Efficiently handle BLE events
  
    //----------------------------ble----------------------------------------

    // Wait for a BLE central to connect
    BLEDevice central = BLE.central();

    if (central) {
      if(!isConnected){
        Serial.println("Connected to central: ");
        Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection
        isConnected = true;
      }


      if (customCharacteristic.written()) {
      int length = customCharacteristic.valueLength();
      
        if (length == 13) { // Expecting 20 bytes (5 floats)
          static uint8_t data[13];
          customCharacteristic.readValue(data, length);

          memcpy(&Kp, data + 1, 4); // Extract third float
          memcpy(&Ki, data + 5, 4); // Extract fourth float
          memcpy(&Kd, data + 9, 4); // Extract fifth float
          myController.setCoefficients(Kp, Ki, Kd);
        }
      }
    } else {
      if(isConnected){
        isConnected = false; 
        digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
      }
    }

    // Check if the characteristic was written
    
    //-----------------------------------------------------------------------
  }
  //----------------complementary filter------------------------
  if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {

    // Serial.print("gyro xyz: ");
    // Serial.print(gyroX);
    // Serial.print("\t");
    // Serial.print(gyroY);
    // Serial.print("\t");
    // Serial.print(gyroZ);
    // Serial.print("\t");
    // Serial.print("Acc xyz");
    // Serial.print(accX);
    // Serial.print("\t");
    // Serial.print(accY);
    // Serial.print("\t");
    // Serial.println(accZ);
    //Serial.print("\t");
    //Serial.println(accAngle);

    static unsigned long lastTime = millis();
    dt = (millis() - lastTime);
    lastTime = millis();
    
    Serial.println(dt);

    if(accZ < 0.1){
      accAngle = 0.0;
      gyroAngle = 0.0;
    } else {
      accAngle = RAD_TO_DEG*atan(accY/accZ) + 0.3;
      gyroAngle = -1.0 * gyroX * dt / 1000.000 + currentAngle;
    }

    currentAngle = kGyro*(gyroAngle) + kAcc*(accAngle);
    // Serial.print("Current Angle: ");
    // Serial.print(currentAngle);
    // Serial.print("\t");
    // Serial.print(gyroAngle);
    // Serial.print("\t");
    // Serial.println(accAngle);
  }
  //-----------------------------------------------------------

  //----------------------PID---------------------------------
  myController.compute();


  //-------------------MOTOR-------------------------
  static float speed;
  speed = abs(PWM)/255.0;

  if (currentAngle > (targetAngle)) {
    M1FPin.write(1.0);
    M1BPin.write(1.0 - speed);
    M2FPin.write(1.0);
    M2BPin.write(1.0 - speed);
  } else if (currentAngle < (targetAngle))  {
    M1FPin.write(1.0 - speed);
    M1BPin.write(1.0);
    M2FPin.write(1.0 - speed);
    M2BPin.write(1.0);
  } else {
    M1FPin.write(1.0);
    M1BPin.write(1.0);
    M2FPin.write(1.0);
    M2BPin.write(1.0);
  }
  //----------------------------------------------------------

  // Serial.print(Kp);
  // Serial.print("\t");
  // Serial.print(Ki);
  // Serial.print("\t");
  // Serial.print(Kd);
  // Serial.print("\t");
  // Serial.println(speed);

  
  //Serial.println("Disconnected from central.");
}