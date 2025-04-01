#include <ArduinoBLE.h>             // Bluetooth
#include "Arduino_BMI270_BMM150.h"  // IMU
#include "mbed.h"                   // Customer PWM freq

#define BUFFER_SIZE 13
#define PWM_FREQ 10000.0

#define M1F D10 //Green: motor 1
#define M1B D9  //Blue:  motor 1
#define M2B D8  //Green: motor 2
#define M2F D7  //Blue:  motor 2
#define BLE_CHECK_INTERVAL 200
#define CAL_LED D11 // This LED will turn on when the IMU calibraiton is completed

#define ACC_STD 3
#define GYRO_STD 4

mbed::PwmOut M2BPin(digitalPinToPinName(M2B));
mbed::PwmOut M1FPin(digitalPinToPinName(M1F));
mbed::PwmOut M1BPin(digitalPinToPinName(M1B));
mbed::PwmOut M2FPin(digitalPinToPinName(M2F));

//--------------------------PID----------------------------------------------------------
float Kp = 0.0, Ki = 0.0, Kd = 0.0;
float pOut = 0.0, iOut = 0.0, dOut = 0.0;
float currentAngle = 0.0, lastAngle = 0.0, targetAngle = 0.0, currPWM = 0.0, lastPWM = 0.0, currError = 0.0, lastError = 0.0, dt;
//---------------------------------------------------------------------------------------

//-------------Comp Angle-------------------------------------------
float accX = 0.0, accY = 0.0, accZ = 0.0, gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
double accAngle, gyroAngle;

//kalman
float kalmanUncertainty;

//CALIBRATION
float gyroXCal = 0.0, accXCal = 0.0, accYCal = 0.0, accZCal = 0.0;
//-------------------------------------------------------------------

int turn = 0; // 0 = balance, 1 = forward, 2 = left, 3 = right, 4 = backward
unsigned long loopTime, lastBLECheck = 0;
bool isConnected;

// Define a custom BLE service and characteristic
BLEService customService("fc096266-ad93-482d-928c-c2560ea93a4e");
BLECharacteristic customCharacteristic("9ff0183d-6d83-4d05-a10e-55c142bee2d1", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);


void setup() {
  //Serial.begin(2000000);
  //while (!Serial);
  //---------------------ble-----------------------------------
  // Initialize the built-in LED to indicate connection status
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CAL_LED, OUTPUT);

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

  // Serial.println("Bluetooth® device active, waiting for connections...");
  //----------------------------------------------------------------------

  //Set up gyroscope and pid
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
  M2BPin.period(1.0/PWM_FREQ);
  M1FPin.period(1.0/PWM_FREQ);
  M1BPin.period(1.0/PWM_FREQ);
  M2FPin.period(1.0/PWM_FREQ);

 //-----------------ANGLE CALIBRATION--------------------------------
 // set init angle to 0
  for(int i = 0; i < 1500; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.2);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 1500;
  accXCal /= 1500;
  accYCal /= 1500;
  accZCal /= 1500;
  digitalWrite(CAL_LED, HIGH); // Turn on LED to indicate calibration complete
 //-----------------------------------------------------------------------
}

void loop() {
  unsigned long currentMillis = millis();
  // ---------------- BLE Handling ----------------
  if (currentMillis - lastBLECheck >= BLE_CHECK_INTERVAL) {
    lastBLECheck = currentMillis;
    BLE.poll();  // Efficiently handle BLE events
  
    //----------------------------BLE----------------------------------------
    BLEDevice central = BLE.central();     // Wait for a BLE central to connect

    if (central) {
      if(!isConnected) {
        // Serial.println("Connected to central: ");
        // Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection
        isConnected = true;
      }

      if (customCharacteristic.written()) {
      int length = customCharacteristic.valueLength();
      
        if (length == 13) {
          static uint8_t data[13];
          customCharacteristic.readValue(data, length);
          memcpy(&turn, data, 1);
          memcpy(&Kp, data + 1, 4); // Extract third float
          memcpy(&Ki, data + 5, 4); // Extract fourth float
          memcpy(&Kd, data + 9, 4); // Extract fifth float

          if (turn == 1) targetAngle += 0.2;
          else if (turn == 4) targetAngle -= 0.2;
          else if (!turn) targetAngle = 0.0;

          // if (turn == 1) bias += 5.0;
          // else if (turn == 4) bias -= 5.0;
          // else if (!turn) bias = 0.0;
          // Serial.print(turn);
          // Serial.print("\t");
          // Serial.println(targetAngle);
        }
      }
    } else {
      if (isConnected) {
        isConnected = false; 
        digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
      }
    }
  }
  //-----------------------------------------------------------------------

  //---------------------KALMAN FILTER----------------------------------------------------
  if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
    static unsigned long lastTime = micros();
    dt = (micros() - lastTime) / 1000000.0;
    lastTime = micros();
    //Serial.println(dt);

    accX -= accXCal;
    accY -= accYCal;
    accZ -= accZCal;
    gyroX -= gyroXCal;

    if(accZ < 0.1){
      accAngle = 0.0;
      gyroAngle = 0.0;
    } else {
      accAngle = RAD_TO_DEG * atan(accY/(sqrt(accZ*accZ + accX*accX)));
      gyroAngle = -1.0 * gyroX * dt + currentAngle;
    }
    
    currentAngle = currentAngle - dt * gyroX;
    kalmanUncertainty += dt * dt * GYRO_STD * GYRO_STD;

    float kalGain = kalmanUncertainty / (kalmanUncertainty + ACC_STD * ACC_STD);
    currentAngle += kalGain*(accAngle - currentAngle);
    kalmanUncertainty = (1.0 - kalGain) * kalmanUncertainty;
    
    // Serial.print("Current Angle: ");
    // Serial.print(currentAngle);
    // Serial.print("\t");
    // Serial.print(gyroAngle);
    // Serial.print("\t");
    // Serial.println(accAngle);
  }
  //--------------------------------------------------------------------------------------

  //----------------------PID---------------------------------
  currError = targetAngle - currentAngle; 

  pOut = Kp * currError;       
  dOut = -Kd *(currentAngle - lastAngle) / dt;                           
  iOut += (Ki * dt) * (currError + lastError) / 2.0;   // Integral term with trapezoidal integration

  static float remainingMax;
  remainingMax = 1000.0 - (pOut + dOut);              // clamp integral windup
  static float remainingMin;
  remainingMin = -1000.0 - (pOut + dOut);             // clamp integral windup
  iOut = constrain(iOut, remainingMin, remainingMax);  

  currPWM = pOut + dOut + iOut;
  currPWM = constrain(currPWM, -1000.0, 1000.0);
  // Serial.print(dOut,5);
  // Serial.print("\t");
  // Serial.println(iOut,5);

  lastAngle = currentAngle;
  lastError = currError;
  //---------------------------------------------------------

  //-----------------------motor control-----------------------
  static float speed;
  speed = abs(currPWM) / 1000.0;

  if (currentAngle > targetAngle && currentAngle < 20.0) {
    M1FPin.write(1.0);
    M1BPin.write(1.0 - speed);
    M2FPin.write(1.0);
    M2BPin.write(1.0 - speed);
  } else if (currentAngle < targetAngle && currentAngle > -20.0) { 
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
  //--------------------------------------------------------------
  // Serial.print(Kp,5);
  // Serial.print("\t");
  // Serial.print(Ki,5);
  // Serial.print("\t");
  // Serial.print(Kd,5);
  // Serial.print("\t");
  // Serial.println(speed,5);
}