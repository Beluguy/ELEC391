#include <ArduinoBLE.h>             // Bluetooth
#include "Arduino_BMI270_BMM150.h"  // IMU
#include "mbed.h"                   // Customer PWM freq

// BLE constants
#define BUFFER_SIZE 13
#define BLE_CHECK_INTERVAL 100

//IMU Calibration constants for 1600 Hz, +- 2G, bwp = normal for both
#define gyroXCal -0.5
#define accXCal 0.03
#define accYCal -0.009
#define accZCal 0.02

// // IMU Calibration constants for 1600 Hz, +- 2G
// #define gyroXCal -0.54
// #define accXCal 0.028
// #define accYCal -0.0085
// #define accZCal 0.022

// // IMU Calibration constants for 1600 Hz, +- 4G
// #define gyroXCal -0.65
// #define accXCal 0.02
// #define accYCal -0.01
// #define accZCal 0.011

// Kalman Filter constants
#define ACC_STD 2
#define GYRO_STD 1.5
#define GYRO_STD_SQUARED GYRO_STD * GYRO_STD
#define ACC_STD_SQUARED ACC_STD * ACC_STD

// Motor constants & init
#define M1F D10  //Green: motor 1
#define M1B D9   //Blue:  motor 1
#define M2B D8   //Green: motor 2
#define M2F D7   //Blue:  motor 2
mbed::PwmOut M2BPin(digitalPinToPinName(M2B));
mbed::PwmOut M1FPin(digitalPinToPinName(M1F));
mbed::PwmOut M1BPin(digitalPinToPinName(M1B));
mbed::PwmOut M2FPin(digitalPinToPinName(M2F));
#define PWM_FREQ 10000.0
#define PWM_PERIOD 1.0 / PWM_FREQ

//-------------------------------------------------------PID for balancing----------------------------------------------------------
float Kp = 0.0, Ki = 0.0, Kd = 0.0, remainingMax, remainingMin;
float pOut = 0.0, iOut = 0.0, dOut = 0.0;
float currentAngle = 0.0, targetAngle = 0.0, lastAngle = 0.0, currPWM = 0.0, lastPWM = 0.0, currError = 0.0, lastError = 0.0, dt, speed;
//--------------------------------------------------------------------------------------------------------------------------------------

//------------------------------Kalman Filter-----------------------------------
float accX = 0.0, accY = 0.0, accZ = 0.0, gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
double accAngle, gyroAngle;
float kalmanUncertainty = ACC_STD_SQUARED, kalGain;
//------------------------------------------------------------------------------

//-----------------------------direction & turning -----------------------------------
int lastTurn = 0, turn = 0;  // 0 = balance, 1 = forward, 2 = left, 3 = right, 4 = backward
//------------------------------------------------------------------------------------

//--------------------------------------BLE-----------------------------------------
// Define a custom BLE service and characteristic
BLEService customService("fc096266-ad93-482d-928c-c2560ea93a4e");
BLECharacteristic customCharacteristic("9ff0183d-6d83-4d05-a10e-55c142bee2d1", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);
unsigned long lastBLECheck = 0, currentMillis;
bool isConnected = false;
int length;
//----------------------------------------------------------------------------------

//-----------------------Finding loop time---------------------------
// unsigned long maxLoopTime = 0;
// unsigned long minLoopTime = 1e6; // Initialize with a large value
// unsigned long totalLoopTime = 0;
// int loopCount = 0;
// #define PRINT_INTERVAL 2000 // Print every 2000 loops
//-------------------------------------------------------------------

void setup() {
  //Serial.begin(2000000);
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

  // Serial.println("Bluetooth® device active, waiting for connections...");
  //----------------------------------------------------------------------

  //Set up IMU
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }

  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);
  M2BPin.period(PWM_PERIOD);
  M1FPin.period(PWM_PERIOD);
  M1BPin.period(PWM_PERIOD);
  M2FPin.period(PWM_PERIOD);
}

void loop() {
  //unsigned long start = micros();
  currentMillis = millis();
  // ---------------- BLE Handling ---------------------------------------------
  if (currentMillis - lastBLECheck >= BLE_CHECK_INTERVAL) {
    lastBLECheck = currentMillis;
    BLE.poll();  // Efficiently handle BLE events

    //----------------------------BLE----------------------------------------
    BLEDevice central = BLE.central();  // Wait for a BLE central to connect

    if (central) {
      if (!isConnected) {
        // Serial.println("Connected to central: ");
        // Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH);  // Turn on LED to indicate connection
        isConnected = true;
      }
      if (customCharacteristic.written()) {
        length = customCharacteristic.valueLength();

        if (length == 13) {
          static uint8_t data[13];
          customCharacteristic.readValue(data, length);
          memcpy(&turn, data, 1);
          memcpy(&Kp, data + 1, 4);  // Extract third float
          memcpy(&Ki, data + 5, 4);  // Extract fourth float
          memcpy(&Kd, data + 9, 4);  // Extract fifth float

          //Kp = 160.0; Ki = 1500; Kd = 2.0;
          if (turn == 5) targetAngle += 1.0;
          else if (turn == 6) targetAngle -= 1.0;
        }
      }
    } else {
      if (isConnected) {
        isConnected = false;
        Kp = 0.0; Ki = 0.0; Kd = 0.0;
        speed = 0.0;
        digitalWrite(LED_BUILTIN, LOW);  // Turn off LED when disconnected
      }
    }
  }
  //---------------------------------------------------------------------------------

  //-----------------direction control---------------------
  if (turn == 1) targetAngle = 0.6;
  else if (turn == 4) targetAngle = -0.6;
  else if (turn == 0 && isConnected) targetAngle = 0.0;
  //-------------------------------------------------------

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

    accAngle = RAD_TO_DEG * atan(accY / sqrt(accZ * accZ + accX * accX));

    currentAngle = currentAngle - dt * gyroX;                            // Prediction of current angle
    kalmanUncertainty = kalmanUncertainty + dt * dt * GYRO_STD_SQUARED;  // Uncertainty of the prediction
    kalGain = kalmanUncertainty / (kalmanUncertainty + ACC_STD_SQUARED); // Get Kalman gain for the next line
    currentAngle = currentAngle + kalGain * (accAngle - currentAngle);  // New angle predicton
    kalmanUncertainty = (1.0 - kalGain) * kalmanUncertainty;  // Calculate new uncertainty

    // Serial.print("turn:");
    // Serial.print(lastTurn);
    // Serial.print("\t");
    // Serial.print(turn);
    // Serial.print("\t");
    // Serial.println(targetAngle);
    // Serial.print("\tKp: ");
    // Serial.print(Kp);
    // Serial.print("\tKi: ");
    // Serial.print(Ki);
    // Serial.print("\tKd: ");
    // Serial.println(Kd);

    // Serial.print(accX);
    // Serial.print("\t");
    // Serial.print(accY);
    // Serial.print("\t");
    // Serial.print(accZ);
    // Serial.print("\t");
    // Serial.print(gyroX);
    // Serial.print("\t");
    // Serial.print(accAngle);
    // Serial.print("\tCurrentAngle: ");
    // Serial.print(currentAngle);
  }
  //--------------------------------------------------------------------------------------

  //----------------------PID---------------------------------
  currError = targetAngle - currentAngle;

  pOut = Kp * currError;
  dOut = -Kd * (currentAngle - lastAngle) / dt;

  if (Ki != 0.0) { // to ensure iOut = 0 when Ki is 0 
    iOut += (Ki * dt) * (currError + lastError) / 2.0;
    remainingMax = 1000.0 - (pOut + dOut);
    remainingMin = -1000.0 - (pOut + dOut);
    iOut = constrain(iOut, remainingMin, remainingMax);
  } else iOut = 0.0;

  currPWM = constrain(pOut + dOut + iOut, -1000.0, 1000.0);

  // Serial.print("\tremainingMax: ");
  // Serial.print(remainingMax,5);
  // Serial.print("\tremainingMin: ");
  // Serial.print(remainingMin,5);
  // Serial.print("\tCurrPWM: ");
  // Serial.print(currPWM,5);
  // Serial.print("\tcurrdOut: ");
  // Serial.print(currdOut,5);
  // Serial.print("\tiOut: ");
  // Serial.println(iOut,5);

  lastAngle = currentAngle;
  lastError = currError;
  //---------------------------------------------------------

  //-----------------------motor control-----------------------
  speed = abs(currPWM) / 1000.0;
  // Serial.print("\tspeed: ");
  // Serial.println(speed,5);

  if (currPWM < 0.0 && currentAngle < 16.0) { // forward
     if(turn == 2){
      M1FPin.write(1.0);
      M1BPin.write(1.0);
      M2FPin.write(1.0);
      M2BPin.write(1.0 - speed);
    } else if (turn == 3){
      M1FPin.write(1.0 );
      M1BPin.write(1.0 - speed);
      M2FPin.write(1.0);
      M2BPin.write(1.0);
    } else {
      M1FPin.write(1.0);
      M1BPin.write(1.0 - speed);
      M2FPin.write(1.0);
      M2BPin.write(1.0 - speed);
    }
  } else if (currPWM > 0.0 && currentAngle > -16.0) { // backward
     if(turn == 2){
      M1FPin.write(1.0 - speed);
      M1BPin.write(1.0);
      M2FPin.write(1.0);
      M2BPin.write(1.0 );
    } else if (turn == 3){
      M1FPin.write(1.0);
      M1BPin.write(1.0 );
      M2FPin.write(1.0 - speed);
      M2BPin.write(1.0);
    } else {
      M1FPin.write(1.0 - speed);
      M1BPin.write(1.0);
      M2FPin.write(1.0 - speed);
      M2BPin.write(1.0);
    }
  } else {
    M1FPin.write(1.0);
    M1BPin.write(1.0);
    M2FPin.write(1.0);
    M2BPin.write(1.0);
  }
  //--------------------------------------------------------------

  // unsigned long elapsed = micros() - start;
  // totalLoopTime += elapsed;
  // loopCount++;

  // if (elapsed > maxLoopTime) maxLoopTime = elapsed;
  // if (elapsed < minLoopTime) minLoopTime = elapsed;

  // if (loopCount >= PRINT_INTERVAL) {
  //   float avg = (totalLoopTime / 1000.0) / loopCount;
  //   Serial.print(avg);
  //   Serial.print("\t");
  //   Serial.print(minLoopTime / 1000.0);
  //   Serial.print("\t");
  //   Serial.println(maxLoopTime / 1000.0);

  //   totalLoopTime = 0;
  //   loopCount = 0;
  //   maxLoopTime = 0;
  //   minLoopTime = 1e6;
  // }
  lastTurn = turn;
}