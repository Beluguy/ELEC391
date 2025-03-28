#include <ArduinoBLE.h>             // Bluetooth
#include "Arduino_BMI270_BMM150.h"  // IMU
#include "ArduPID.h"                // PID
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
int lastTurn = 0;
double currentAngle = 0.0, targetAngle = 0.0, PWM;
float kAcc = 0.15, kGyro = 0.85;
float accX, accY, accZ, gyroX, gyroY, gyroZ, accAngle, gyroAngle, SampleRate, SampleRatea;
double dt1, dt2;

// Define a custom BLE service and characteristic
BLEService customService("fc096266-ad93-482d-928c-c2560ea93a4e");
BLECharacteristic customCharacteristic("9ff0183d-6d83-4d05-a10e-55c142bee2d1", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);


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

  //Serial.println("Bluetooth® device active, waiting for connections...");
  //----------------------------------------------------------------------

  //Set up gyroscope and pid
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  myController.begin(&currentAngle, &PWM, &targetAngle, Kp, Ki, Kd);

  myController.setOutputLimits(-255, 255);
  myController.setWindUpLimits(-255, 255); // Groth bounds for the integral term to prevent integral wind-up
  //myController.setBias(-10);
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
        
        if (length == 13) { // Expecting 20 bytes (5 floats)
          static uint8_t data[13];
          customCharacteristic.readValue(data, length);
          lastTurn = turn;
          memcpy(&turn, data, 1);
          memcpy(&Kp, data + 1, 4); // Extract third float
          memcpy(&Ki, data + 5, 4); // Extract fourth float
          memcpy(&Kd, data + 9, 4); // Extract fifth float
          myController.setCoefficients(Kp, Ki, Kd);
        }
      }
      //----------------------------------------------------------------------------------

      //----------------complementary filter------------------------
      if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
        IMU.readAcceleration(accX, accY, accZ);
        accAngle = RAD_TO_DEG*atan(accY/accZ) - 0.5;

        IMU.readGyroscope(gyroX, gyroY, gyroZ);
        static double lastTimeInBLE = micros();
        dt2 = (micros() - lastTimeInBLE) / 1000000.0;
        lastTimeInBLE = micros();
        gyroAngle = -1.0 * gyroX * dt2 + currentAngle;
        //Serial.println(dt2,5);

        currentAngle = kGyro*(gyroAngle) + kAcc*(accAngle);
        // Serial.print("Current Angle: ");
        Serial.println(currentAngle);
        // Serial.print("\t");
        // Serial.print(gyroAngle);
        // Serial.print("\t");
        // Serial.println(accAngle);
        }
      //-----------------------------------------------------------

      //----------------------PID---------------------------------
      // if(turn == 10){
      //   myController.reset();
      //   delay(200);
      //   //Serial.println("RESET");
      //   turn = lastTurn;
      // }

      myController.compute();
      //-----------------------motor control-----------------------
      static float speed;
      speed = abs(PWM)/255.0;
      //if (speed < 0.05) speed = 0.05;

      if (currentAngle > (targetAngle)) {
        //speed = speed * 1.1;
        //if (speed > 1.0) speed = 1.0;
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
      //--------------------------------------------------------------
      // Serial.print(Kp);
      // Serial.print("\t");
      // Serial.print(Ki);
      // Serial.print("\t");
      // Serial.print(Kd);
      // Serial.print("\t");
      //Serial.println(speed);
      // Serial.print(lastTurn);
      // Serial.print("\t");
      // Serial.println(turn);
      //myController.debug(&Serial, "", PRINT_INPUT | PRINT_OUTPUT | PRINT_SETPOINT | PRINT_BIAS | PRINT_P | PRINT_I | PRINT_D );

    }
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    //Serial.println("Disconnected from central.");
  }
}