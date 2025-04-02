#include "Arduino_BMI270_BMM150.h"  // IMU
#define CAL_LED D11 // This LED will turn on when the IMU calibraiton is completed

float accX = 0.0, accY = 0.0, accZ = 0.0, gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;

float gyroXCal = 0.0, accXCal = 0.0, accYCal = 0.0, accZCal = 0.0; //CALIBRATION

void setup() {
  Serial.begin(2000000);
  while (!Serial);
  pinMode(CAL_LED, OUTPUT);

  //Set up gyroscope and pid
  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU!");
    while (1);
  }

  for(int i = 0; i < 500; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.5);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 500;
  accXCal /= 500;
  accYCal /= 500;
  accZCal = accZCal / 500 - 1.0;
  Serial.print("i = 500 ");
  Serial.print("\tgyroX Cal: ");
  Serial.print(gyroXCal,5);
  Serial.print("\taccXCal: ");
  Serial.print(accXCal,5);
  Serial.print("\taccYCal: ");
  Serial.print(accYCal,5);
  Serial.print("\taccZCal: ");
  Serial.println(accZCal,5);

  for(int i = 0; i < 1000; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.5);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 1000;
  accXCal /= 1000;
  accYCal /= 1000;
  accZCal = accZCal / 1000 - 1.0;
  Serial.print("i = 1000 ");
  Serial.print("\tgyroX Cal: ");
  Serial.print(gyroXCal,5);
  Serial.print("\taccXCal: ");
  Serial.print(accXCal,5);
  Serial.print("\taccYCal: ");
  Serial.print(accYCal,5);
  Serial.print("\taccZCal: ");
  Serial.println(accZCal,5);

  for(int i = 0; i < 2000; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.5);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 2000;
  accXCal /= 2000;
  accYCal /= 2000;
  accZCal = accZCal / 2000 - 1.0;
  Serial.print("i = 2000 ");
  Serial.print("\tgyroX Cal: ");
  Serial.print(gyroXCal,5);
  Serial.print("\taccXCal: ");
  Serial.print(accXCal,5);
  Serial.print("\taccYCal: ");
  Serial.print(accYCal,5);
  Serial.print("\taccZCal: ");
  Serial.println(accZCal,5);

  for(int i = 0; i < 4000; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.5);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 4000;
  accXCal /= 4000;
  accYCal /= 4000;
  accZCal = accZCal / 4000 - 1.0;
  Serial.print("i = 4000 ");
  Serial.print("\tgyroX Cal: ");
  Serial.print(gyroXCal,5);
  Serial.print("\taccXCal: ");
  Serial.print(accXCal,5);
  Serial.print("\taccYCal: ");
  Serial.print(accYCal,5);
  Serial.print("\taccZCal: ");
  Serial.println(accZCal,5);

  for(int i = 0; i < 8000; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.5);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 8000;
  accXCal /= 8000;
  accYCal /= 8000;
  accZCal = accZCal / 8000 - 1.0;
  Serial.print("i = 8000 ");
  Serial.print("\tgyroX Cal: ");
  Serial.print(gyroXCal,5);
  Serial.print("\taccXCal: ");
  Serial.print(accXCal,5);
  Serial.print("\taccYCal: ");
  Serial.print(accYCal,5);
  Serial.print("\taccZCal: ");
  Serial.println(accZCal,5);

  for(int i = 0; i < 16000; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.5);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 16000;
  accXCal /= 16000;
  accYCal /= 16000;
  accZCal = accZCal / 16000 - 1.0;
  Serial.print("i = 16000 ");
  Serial.print("\tgyroX Cal: ");
  Serial.print(gyroXCal,5);
  Serial.print("\taccXCal: ");
  Serial.print(accXCal,5);
  Serial.print("\taccYCal: ");
  Serial.print(accYCal,5);
  Serial.print("\taccZCal: ");
  Serial.println(accZCal,5);

  for(int i = 0; i < 32000; i++){
    if (IMU.readAcceleration(accX, accY, accZ) && IMU.readGyroscope(gyroX, gyroY, gyroZ)) {
      gyroXCal += gyroX;
      accXCal += accX;
      accYCal += accY;
      accZCal += accZ;
      delay(0.5);
    } else {
      i--;  // Decrement counter to repeat this iteration
    }
  }
  gyroXCal /= 32000;
  accXCal /= 32000;
  accYCal /= 32000;
  accZCal = accZCal / 32000 - 1.0;
  Serial.print("i = 32000 ");
  Serial.print("\tgyroX Cal: ");
  Serial.print(gyroXCal,5);
  Serial.print("\taccXCal: ");
  Serial.print(accXCal,5);
  Serial.print("\taccYCal: ");
  Serial.print(accYCal,5);
  Serial.print("\taccZCal: ");
  Serial.println(accZCal,5);

  digitalWrite(CAL_LED, HIGH); // Turn on LED to indicate calibration complete
}
void loop() {
}