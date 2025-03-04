  //CURRENTLY NOT WORKING
  
  #include "mbed.h"
  
  //MOTOR 1
  #define M1F D7 //BIN1 - GREEN
  #define M1B D8 //BIN2 - BLUE
  //MOTOR 2
  #define M2F D10 //AIN1 - BLUE
  #define M2B D9 //AIN2 - GREEN


  mbed::PwmOut (M1F);
  mbed::PwmOut (M1B);
  
  mbed::PwmOut (M2F);
  mbed::PwmOut (M2B);
  
void setup()
{
  
  M1F.period_ms(0.002);
  M1B.period_ms(0.002);
  M2F.period_ms(0.002);
  M2B.period_ms(0.002);

  //Motor 1
  // analogWrite(M1F, 255);
  // analogWrite(M1B, 0); 
  M1F = 0.5f;
  M1B = 0.0f;
  
  // //Motor 2
  // analogWrite(M2F, 255);
  // analogWrite(M2B, 0);
  M2F = 0.5f;
  M2B = 0.0f;

  while(1);
}

// void setup() {
//   // put your setup code here, to run once:



//   pinMode(M1F, OUTPUT);
//   pinMode(M1B, OUTPUT);
//   pinMode(M2F, OUTPUT);
//   pinMode(M2B, OUTPUT);

// }

// void loop() {
//   //Motor 1
//   analogWrite(M1F, 255);
//   analogWrite(M1B, 0); 

  
//   // //Motor 2
//   analogWrite(M2F, 255);
//   analogWrite(M2B, 0);


// }
