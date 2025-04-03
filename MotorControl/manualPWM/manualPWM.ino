  #include "mbed.h"
  
#define M2B D10 //yellow: motor 2
#define M1F D9  //white:  motor 1
#define M1B D8  //green:  motor 1
#define M2F D7  //blue:   motor 2

mbed::PwmOut M2BPin( digitalPinToPinName( M2B ) );
mbed::PwmOut M1FPin ( digitalPinToPinName( M1F ) );
mbed::PwmOut M1BPin( digitalPinToPinName( M1B ) );
mbed::PwmOut M2FPin( digitalPinToPinName( M2F ) );

void setup() {
  // put your setup code here, to run once:
  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);

  M2BPin.period(1.0/10000.0);
  M1FPin.period(1.0/10000.0);
  M1BPin.period(1.0/10000.0);
  M2FPin.period(1.0/10000.0);




}

void loop() {
  //Motor 1
  M2BPin.write(0.5);
  M1FPin.write(0.75);
  M1BPin.write(0.25);
  M2FPin.write(0.0);
  //analogWrite(M2F, 128);
}
