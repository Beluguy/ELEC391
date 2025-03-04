//MOTOR 1
#define M1F D7 //BIN1 - GREEN
#define M1B D8 //BIN2 - BLUE
//MOTOR 2
#define M2F D10 //AIN1 - BLUE
#define M2B D9 //AIN2 - GREEN



void setup() {
  // put your setup code here, to run once:



  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(M1F, 255);
  analogWrite(M1B, 255-38);
  analogWrite(M2F, 255);
  analogWrite(M2B, 255-38);

}
