  #define PWM_1 D10
  #define PWM_2 D9
  #define PWM_3 D8
  #define PWM_4 D7


void setup() {
  // put your setup code here, to run once:



  pinMode(M1F, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2F, OUTPUT);
  pinMode(M2B, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(M1F, 0);
  analogWrite(M1B, 77);
  analogWrite(M2F, 127);
  analogWrite(M2B, 255);

}
