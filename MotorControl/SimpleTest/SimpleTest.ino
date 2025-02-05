  #define PWM_1 A7
  #define PWM_2 A6
  #define PWM_3 A5
  #define PWM_4 A4


void setup() {
  // put your setup code here, to run once:



  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(PWM_1, 0);
  analogWrite(PWM_2, 77);
  analogWrite(PWM_3, 127);
  analogWrite(PWM_4, 255);

}
