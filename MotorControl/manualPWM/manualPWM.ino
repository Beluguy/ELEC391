  //MOTOR 1
  #define PWM_1 D7 //BIN1 - GREEN
  #define PWM_2 D8 //BIN2 - BLUE
  //MOTOR 2
  #define PWM_3 D9 //AIN2 - GREEN
  #define PWM_4 D10 //AIN1 - BLUE


void setup() {
  // put your setup code here, to run once:



  pinMode(PWM_1, OUTPUT);
  pinMode(PWM_2, OUTPUT);
  pinMode(PWM_3, OUTPUT);
  pinMode(PWM_4, OUTPUT);

}

void loop() {
  //Motor 1
  digitalWrite(PWM_1, HIGH);
  digitalWrite(PWM_2, LOW);

  
  // //Motor 2
  // digitalWrite(PWM_3, HIGH);
  // digitalWrite(PWM_4, LOW);

}
