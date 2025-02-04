void setup() {
  // put your setup code here, to run once:

  #define PWM_RIGHT 24
  #define PWM_LEFT 23

  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(PWM_LEFT, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  analogWrite(PWM_RIGHT, 127);
  analogWrite(PWM_LEFT,127);
}
