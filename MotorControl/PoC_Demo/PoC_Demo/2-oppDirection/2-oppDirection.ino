//MOTOR 1
#define M1F D7 //BIN1 - GREEN
#define M1B D8 //BIN2 - BLUE
//MOTOR 2

#define M2F D10 //AIN1 - BLUE
#define M2B D9 //AIN2 - GREEN


void setup() {
//Motor 1 Setup

pinMode(M1F, OUTPUT);
pinMode(M1B, OUTPUT);

//Motor 2 Setup
pinMode(M2F, OUTPUT);
pinMode(M2B, OUTPUT);

}

void loop() {
    //Motor 1
    analogWrite(M1F, 0);
    analogWrite(M1B, 255 - 1 * 64);

    //Motor 2
    analogWrite(M2F, 0);
    analogWrite(M2B, 255 - 1 * 64);

    delay(5000);

    //Motor 1
    analogWrite(M1F, 0);
    analogWrite(M1B, 255 - 3 * 64);

    //Motor 2
    analogWrite(M2F, 0);
    analogWrite(M2B, 255 - 3 * 64);

    delay(5000);

    //255 (100%), 191 (75%), 127 (50%), 63 (25%), 0 

}
