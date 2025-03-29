#include <Servo.h>

const int SERVO = 10;
Servo servo;

void setup(){
    servo.attach(SERVO);
    servo.write(0);

    delay(1000);

    for(int cnt = 0; cnt <= 2; cnt++){
        servo.write(0);
        delay(1000);;
        servo.write(180);
        delay(1000);
    }
    servo.detach();
}

void loop(){
    
}

