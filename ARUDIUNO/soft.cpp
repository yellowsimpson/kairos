#include "softwareSerial.h"

SoftwareSerial mySerial(10, 11);

void setup(){
    Serial.begin(9600);
    mySerial.begin(9600);
}

int count = 1;
void loop(){
    Serial.print("value: ");
    Serial.print("value: ");
    mySerial.println(count);
    delay(1000);
    count ++;
}