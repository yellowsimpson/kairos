#include "SoftwareSerial.h"

SoftwareSerial Serial2(7,8);

void setup(){
  Serial.begin(115200);
  Serial.begin(9600);
}

void loop(){
  if(Serial.available()){
    String a = Serial.readStringUntil('\n');
    Serial2.println(a);
    int start = 0;
    int end = a.indexOf(',');
    String a_sub = a.substring(start+1, end);
    Serial2.println(a_sub);
    }
  }

