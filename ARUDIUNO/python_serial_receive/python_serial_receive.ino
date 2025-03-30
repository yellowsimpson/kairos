#include "SoftwareSerial.h"

SoftwareSerial mySerial(10, 11);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mySerial.begin(9600);
}

int count = 1;
void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("value: ");
  Serial.println(count);
  mySerial.println(count);
  delay(1000);
  count++;
}
