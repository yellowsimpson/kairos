// 첫 번째 아두이노 (송신기)

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // Rx, Tx

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600); // 하드웨어 시리얼 모니터 출력용
}

void loop() {
  // PC에서 아두이노 IDE 시리얼 모니터 (a90b) -> 아두이노 -> terra term (PC)
  if(Serial.available() >0){
    String uga = Serial.readStringUntil('\n');
    //Serial.println(uga);
    mySerial.println(uga);
    int start = uga.indexOf('a');
    int end = uga.indexOf('b');
    String sub_string = uga.substring(start+1, end);
    mySerial.println(sub_string);
  }
  Serial.println("simpson");
  mySerial.println("simpson");
  delay(1000);
}

