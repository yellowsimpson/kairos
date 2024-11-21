#include <Servo.h>

Servo myServo;

void setup() {
  Serial.begin(9600);  // 시리얼 통신을 9600 baud로 초기화
  myServo.attach(10);  // 서보모터를 10번 핀에 연결
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // 시리얼 포트에서 문자열 읽기

    if (command == "1") {  // "1"을 수신하면
      myServo.write(90);  // 서보모터를 90도로 회전
      delay(1000);        // 위치를 유지하기 위해 지연 (1초)
    }
  }
}
