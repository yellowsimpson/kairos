#include "Motor.h"
#include "Arduino.h"

// 생성자  클래스이름::클래스이름()
Motor::Motor(int IA, int IB){
  ia = IA;
  ib = IB;
}

void Motor::begin(){
  pinMode(ia, OUTPUT);
  pinMode(ib, OUTPUT);
}

void Motor::stop(){
  digitalWrite(ia,LOW);
  digitalWrite(ib,LOW);
}

void Motor::go_forward(){
  digitalWrite(ia,HIGH);
  digitalWrite(ib,LOW);
}
void Motor::go_backward(){
  digitalWrite(ia,LOW);
  digitalWrite(ib,HIGH);
}

