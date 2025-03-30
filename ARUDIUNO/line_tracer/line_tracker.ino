int IR1 = 3;
int IR2 = 4;
int LED = 13;
int A_IA = 5;
int A_IB = 6;
int B_IA = 11;
int B_IB = 10;
void setup(){
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(LED,OUTPUT);
  Serial.begin(115200);
  pinMode(A_IA,OUTPUT);
  pinMode(A_IB,OUTPUT);
  pinMode(B_IA,OUTPUT);
  pinMode(B_IB,OUTPUT);
}

void loop() {
  int IRsensor1 = digitalRead(IR1);
  int IRsensor2 = digitalRead(IR2);
  if (IRsensor1 != HIGH && IRsensor2 != HIGH) { // 두 센서 모두 감지
    analogWrite(A_IA, 0);
    analogWrite(A_IB, 80);
    analogWrite(B_IA,80);
    analogWrite(B_IB,0);
    Serial.println("forward");

  } else if (IRsensor1 != HIGH && IRsensor2 != LOW) { // IR1 감지되지 않음, IR2 감지
    analogWrite(A_IA, 0);
    analogWrite(A_IB, 0);
    analogWrite(B_IA, 80);
    analogWrite(B_IB, 0);
     Serial.println("right");

  } else if (IRsensor1 != LOW && IRsensor2 != HIGH) { // IR1 감지, IR2 감지되지 않음
    analogWrite(A_IA, 0);
    analogWrite(A_IB, 80);
    analogWrite(B_IA, 0);
    analogWrite(B_IB, 0);
       Serial.println("left");

  } else { // 두 센서 모두 감지되지 않음
    analogWrite(A_IA, 0);
    analogWrite(A_IB, 0);
    analogWrite(B_IA, 0);
    analogWrite(B_IB, 0);
    Serial.println("stop");
  }
}