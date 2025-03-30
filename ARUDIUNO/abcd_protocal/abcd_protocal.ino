//a90b 이후에 나오는 문자(b포함)는 모두 삭제됨
void setup() {
  Serial.begin(9600);
}

void loop() {
  if(Serial.available() > 0) {
    String uga = Serial.readStringUntil('\n');
    int start = uga.indexOf('a');
    int end = uga.indexOf('b');
    //Serial.println(uga);
    String sub_uga = uga.substring(start+1, end);
    int uga_num = sub_uga.toInt();
    Serial.println(sub_uga);
  }
}
