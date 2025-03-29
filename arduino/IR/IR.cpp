int IR = 2;
int LED = 13;

void setup(){
    pinMode(IR, INPUT);
    pinMode(LED, OUTPUT);
    Serial.begin(115200);
}

void loop(){
    int IRsensor = digitalRead(IR);
    Serial.println(IRsensor);
    if(IRsensor==LOW){
        digitalWrite(LED, HIGH);
    }
    else{
        digitalWrite(LED, LOW);
    }
}