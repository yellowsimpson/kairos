const int BUZZER = 10;

void setup(){
    for(int cnt = 0; cnt <=2; cnt++){
        tone(BUZZER, 262);
        delay(1000);
        tone(BUZZER, 294);
        delay(1000);
    }
    noTone(BUZZER);
}

void loop(){
    
}