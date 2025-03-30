const int BUZZER = 10;

const int melody[8] = { 
    262, 294, 330, 349, 392, 440, 494, 523 
};

void setup() {
    for (int note = 0; note < 8; note++) { // <= 7 대신 < 8 사용
        tone(BUZZER, melody[note], 250); // 250ms 동안 음 재생
        delay(300);  // 짧은 간격 유지
    }
    noTone(BUZZER); // 모든 음 재생 후 버저 종료
}

void loop() {
    // 아무 작업 없음
}
