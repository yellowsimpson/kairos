const int BUZZER = 10;
char note [] = "ggaaggeggeed ggaaggegedec";
char beat [] = "111111121111221111112111122";
int note_length = sizeof(note) / sizeof(note[0]) -1;

int tempo = 300;

int freq(char note){
    char note_name[] = {'c', 'd', 'e', 'f', 'g', 'a', 'b', 'c'};
    int note_freq[] = {262, 294, 330, 349, 392, 440, 494, 523};

    for (int i = 0; i < sizeof(note_name) / sizeof(note_name[0]); i++){
        if (note_name[i] == note){
            return note_freq[i];
        }
    }
}

int duration(char beat){
    return beat - '0';
}

void setup() {
    for (int i = 0; i < note_length ++) {
        if(note[i] != ' '){
            tone(BUZZER, freq(note[i]));
        }
        delay(tempo*duration(beat[i]));
        noTone(BUZZER); // 모든 음 재생 후 버저 종료
        delay(100);
    }
}

void loop() {
    // 아무 작업 없음
}
