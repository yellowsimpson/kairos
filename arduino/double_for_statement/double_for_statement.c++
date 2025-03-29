void setup() {
    Serial.begin(115200);
}

void loop() {
    int i, j;

    for(i = 0; i< 5; i++){
        for(j = 0; j <= i; j++){
            Serial.print('s');
        }
        Serial.println();
    }
    for(;;);
}

