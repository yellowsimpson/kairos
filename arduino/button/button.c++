const int digitalPin = 2;

void setup(){
    Serial.begin(115200);

    pinMode(digitalPin, INPUT_PULLUP);
}

void loop(){
    int digitalValue = digitalRead(digitalPin);
    Serial.println(digitalValue);
}

