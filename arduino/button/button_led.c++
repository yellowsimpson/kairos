const int buttonPin = 2;
const int ledPin = 10;

void setup(){
    pinMode(buttonPin, INPUT_PULLUP);
}

void loop(){
    int buttonInput = digitalRead(buttonPin);
    if(buttonInput == LOW){
        for(int t_high = 0; t_high <= 255; t_high++){
            analogWrite(ledPin, t_high);
            delay(4);
        }
    }
    else{
        analogWrite(ledPin, 0);
    }
}

