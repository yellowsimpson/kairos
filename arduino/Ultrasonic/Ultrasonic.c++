const int tring_pin = 11;
const int echo_pin = 12;

void setup(){
    pinMode(tring_pin, OUTPUT);
    pinMode(echo_pin, INPUT);

    Serial.begin(115200);
}

void loop(){
    digitalWrite(tring_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(tring_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(tring_pin, LOW);

    long duration = pulseIn(echo_pin, HIGH);
    long distance = (duration/2) / 29.1;

    Serial.print(distance);
    Serial.println("cm");
}   