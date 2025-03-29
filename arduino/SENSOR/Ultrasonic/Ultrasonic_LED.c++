// 초음파 센서 핀 설정
const int trigPin = 11;
const int echoPin = 12;

// RGB LED 핀 설정
const int redPin = 5;
const int greenPin = 6;
const int bluePin = 7;

// 거리 측정 함수
long getDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2; // 거리 계산 (cm)
    return distance;
}

// RGB LED 색상 변경 함수
void setColor(int red, int green, int blue) {
    analogWrite(redPin, red);
    analogWrite(greenPin, green);
    analogWrite(bluePin, blue);
}

void setup() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
  
    Serial.begin(9600); // 시리얼 모니터 출력
}

void loop() {
    long distance = getDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance <= 5) {
        setColor(255, 0, 0);  // 빨강
    } else if (distance <= 10) {
        setColor(255, 255, 0); // 노랑
    } else if (distance <= 15) {
        setColor(0, 0, 255);  // 파랑
    } else {
        setColor(0, 0, 0);  // LED 끄기
    }

    delay(500); // 0.5초 대기
}
