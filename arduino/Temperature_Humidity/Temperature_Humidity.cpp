#include <DHT.h>

#define DHTPIN 2      // DHT 센서의 데이터 핀을 2번 핀에 연결
#define DHTTYPE DHT11 // DHT 센서 타입 (DHT22를 사용하면 DHT22로 변경)

DHT dht(DHTPIN, DHTTYPE);

void setup() {
    Serial.begin(115200);
    Serial.println("DHT 센서 초기화 중...");
    dht.begin();
}

void loop() {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature(); // 섭씨 온도
    // float temperatureF = dht.readTemperature(true); // 화씨 온도를 원하면 사용

    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("센서에서 데이터를 읽을 수 없습니다!");
        return;
    }

    Serial.print("온도: ");
    Serial.print(temperature);
    Serial.print(" °C, 습도: ");
    Serial.print(humidity);
    Serial.println(" %");

    delay(2000); // 2초마다 측정
}
