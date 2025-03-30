#include <Wire.h>              // I2C 라이브러리 추가
#include <Adafruit_GFX.h>      // GFX 라이브러리 추가
#include <Adafruit_SSD1306.h>  // OLED 라이브러리 추가

#define SCREEN_WIDTH 128   // OLED 가로 크기
#define SCREEN_HEIGHT 64   // OLED 세로 크기
#define OLED_RESET    -1   // 리셋 핀 (-1: 사용 안 함)

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
    Serial.begin(115200);

    // OLED 초기화
    if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 초기화 실패!");
        while (1);  // OLED 초기화 실패 시 무한 루프
    }
    Serial.println("SSD1306 OLED 초기화 완료!");

    delay(2000);
    oled.clearDisplay();  // 화면 지우기

    // 텍스트 설정
    oled.setTextSize(3);               // 글자 크기 2배 확대
    oled.setTextColor(SSD1306_WHITE);  // 글자 색상 (흰색)
    oled.setCursor(15, 20);            // 텍스트 시작 위치 (X: 10, Y: 20)
    oled.println("KAIROS");            // 출력할 문자열
    oled.display();                     // OLED 화면에 표시
}

void loop() {
}
