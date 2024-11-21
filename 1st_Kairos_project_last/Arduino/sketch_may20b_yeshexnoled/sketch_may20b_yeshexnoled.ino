#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// 세마포어 생성
SemaphoreHandle_t xSerialSemaphore;

// Task 선언
void TaskLEDRead(void *pvParameters);

void setup() {
  Serial.begin(115200); // 시리얼 통신 속도 설정

  // 임계 구역(시리얼 모니터) 접근 제한을 위한 세마포어 생성
  if (xSerialSemaphore == NULL) {
    xSerialSemaphore = xSemaphoreCreateMutex();
    if (xSerialSemaphore != NULL) {
      xSemaphoreGive(xSerialSemaphore); // 세마포어 초기화
    }
  }

  // LED Read를 위한 Task 설정
  xTaskCreate(
    TaskLEDRead,
    "LEDRead", // 사람을 위한 이름
    128, // 스택 크기
    NULL,
    1, // 우선 순위
    NULL
  );

  // 스케줄러 자동 시작
}

void loop() {
  // 아무것도 하지 않음
}

// LED Read Task
void TaskLEDRead(void *pvParameters) {
  (void) pvParameters;

  for (;;) {
    if (xSemaphoreTake(xSerialSemaphore, (TickType_t)5) == pdTRUE) {
      if (Serial.available() >= 4) {
        uint8_t receivedValues[4];
        for (int i = 0; i < 4; i++) {
          receivedValues[i] = Serial.read();
        }

        // 받은 값을 처리하고 LED 상태를 제어
        for (int i = 0; i < 4; i++) {
          // 받은 값에 따라 LED 상태 결정 (0 또는 1을 그대로 사용)
          digitalWrite(13 - i, receivedValues[i] == '1' ? HIGH : LOW);
        }

        // LED 상태를 시리얼로 전송
        for (int i = 0; i < 4; i++) {
          Serial.write(receivedValues[i]);
        }
      }
      xSemaphoreGive(xSerialSemaphore);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
  }
}
