#include <Arduino_FreeRTOS.h>
#include <semphr.h>

// 세마포어 생성
SemaphoreHandle_t xSerialSemaphore;
//---------- pin setting ------------//
int wire1 = 13;
int wire2 = 12;
int wire3 = 11;
int wire4 = 10;
//////////////////////////////////////////
// Task 선언
void TaskLEDRead(void *pvParameters);
uint8_t receivedValues[4];

void setup() {
  Serial.begin(115200); // 시리얼 통신 속도 설정
  pinMode(wire1, OUTPUT);
  pinMode(wire2, OUTPUT);
  pinMode(wire3, OUTPUT);
  pinMode(wire4, OUTPUT);

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
      if (Serial.available() > 0 ) {
      
        for (int i = 0; i < 4; i++) {
          receivedValues[i] = Serial.read();
        }

        // 받은 값을 처리하고 LED 상태를 제어
        if (receivedValues[0] == 01) {
          digitalWrite(wire1, HIGH);
        } else {
          digitalWrite(wire1, LOW);
        }
        if (receivedValues[1] == 01) {
          digitalWrite(wire2, HIGH);
        } else {
          digitalWrite(wire2, LOW);
        }
        if (receivedValues[2] == 01) {
          digitalWrite(wire3, HIGH);
        } else {
          digitalWrite(wire3, LOW);
        }
        if (receivedValues[3] == 01) {
          digitalWrite(wire4, HIGH);
        } else {
          digitalWrite(wire4, LOW);
        }

        // LED 상태를 시리얼로 전송
        Serial.write(receivedValues, 4);
      }
      xSemaphoreGive(xSerialSemaphore);
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms 대기
  }
}
