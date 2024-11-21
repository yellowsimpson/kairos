#include <DFRobot_DHT11.h>

int sensor_pin = A0;  
int output_value; 

DFRobot_DHT11 DHT;
#define DHT11_PIN 8 // DHT11 센서를 연결할 핀 설정

void setup() 
{  
  Serial.begin(9600);  
  delay(1000);  //1초 대기 (안정화 시간)
}  
  void loop() 
{  
  // 수분 센서 값 읽기
  output_value = analogRead(sensor_pin);
  // 센서 값을 백분율로 변환
  output_value = map(output_value, 550, 0, 0, 100);
  // DHT11 센서로부터 온도와 습도 읽기
  DHT.read(DHT11_PIN);
  float temperature = DHT.temperature;
  float humidity = DHT.humidity;

  // 온도와 습도를 정수로 변환
  int temp_int = int(temperature);
  int humi_int = int(humidity);
  int moist_int = output_value; // 수분 값을 정수로 변환

  // 데이터 프레임 생성 (시작 바이트, 데이터 바이트들, 체크섬)
  byte data[5]; // 배열 크기 5로 설정
  data[0] = 0x02; // 시작 바이트
  data[1] = temp_int;
  data[2] = humi_int;
  data[3] = moist_int; // 수분 값 추가
  data[4] = (data[1] + data[2] + data[3]) & 0xFF; // 체크섬 계산

  // 데이터 프레임 전송
  Serial.write(data, 5); // 5바이트 전송

  // 시리얼 모니터에 전송된 값 출력
  Serial.print("Sent: ");
  Serial.print(temp_int);
  Serial.print("C, ");
  Serial.print(humi_int);
  Serial.print("%, ");
  Serial.print(moist_int);
  Serial.println("%");

  // 1초 대기
  delay(1000);  
}
