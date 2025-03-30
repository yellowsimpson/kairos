#define ir_R 3
#define ir_L 4

#define motor_A_1A 5
#define motor_A_1B 6
#define motor_B_1A 11
#define motor_B_1B 10

int moving_direction = 0;
int moving_speed = 0;
int delta_R = 0;
int delta_L = 0;
int speed_R = 100;
int speed_L = 100;

void forward(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, speed_R);
  analogWrite(motor_B_1A, speed_L);
  analogWrite(motor_B_1B, 0);
}

void backward(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, speed_R);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, speed_L);
}

void turnLeft(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, speed_R);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, speed_L);
}

void turnRight(int speed_R, int speed_L) {
  analogWrite(motor_A_1A, speed_R);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, speed_L);
  analogWrite(motor_B_1B, 0);
}

void stopAll() {
  analogWrite(motor_A_1A, 0);
  analogWrite(motor_A_1B, 0);
  analogWrite(motor_B_1A, 0);
  analogWrite(motor_B_1B, 0);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(ir_R,INPUT);
  pinMode(ir_L,INPUT);

  pinMode(motor_A_1A, OUTPUT);
  pinMode(motor_A_1B, OUTPUT);
  pinMode(motor_B_1A, OUTPUT);
  pinMode(motor_B_1B, OUTPUT);
}

void loop() {
  int val_R = digitalRead(ir_R);
  int val_L = digitalRead(ir_L);

  if (val_R == LOW && val_L == LOW) {
    forward(80,80);
    Serial.println("F");

  } else if (val_R == HIGH && val_L == HIGH) {
    stopAll();

  } else if (val_L == HIGH) {
    turnLeft(80,80);
    Serial.println("L");
    delay(100);

  }  else if (val_R == HIGH) {
    turnRight(80,80);
    Serial.println("R");
    delay(100);
  }
}
