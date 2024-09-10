const int buttonPin = 2;    
const int A_IA = 10;
const int A_IB = 9;

int DCspeed = 0;
int buttonState = HIGH;  
int lastButtonState = HIGH;  

unsigned long lastDebounceTime = 0;  
unsigned long debounceDelay = 50; 

void motor(int data){
  analogWrite(A_IA,0);
  analogWrite(A_IB,data);
}

void setup() {
  pinMode(A_IA,OUTPUT);
  pinMode(A_IB,OUTPUT);
  pinMode(buttonPin,INPUT_PULLUP);
  analogWrite(A_IA,0);
  analogWrite(A_IB,0);
  Serial.begin(9600);
}

void loop(){
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        DCspeed += 120;
        if(DCspeed >=300){
          DCspeed=0;
        }
        Serial.println(DCspeed);
      }
    }
  }
  lastButtonState = reading;
  motor(DCspeed);  
}
