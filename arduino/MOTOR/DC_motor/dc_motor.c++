const int A_IA = 10;
const int A_IB = 9;

void setup() {
  Serial.begin(115200);
  pinMode(A_IA,OUTPUT);
  pinMode(A_IB,OUTPUT);
}

void loop(){
  if (Serial.available() > 0){
    char c = Serial.read();
    if(c == 'f'){
      analogWrite(A_IA, 128);
      analogWrite(A_IB, 0);
    } 
    else if(c == 'b'){
      analogWrite(A_IA, 0 );
      analogWrite(A_IB, 128 );
    }
    else if(c == 's'){
      analogWrite(A_IA, 0 );
      analogWrite(A_IB, 0 );  
    }
  }
}
