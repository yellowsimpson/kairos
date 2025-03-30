int RED = 11;
int GREEN = 10;
int BLUE = 9;

void setup(){
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
    
}

void loop(){
    analogWrite(RED, 255);    //빨간불 켜짐
    analogWrite(GREEN, 0);    //초록불 꺼짐
    analogWrite(BLUE, 0);     //파란불 꺼짐
    delay(1000);              //1초 대기
    analogWrite(RED, 0);      //빨간불 꺼짐
    analogWrite(GREEN, 255);  //초록불 켜짐
    analogWrite(BLUE, 0);     //파란불 꺼짐
    delay(1000);              //1초 대기
    analogWrite(RED, 0);     //빨간불 꺼짐
    analogWrite(GREEN, 0);   //초록불 꺼짐
    analogWrite(BLUE, 255);  //파란불 켜짐
    delay(1000);
    
}