#include <WiFi.h>

#define ssid  "ConnectValue_A401_2G"
#define password   "CVA401!@#$"

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.println(".");
    }
    Serial.print("WiFi connected with IP:");
    Serial.println(WiFi.localIP());
}

void loop() {
    WiFiClient client;
    if(!client.connect(IPAddress(172,30,1,47), 10000)){    
    // if (!client.connect("192.168.1.68", 10000)) {          
        Serial.println("Connection to host failed");
        delay(1000);
        return;
    }
    //Serial.println("client connected sending packet");  
    if(Serial.available()>0){
        char c = Serial.read();
        if(c == 'a'){
            client.print("'a' is sent!");
        }else if(c == 'b'){
            client.print("'b' is sent!");
        }
    } 
    client.print("해킹중!!");
    client.stop();
    delay(100);
}