#include <SoftwareSerial.h>

// Create a SoftwareSerial object
SoftwareSerial mySerial(10, 11); // TX, RX

void setup() {
  // Start the hardware serial communication
  Serial.begin(9600);
  // Start the software serial communication
  mySerial.begin(9600);
  // Initial message
  Serial.println("Ready to communicate via software serial!");
}

void loop() {
  // Check if data is available on the hardware serial port
  if (Serial.available()) {
    char dataFromPC = Serial.read();
    mySerial.write(dataFromPC); // Send data to the software serial device
  }

  // Check if data is available on the software serial port
  if (mySerial.available()) {
    char dataFromDevice = mySerial.read();
    Serial.write(dataFromDevice); // Send data to the serial monitor
  }
}

