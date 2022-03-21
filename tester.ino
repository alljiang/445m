//This arduino will relay commands from the computer to the drone and back using the HC-12 Module

#include <SoftwareSerial.h>

byte HC12SetPin = 2;
byte HC12TXPin = 7;
byte HC12RXPin = 4;

long receiveBaud = 115200;
long sendBaud = 9600;
long initialHC12Baud = 9600;

SoftwareSerial hc12(HC12TXPin, HC12RXPin); //tx, rx

void setup() {
  pinMode(HC12SetPin, OUTPUT);
  digitalWrite(HC12SetPin, LOW);
  Serial.begin(receiveBaud);
//  hc12.begin(initialHC12Baud);
//  delay(500);
//  Serial.println("Receiver connected");
//  hc12.print(("AT+B"+String(sendBaud)));
  hc12.begin(sendBaud);
  delay(100);
//  hc12.flush();
//  Serial.println(hc12.readString());
  digitalWrite(HC12SetPin, HIGH);
}

void loop() {
  if(Serial.available() > 0) {
    byte toSend[1];
    Serial.readBytes(toSend, 1);
    hc12.write(toSend[0]);
  }
  if(hc12.available()) {
    byte toSend[1];
    hc12.readBytes(toSend, 1);
    Serial.write(toSend[0]);
  }
}
