
#include <SoftwareSerial.h>

const int txPin = 2;
const int rxPin = 3;

SoftwareSerial bleSerial(txPin, rxPin);

int counter = -1;

void setup() {
  Serial.begin(9600);
  bleSerial.begin(9600);
}

void loop() {
  String bleData = "_";

  if (bleSerial.available()) {
    bleData = bleSerial.readString();
    Serial.write("RECEIVED:");
    Serial.println(bleData);
  }

  if (bleData == "takeReading") {
    delay(5000);
    Serial.write("TRANSMITTED");
    bleSerial.println("5,4,dateTime,3,37.78825,-122.4324,2,1,1,2,3");
    bleSerial.println("$");
    bleData = '_';
  }
}