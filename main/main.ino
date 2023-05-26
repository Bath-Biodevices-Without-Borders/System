#include <SoftwareSerial.h>
#include <math.h>
#include "TinyGPS.h"
TinyGPS gps;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

float chl[10] = {25, 24, 25, 24, 23, 22, 23, 22, 22.5, 22.9};
float cond[10] = {100, 101.5, 103.6, 102.9, 102, 103.5, 102.3, 101.8, 102.6, 102.5};
float flu[10] = {0.9, 0.8, 0.9, 0.88, 0.91, 0.89, 0.83, 0.86, 0.92, 0.86};
float nit[10] = {26, 25.6, 24.9, 25.8, 25.9, 26.3, 25.7, 24.8, 25.5, 25.8};
float pH[10] = {6.9, 7.3, 7.2, 7.5, 7.3, 7.4, 7.5, 7.3, 7.2, 6.9};
float turb[10] = {0.11, 0.12, 0.11, 0.11, 0.10, 0.11, 0.12, 0.11, 0.09, 0.10};

float t[10] = {5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5};

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 


  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

const int redPin = 6;
const int greenPin = 4;
const int bluePin = 5;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

String result;
const int bleTxPin = 3;
const int bleRxPin = 2;

int gpsTxPin = 9;
int gpsRxPin = 8;
int gpsEnPin = 7;

int startBtnPin = 10;

int ledStartTime = 0;

SoftwareSerial bleSerial(bleTxPin, bleRxPin);
SoftwareSerial gpsSerial(gpsTxPin, gpsRxPin);

void gpsInfo(TinyGPS &gps);
byte gpsData;

int counter = -1;

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

void setup() {
  //initialise red, green & blue LED pins as outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, LOW);
  digitalWrite(bluePin, LOW);
  pinMode(startBtnPin, INPUT);
  Serial.begin(9600);
  bleSerial.begin(9600);
  gpsSerial.begin(9600);
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

void loop() {
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int startBtnState = digitalRead(startBtnPin);

  String receiveMessage = "_";
  bleSerial.listen();
  if (bleSerial.available()) {
    Serial.println("AVAILABLE!");
    receiveMessage = bleSerial.readString();
  }

  //if this message recieved is takeReading
  if (receiveMessage == "takeReading" || startBtnState == HIGH) {
    float latitude, longitude;
    unsigned long date, time;
    gpsInfo(gps, &latitude, &longitude, &date, &time);
    latitude = float(round(latitude * 100.0)) / 100.0;
    longitude = float(round(longitude * 100.0)) / 100.0;

    int numOfReadings = 10;
    int start_idx = findStartIndex(numOfReadings);
    int finalNumOfReadings = numOfReadings - start_idx;
    int result = isSafe(start_idx, numOfReadings);
    changeLEDColor(result);
    ledStartTime = millis();
    String message = createMessage(start_idx, numOfReadings, result, latitude, longitude, date, time);
    Serial.println(message);
    startBtnState = 0;

    if (receiveMessage == "takeReading") {
      bleSerial.listen();
      bleSerial.println(message);
      Serial.println(message);
      receiveMessage = '_';
    }
  }

  if (millis() > (ledStartTime + 5000)) {
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, LOW);
  }
   
}

int findStartIndex(int numOfReadings) {
  int chl_idx = findStableIndex(chl, numOfReadings, 5);
  int cond_idx = findStableIndex(cond, numOfReadings, 147);
  int flu_idx = findStableIndex(flu, numOfReadings, 0.5);
  int nit_idx = findStableIndex(nit, numOfReadings, 5);
  int pH_idx = findStableIndex(pH, numOfReadings, 0.5);
  int turb_idx = findStableIndex(turb, numOfReadings, 0.5);

  int start_idx = max(max(pH_idx, turb_idx), max(max(chl_idx, cond_idx), max(flu_idx, nit_idx)));

  return start_idx;
}

void gpsInfo(TinyGPS &gps, float *latitude, float *longitude, unsigned long *date, unsigned long *time) {
  // Make arduino listen to GPS
  gpsSerial.listen();
  // Check if Arduino is listening to GPS
  int gpsListening = gpsSerial.isListening();
  while(gpsListening) {
    // Loop until there is data in the serial buffer
    if (gpsSerial.available()) {
      // Read and encode the data.
      gpsData = gpsSerial.read();
      if (gps.encode(gpsData)) {
        // Once data is encoded, you can stop looping
        gpsListening = 0;
        gps.f_get_position(latitude, longitude);
        gps.get_datetime(date, time);
      }
    }
  }
}

int isSafe(int start_idx, int numOfReadings) {
  float minChl = 0;
  float maxChl = 50;
  float minCond = 30;
  float maxCond = 1500;
  float minFlu = 0;
  float maxFlu = 1.5;
  float minNit = 0;
  float maxNit = 50;
  float minpH = 6.5;
  float maxpH = 8.5;
  float minTurb = 0;
  float maxTurb = 0.15;
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //this is to sum all the valid chloride values
  //this is done by having a seperate indexing value for each sensor so the real "i" doesn't change
  //iChloride is iterated until it reaches the final value of numOfReadings and all results are incremented
  float meanChl = getMean(chl, start_idx, numOfReadings);
  float meanCond = getMean(cond, start_idx, numOfReadings);
  float meanFlu = getMean(flu, start_idx, numOfReadings);
  float meanNit = getMean(nit, start_idx, numOfReadings);
  float meanpH = getMean(pH, start_idx, numOfReadings);
  float meanTurb = getMean(turb, start_idx, numOfReadings);
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //this checks if all the results are within in the spec
  //if they are, the test passes and the LED shines green  
  if (
    (minChl < meanChl) && (meanChl < maxChl) &&
    (minCond<meanCond) && (meanCond<maxCond) &&
    (minFlu<meanFlu) && (meanFlu<maxFlu) &&
    (minNit<meanNit)&& (meanNit<maxNit) &&
    (minpH<meanpH) && (meanpH<maxpH) &&
    (minTurb<meanTurb) && (meanTurb<maxTurb)) {
    return 1;
  } else {
    return 0;
  }  
}

void changeLEDColor(int isSafe) {
  if (isSafe == 1) {
    digitalWrite(greenPin, HIGH);
    digitalWrite(redPin, LOW);
    digitalWrite(bluePin, LOW);
  } else {
    digitalWrite(redPin, HIGH);
    digitalWrite(bluePin, LOW);
    digitalWrite(greenPin, LOW);
  }
}

String createMessage(int start_idx, int numOfReadings, int result, float latitude, float longitude, unsigned long date, unsigned long time) {
  String message = "";
  for(int i = start_idx; i < numOfReadings; i++) {
    message.concat(String(t[i])+ "," + String(chl[i]) + "," + String(cond[i]) + "," + String(flu[i]) + "," + String(nit[i]) + "," 
    + String(pH[i]) + "," + String(turb[i]) + "?");
  }
  message.concat(String(latitude) + "," + String(longitude) + "," + String(date) + "," + String(time) + "," + String(result) + "!");
  return message;
}

void transmitMessage(String message) {
  String receiveMessage = "_";

  //if there's no other comm on the channel, wait for a command/message
  //receiveMessage means the device is available to receive a message
  // Serial.println("Ble Available:");
  // Serial.println(bleSerial.available());
  Serial.println(bleSerial.listen());
  if (bleSerial.isListening()) {
    Serial.println("BLE LISTENING");
    if (bleSerial.available()) {
      Serial.println("AVAILABLE!");
      receiveMessage = bleSerial.readString();
    }

    //if this message recieved is takeReading
    if (receiveMessage == "takeReading") {
      bleSerial.println(message);
      Serial.println("TRANSMITTED");
      Serial.println(message);
      receiveMessage = '_';
    }
  }
}

int findStableIndex(float resultsArray[], int numOfReadings, float unstable) {
  int start_idx = 0;
  for(int i = 1; i < numOfReadings; i++) {
    //initialises two consecutive readings so they can be compared
    float prev = resultsArray[i - 1];
    float curr = resultsArray[i];
    //finds the difference between two consecutive results 
    float diff = curr - prev;
    if (abs(diff) > unstable){
      start_idx = i;
    } else {
      break;
    }
  }
  return start_idx;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

float getMean(float resultsArray[], int startIndex, int numOfReadings) {
  float total = 0;
  for (int i = startIndex; i < numOfReadings; i++) {
    total += resultsArray[i];
  }
  //once all values have been summed they are divided by the total number of readings
  //this calculates the mean value
  float mean = total/(numOfReadings - startIndex);
  return mean;
}
