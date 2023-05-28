#include <SoftwareSerial.h>
#include <math.h>
#include "TinyGPS.h"
TinyGPS gps;

#define bleRxPin 2
#define bleTxPin 3

#define greenLEDPin 4
#define blueLEDPin 5 
#define redLEDPin 6

#define gpsEnPin 7
#define gpsRxPin 8
#define gpsTxPin 9

#define startBtnPin 10

#define WATER_SENSOR 11

// Initialise Variables
#define condPin1 A1
#define condTempPin1 A2
#define condTempPin2 A3
#define condPin1 A4

int ledStartTime = 0;

SoftwareSerial bleSerial(bleTxPin, bleRxPin);
SoftwareSerial gpsSerial(gpsTxPin, gpsRxPin);

void gpsInfo(TinyGPS &gps);
byte gpsData;

void setup() {
  //initialise red, green & blue LED pins as outputs
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(blueLEDPin, LOW);

  pinMode(condPin1, INPUT);           // A1 - Output current 1.
  pinMode(condTempPin1, INPUT);      // A2 - Temperature 1.
  pinMode(condTempPin2, INPUT);      // A3 - Temperature 2.
  pinMode(condPin1, INPUT);           // A4 - Output current 2.

  pinMode(startBtnPin, INPUT);

  pinMode(WATER_SENSOR, INPUT);
  // put your setup code here, to run once:
  Serial.begin (9600);
  bleSerial.begin(9600);
  gpsSerial.begin(9600);
};

void loop() {
  int startBtnState = digitalRead(startBtnPin);

  String receiveMessage = receiveBluetoothMessage();

  //if this message recieved is takeReading
  if (receiveMessage == "takeReading" || startBtnState == HIGH) {
    if (inWater()) {
      Serial.println("takeReading");
      String message = takeReading();
      startBtnState = 0;
      ledStartTime = millis();

      if (receiveMessage == "takeReading") {
        transmitBluetoothMessage(message);
        receiveMessage = '_';
      }
    } else {
      Serial.println("Not in water");
    }
  }

  if (millis() > (ledStartTime + 5000)) {
    setRgbLedColor(0, 0, 0);
  }
   
}

int inWater() {
  int value = digitalRead(WATER_SENSOR);
  if (value == 1) {
    Serial.print("Not in water - ");
    Serial.println(value);
  } else {
    Serial.print("In water - ");
    Serial.println(value);
  }
  return 1;
}

String receiveBluetoothMessage() {
  String message = "_";
  int replaced = bleSerial.listen();

  if (replaced) {
    Serial.println("BLE Erased GPS Buffer");
  }

  if (bleSerial.available()) {
    Serial.println("AVAILABLE!");
    message = bleSerial.readString();
  }
  return message;
}

void transmitBluetoothMessage(String message) {
  bleSerial.listen();
  bleSerial.println(message);
  Serial.println(message);
}

String takeReading() {
  float chl[10] = {25, 24, 25, 24, 23, 22, 23, 22, 22.5, 22.9};
  float cond[10] = {100, 101.5, 103.6, 102.9, 102, 103.5, 102.3, 101.8, 102.6, 102.5};
  float flu[10] = {0.9, 0.8, 0.9, 0.88, 0.91, 0.89, 0.83, 0.86, 0.92, 0.86};
  float nit[10] = {26, 25.6, 24.9, 25.8, 25.9, 26.3, 25.7, 24.8, 25.5, 25.8};
  float pH[10] = {6.9, 7.3, 7.2, 7.5, 7.3, 7.4, 7.5, 7.3, 7.2, 6.9};
  float turb[10] = {0.11, 0.12, 0.11, 0.11, 0.10, 0.11, 0.12, 0.11, 0.09, 0.10};
  float t[10] = {5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5};
  int numOfReadings = 10;
  float latitude, longitude;
  unsigned long date, time;

  gpsInfo(gps, &latitude, &longitude, &date, &time);

  int stableIndex = findCommonStableIndex(numOfReadings, chl, cond, flu, nit, pH, turb);
  int safe = isSafe(stableIndex, numOfReadings, chl, cond, flu, nit, pH, turb);
  String message = createMessage(stableIndex, numOfReadings, safe, latitude, longitude, date, time, chl, cond, flu, nit, pH, turb, t);
  changeLEDColor(safe);
  return message;
}


void gpsInfo(TinyGPS &gps, float *latitude, float *longitude, unsigned long *date, unsigned long *time) {
  // Make arduino listen to GPS
  gpsSerial.listen();
  Serial.println("Listening to GPS");
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
        *latitude = float(round((*latitude) * 100.0)) / 100.0;
        *longitude = float(round((*longitude) * 100.0)) / 100.0;
      }
    }
  }
}

int findCommonStableIndex(int numOfReadings, float *chl, float *cond, float *flu, float *nit, float *pH, float *turb) {
  int chl_idx = findStableIndex(chl, numOfReadings, 5);
  int cond_idx = findStableIndex(cond, numOfReadings, 147);
  int flu_idx = findStableIndex(flu, numOfReadings, 0.5);
  int nit_idx = findStableIndex(nit, numOfReadings, 5);
  int pH_idx = findStableIndex(pH, numOfReadings, 0.5);
  int turb_idx = findStableIndex(turb, numOfReadings, 0.5);
  int start_idx = max(max(pH_idx, turb_idx), max(max(chl_idx, cond_idx), max(flu_idx, nit_idx)));
  return start_idx;
}

int findStableIndex(float *resultsArray, int numOfReadings, float unstable) {
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

int isSafe(int start_idx, int numOfReadings, float *chl, float *cond, float *flu, float *nit, float *pH, float *turb) {
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

  //this is to sum all the valid chloride values
  //this is done by having a seperate indexing value for each sensor so the real "i" doesn't change
  //iChloride is iterated until it reaches the final value of numOfReadings and all results are incremented
  float meanChl = getMean(chl, start_idx, numOfReadings);
  float meanCond = getMean(cond, start_idx, numOfReadings);
  float meanFlu = getMean(flu, start_idx, numOfReadings);
  float meanNit = getMean(nit, start_idx, numOfReadings);
  float meanpH = getMean(pH, start_idx, numOfReadings);
  float meanTurb = getMean(turb, start_idx, numOfReadings);

  //this checks if all the results are within in the spec
  //if they are, the test passes and the LED shines green  
  if (
    (minChl < meanChl) && (meanChl < maxChl) &&
    (minCond < meanCond) && (meanCond < maxCond) &&
    (minFlu < meanFlu) && (meanFlu < maxFlu) &&
    (minNit < meanNit)&& (meanNit < maxNit) &&
    (minpH < meanpH) && (meanpH < maxpH) &&
    (minTurb < meanTurb) && (meanTurb < maxTurb)) {
    return 1;
  } else {
    return 0;
  }
}

float getMean(float *resultsArray, int startIndex, int numOfReadings) {
  float total = 0;
  for (int i = startIndex; i < numOfReadings; i++) {
    total += resultsArray[i];
  }
  //once all values have been summed they are divided by the total number of readings
  //this calculates the mean value
  float mean = total/(numOfReadings - startIndex);
  return mean;
}

String createMessage(int start_idx, int numOfReadings, int result, float latitude, float longitude, unsigned long date, unsigned long time, float *chl, float *cond, float *flu, float *nit, float *pH, float *turb, float *t) {
  String message = "";
  for(int i = start_idx; i < numOfReadings; i++) {
    message.concat(String(t[i])+ "," + String(chl[i]) + "," + String(cond[i]) + "," + String(flu[i]) + "," + String(nit[i]) + "," + String(pH[i]) + "," + String(turb[i]) + "?");
  }
  message.concat(String(latitude) + "," + String(longitude) + "," + String(date) + "," + String(time) + "," + String(result) + "!");
  return message;
}

void changeLEDColor(int isSafe) {
  if (isSafe == 1) {
    Serial.println("Safe");
    setRgbLedColor(0, 1, 0);
  } else {
    Serial.println("Unsafe");
    setRgbLedColor(1, 0, 0);
  }
}

void setRgbLedColor(int r, int g, int b) {
  analogWrite(redLEDPin, r);
  analogWrite(greenLEDPin, g);
  analogWrite(blueLEDPin, b);
}

float readConductivity() {
  // put your main code here, to run repeatedly:
  // Set all variables to zero.
  float Cond_Voltage1 = 0; // A1 - Output voltage 1.
  float Cond_Voltage2 = 0; // A4 - Output voltage 2.
  float Cond_Temp1 = 0;    // A2 - Temperature 1.
  float Cond_Temp2 = 0;    // A3 - Temperature 2.

  float resistance = 1000; // Resistance (Ohms).
  float length = 1;        // Length of the solution (m).

  // Read values on input pins.
  Cond_Voltage1 += ((float)analogRead(condPin1)/1023)*5;
  Cond_Temp1 += ((float)analogRead(condTempPin1)/1023)*5;
  Cond_Temp2 += ((float)analogRead(condTempPin2)/1023)*5;
  Cond_Voltage2 += ((float)analogRead(condPin1)/1023)*5;

  // Calculate current and average variables.
  float Cond_Current1 = (float)(Cond_Voltage1/resistance);
  float Cond_Current2 = (float)(Cond_Voltage2/resistance);
  float Avg_Temp = (float)(Cond_Temp1 + Cond_Temp2)/2;
  float Avg_Voltage = (float)(Cond_Voltage1 + Cond_Voltage2)/2;
  float Avg_Current = (float)((Cond_Current1 + Cond_Current2)/2);

  // Calculate conductance and conductivity.
  float Conductance = Avg_Current/Avg_Voltage;
  float Conductivity = (Conductance * length) / (length + (Conductance * 0.4) + (Avg_Temp - 25));    // Define length.

  Serial.println("Conductivity");
  Serial.println(Conductivity);
  return Conductivity;
}