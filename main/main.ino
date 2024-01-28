#include <SoftwareSerial.h>
#include <math.h>
#include "TinyGPS.h"
#include <OneWire.h>
#include <DallasTemperature.h>
TinyGPS gps;

//##################################################################
/////////////////////////// CONSTANTS //////////////////////////////
//##################################################################

// Initialise pins used for bluetooth, LEDs, GPS, start button, water sensor, and sensors

// Initialise bluetooth pins
#define bleRxPin 2
#define bleTxPin 3

// Initialise LED pins
#define greenLEDPin 4
#define blueLEDPin 5 
#define redLEDPin 6

// Initialise GPS pins
#define gpsEnPin 7
#define gpsRxPin 8
#define gpsTxPin 9

// Initialise start button pin
#define startBtnPin 10

// Initialise depth sensor
#define WATER_SENSOR 11
#define ONE_WIRE_BUS 12

// Initialise conductivity sensor pins
#define condPin1 A0
#define condTempPin1 A1
#define condTempPin2 A2
#define condPin1 A3

// Initialise Turbidity sensor pins
#define turbPin A4

int ledStartTime = 0;

int bluetoothEnabled = 0;

// Set the bluetooth pins to be used for serial communications
SoftwareSerial bleSerial(bleTxPin, bleRxPin);

// Set the GPS pins to be used for serial communications
SoftwareSerial gpsSerial(gpsTxPin, gpsRxPin);

void gpsInfo(TinyGPS &gps);
byte gpsData;

// Initialise the one wire bus for the temperature sensor
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature tempSensor(&oneWire);

//##################################################################
/////////////////////////// SETUP //////////////////////////////////
//##################################################################

void setup() {
  //initialise red, green & blue LED pins as outputs
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  digitalWrite(greenLEDPin, LOW);
  digitalWrite(redLEDPin, LOW);
  digitalWrite(blueLEDPin, LOW);

  // Initialise pins used for conductivity sensor
  pinMode(condPin1, INPUT);
  pinMode(condTempPin1, INPUT);
  pinMode(condTempPin2, INPUT);
  pinMode(condPin1, INPUT);

  // Initialise pins used for turbidity sensor and set as input
  pinMode(turbPin, INPUT);

  // Initialise pin used for start button and set as input
  pinMode(startBtnPin, INPUT);

  // Initialise pin used for depth sensor and set as input
  pinMode(WATER_SENSOR, INPUT);

  // Set the baud rate for serial communications
  Serial.begin (9600);
  tempSensor.begin();
  gpsSerial.begin(9600);
  bleSerial.begin(9600);
};

//##################################################################
/////////////////////////// LOOP ///////////////////////////////////
//##################################################################

void loop() {
  // Read the start button state
  int startBtnState = digitalRead(startBtnPin);

  // Check to see if the app has successfully connected to the Arduino and is
  // ready to receive readings from the sensors
  bluetoothEnabled = receiveBluetoothMessage();

  // Check if the start button has been pressed either on the device or from the
  // app. If it has, then take a reading from each sensor and transmit the
  // readings to the mobile app.
  if (bluetoothEnabled || startBtnState == HIGH) {
    // Check if the depth sensor is in water. If it is, then take a reading from
    // each sensor and transmit the readings to the mobile app. If it is not,
    // then do nothing.
    if (inWater()) {
      // Log to the serial monitor that a reading is being taken
      Serial.println("takeReading");

      // Take a reading from each sensor and transmit the readings to the mobile
      // app
      takeReading();

      // Reset the bluetoothEnabled and startBtnState variables so that the
      // Arduino does not keep taking readings
      bluetoothEnabled = 0;
      startBtnState = 0;

      // We want the LED to stay on for 5 seconds after a reading has been taken
      // so we need to keep track of when the LED was turned on. We do this by
      // setting the ledStartTime variable to the current time in milliseconds.
      ledStartTime = millis();
    } else {
      // Log to the serial monitor that the depth sensor is not in water
      Serial.println("Not in water");
    }
  }

  // Turn the LED off after 5 seconds
  if (millis() > (ledStartTime + 5000)) {
    setRgbLedColor(0, 0, 0);
  }
   
}

//##################################################################
/////////////////////////// OUTPUT /////////////////////////////////
//##################################################################

/**
 * @brief Receives a Bluetooth message from the app and checks if it is a
 * specific command.
 * 
 * This function listens for a Bluetooth message from the mobile app and checks 
 * if the received message is "takeReading". If the message is "takeReading",
 * then we know that the app has successfully connected to the Arduino and is
 * ready to receive readings from the sensors.
 * 
 * @return int - The value of 'enabled' indicating if the command was received.
 */
int receiveBluetoothMessage() {
  int enabled = 0;
  int replaced = bleSerial.listen();

  if (replaced) {
    Serial.println("BLE Erased GPS Buffer");
  }

  if (bleSerial.available()) {
    Serial.println("AVAILABLE!");
    String message = bleSerial.readString();
    if (message == "takeReading") {
      enabled = 1;
    }
  }
  return enabled;
}

/**
 * @brief Transmits a Bluetooth message to the app.
 * 
 * @param start_idx 
 * @param numOfReadings 
 * @param result 
 * @param latitude 
 * @param longitude 
 * @param date 
 * @param time 
 * @param chl 
 * @param cond 
 * @param flu 
 * @param nit 
 * @param pH 
 * @param turb 
 * @param t 
 */
void transmitBluetoothMessage(
  int start_idx,
  int numOfReadings,
  int result,
  float latitude,
  float longitude,
  unsigned long date,
  unsigned long time,
  float *chl,
  float *cond,
  float *flu,
  float *nit,
  float *pH,
  float *turb,
  unsigned long *t
) {

  bleSerial.listen();

  // The full bluetooth transmission message is structured like a CSV file. This
  // function essentially sends each row of the CSV file one by one. However,
  // the bluetooth module can only send a certain amount of characters at a time
  // (23 bytes I think) so each row is split into smaller packages. The receiver
  // (the mobile app) collects each package but does not know how many packages
  // to expect. Therefore, each row ends with a '?' character. The receiver will
  // keep collecting packages until it receives a sees a '?' character at the
  // end of a package. Once it sees the '?' character, it will send a message
  // back to the Arduino saying "next". The Arduino will then send the next
  // row. This process repeats until the last row, in which the row ends with a
  // '!' character. Once the receiver sees the '!' character, it knows that it
  // has received the last row. The Arduino will then stop sending rows.

  // Loops through each value in the arrays and sends them to the mobile app
  for(int i = start_idx; i < numOfReadings; i++) {
    // Creates a transmission message containing a value from each array in the
    // format of a CSV file.
    String subMessage = String(t[i]) + "," + String(chl[i]) + "," + String(cond[i]) + "," + String(flu[i]) + "," + String(nit[i]) + "," + String(pH[i]) + "," + String(turb[i]) + "?";
    // Prints the transmission message to the serial monitor for debugging
    Serial.println(subMessage);

    // Check if bluetooth is enabled and if so, print the transmission message
    // to the bluetooth serial port
    if (bluetoothEnabled) {
      bleSerial.println(subMessage);

      // Wait for the mobile app to send a message back saying that it has
      // received the transmission message
      int transmitted = 0;
      while (!transmitted) {

        // Check if the mobile app has sent a message
        if (bleSerial.available()) {
          
          // Read the message and check if it is "next". If it is, then the
          // mobile app has received the transmission message and is ready for
          // the next one.
          String receivedMessage = bleSerial.readString();
          if (receivedMessage == "next") {
            transmitted = 1;
          }
        }
      }
    }

  }

  // Creates a transmission message containing the latitude, longitude, date,
  // time, and result in the format of a CSV file. This is the last row of the
  // CSV file so it ends with a '!' character.
  String subMessage = String(latitude) + "," + String(longitude) + "," + String(date) + "," + String(time) + "," + String(result) + "!";
  // Prints the transmission message to the serial monitor for debugging
  Serial.println(subMessage);

  // Check if bluetooth is enabled and if so, print the transmission message
  // to the bluetooth serial port
  if (bluetoothEnabled) {
    bleSerial.println(subMessage);
  }
}

/**
 * @brief Gets the latitude, longitude, date, and time from the GPS.
 * 
 * @param gps The TinyGPS object used to decode the GPS data.
 * @param latitude The address of the variable to store the latitude in.
 * @param longitude The address of the variable to store the longitude in.
 * @param date The address of the variable to store the date in.
 * @param time The address of the variable to store the time in.
 */
void gpsInfo(TinyGPS &gps, float *latitude, float *longitude, unsigned long *date, unsigned long *time) {
  // Make arduino listen to GPS
  gpsSerial.listen();
  Serial.println("Listening to GPS");

  // Check if Arduino is listening to GPS
  int gpsListening = gpsSerial.isListening();

  // Start timer which is used to timeout the GPS. Essentially, the GPS will
  // keep trying is get a signal forever, but we only want to wait a certain
  // amount of time before giving up. This timer is used to timeout the GPS
  // after a certain amount of time.
  int gpsStartTime = millis();

  // Loop until there is data in the serial buffer
  while(gpsListening) {
    // Check if there is data in the serial buffer
    if (gpsSerial.available()) {

      // If there is data, read it and encode it
      gpsData = gpsSerial.read();
      if (gps.encode(gpsData)) {

        // Once data is encoded, you can stop looping
        gpsListening = 0;

        // Print a message to the serial monitor for debugging
        Serial.println("GPS Data Received");

        // Get the latitude, longitude, date, and time from the GPS
        gps.f_get_position(latitude, longitude);
        gps.get_datetime(date, time);
        *latitude = float(round((*latitude) * 100.0)) / 100.0;
        *longitude = float(round((*longitude) * 100.0)) / 100.0;
      }
    }

    // Check if the GPS has timed out. If it has, then set the latitude,
    // longitude, date, and time to 0.
    if (millis() > (gpsStartTime + 10000)) {
      Serial.println("GPS Timeout");
      gpsListening = 0;
      *latitude = 0.0f;
      *longitude = 0.0f;
      *date = 0.0f;
      *time = 0.0f;
    }
  }
}

/**
 * @brief Changes the colour of the RGB LED depending on if the water is safe
 * or not.
 * 
 * @param isSafe A boolean value indicating if the water is safe or not.
 */
void changeLEDColor(int isSafe) {
  // Set the RGB LED to be red if the water is unsafe and green if the water is
  // safe. Log the result to the serial monitor for debugging.
  if (isSafe == 1) {
    Serial.println("Safe");
    setRgbLedColor(0, 1, 0);
  } else {
    Serial.println("Unsafe");
    setRgbLedColor(1, 0, 0);
  }
}

/**
 * @brief Set the Rgb Led Color object
 * 
 * @param r integer value for red LED (0, 1)
 * @param g integer value for green LED (0, 1)
 * @param b integer value for blue LED (0, 1)
 */
void setRgbLedColor(int r, int g, int b) {
  digitalWrite(redLEDPin, r);
  digitalWrite(greenLEDPin, g);
  digitalWrite(blueLEDPin, b);
}

//##################################################################
/////////////////////////// MAIN ///////////////////////////////////
//##################################################################

/**
 * @brief Takes a reading from each sensor and transmits the readings to the
 * mobile app.
 * 
 */
void takeReading() {
  // Define the number of readings to take
  int numOfReadings = 20;

  // Initialise arrays to store the readings for each sensor
  float *chl = (float*)calloc(numOfReadings, sizeof(float));
  float *cond = (float*)calloc(numOfReadings, sizeof(float));
  float *flu = (float*)calloc(numOfReadings, sizeof(float));
  float *nit = (float*)calloc(numOfReadings, sizeof(float));
  float *pH = (float*)calloc(numOfReadings, sizeof(float));
  float *turb = (float*)calloc(numOfReadings, sizeof(float));
  // Initialise array to store the time of each reading
  unsigned long *t = (unsigned long*)calloc(numOfReadings, sizeof(unsigned long));

  // Get the latitude, longitude, date, and time from the GPS
  float latitude, longitude;
  unsigned long date, time;
  gpsInfo(gps, &latitude, &longitude, &date, &time);

  // Define the time interval between each reading
  int timeInterval = 100;
  unsigned long prevTime = millis();

  // Take a reading every timeInterval milliseconds until numOfReadings
  // readings have been taken.
  int index = 0;
  while(index < numOfReadings) {
    unsigned long currentTime = millis();
    if (currentTime > (prevTime + timeInterval)) {
      prevTime += timeInterval;
      chl[index] = readChl();
      cond[index] = readCond();
      flu[index] = readFlu();
      nit[index] = readNit();
      pH[index] = readpH();
      turb[index] = readTurb(numOfReadings);
      t[index] = timeInterval * index;
      index++;
    }
  }

  // Log to the serial monitor that all readings have been taken
  Serial.println("Done Reading Sensors");

  // Find the index of the first stable reading
  // For example, if you leavethe temperature sensor in hot water for a while,
  // then transfer it to ice cold water, the temperature will change rapidly
  // until it reaches the temperature of the water. When we calculate if the
  // water is safe or not, we only want to use the stable readings. Therefore,
  // we need to find the index of the first stable reading.
  int stableIndex = 0;
  
  // Check if the water is safe or not
  int safe = isSafe(stableIndex, numOfReadings, chl, cond, flu, nit, pH, turb);

  // Change the colour of the RGB LED depending on if the water is safe or not
  changeLEDColor(safe);

  // Transmit the readings to the mobile app
  transmitBluetoothMessage(stableIndex, numOfReadings, safe, latitude, longitude, date, time, chl, cond, flu, nit, pH, turb, t);

  // Free the memory used by the arrays
  free(chl);
  free(cond);
  free(flu);
  free(nit);
  free(pH);
  free(turb);
  free(t);
}

/**
 * @brief Finds the first index where each sensor reading is stable.
 * 
 * @param numOfReadings Number of readings
 * @param chl Reading from the Chloride sensor
 * @param cond Readings from the Conductivity sensor
 * @param flu Readings from the Fluoride sensor
 * @param nit Readings from the Nitrate sensor
 * @param pH Readings from the pH sensor
 * @param turb Readings from the Turbidity sensor
 * 
 * @return int the index of the first stable reading
 */
int findCommonStableIndex(int numOfReadings, float *chl, float *cond, float *flu, float *nit, float *pH, float *turb) {
  // Find the index of the first stable reading for each sensor
  int chl_idx = findStableIndex(chl, numOfReadings, 5);
  int cond_idx = findStableIndex(cond, numOfReadings, 147);
  int flu_idx = findStableIndex(flu, numOfReadings, 0.5);
  int nit_idx = findStableIndex(nit, numOfReadings, 5);
  int pH_idx = findStableIndex(pH, numOfReadings, 0.5);
  int turb_idx = findStableIndex(turb, numOfReadings, 0.5);

  // Find the maximum index of the first stable reading for each sensor and that
  // will be the index of the first stable reading for all sensors.
  int start_idx = max(max(pH_idx, turb_idx), max(max(chl_idx, cond_idx), max(flu_idx, nit_idx)));

  // Return the index of the first stable reading
  return start_idx;
}

/**
 * @brief Find the index of the first stable reading.
 * 
 * This function finds the index of the first stable reading. It does this by
 * comparing two consecutive readings and if the difference between them is
 * greater than the unstable value, then it is considered unstable. If the
 * difference between two consecutive readings is less than the unstable value,
 * then it is considered stable. This function loops through each reading until
 * it finds the first stable reading.
 * 
 * @param resultsArray The array of readings to find the first stable reading
 * @param numOfReadings The number of readings in the array
 * @param unstable The value used to determine if a reading is stable or not
 * 
 * @return int The index of the first stable reading
 */
int findStableIndex(float *resultsArray, int numOfReadings, float unstable) {
  int start_idx = 0;
  for(int i = 1; i < numOfReadings; i++) {

    // Initialises two consecutive readings so they can be compared
    float prev = resultsArray[i - 1];
    float curr = resultsArray[i];

    // Checks if the difference between the previous and current value is larger
    // than the unstable value. If it is, then the current value is considered
    // unstable and the start index is set to the current index. If it is not,
    // then the current value is considered stable and the loop is broken.
    float diff = curr - prev;
    if (abs(diff) > unstable){
      start_idx = i;
    } else {
      break;
    }
  }

  // Returns the index of the first stable reading
  return start_idx;
}

/**
 * @brief Check if the water is safe or not.
 * 
 * @param start_idx The index of the first stable reading
 * @param numOfReadings Number of readings
 * @param chl Reading from the Chloride sensor
 * @param cond Readings from the Conductivity sensor
 * @param flu Readings from the Fluoride sensor
 * @param nit Readings from the Nitrate sensor
 * @param pH Readings from the pH sensor
 * @param turb Readings from the Turbidity sensor
 * 
 * @return int The value indicating if the water is safe or not
 */
int isSafe(int start_idx, int numOfReadings, float *chl, float *cond, float *flu, float *nit, float *pH, float *turb) {
  // Define the minimum and maximum values for each sensor which are used to
  // determine if the water is safe or not. If the mean of the readings for each
  // sensor is between the minimum and maximum values, then the water is safe.
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
  float maxTurb = 5;

  // Calculate the mean of the readings for each sensor
  float meanChl = getMean(chl, start_idx, numOfReadings);
  float meanCond = getMean(cond, start_idx, numOfReadings);
  float meanFlu = getMean(flu, start_idx, numOfReadings);
  float meanNit = getMean(nit, start_idx, numOfReadings);
  float meanpH = getMean(pH, start_idx, numOfReadings);
  float meanTurb = getMean(turb, start_idx, numOfReadings);

  // Check if the mean of the readings for each sensor is between the minimum
  // and maximum values. If it is, then the water is safe. If it is not, then
  // the water is unsafe.
  if (
    (minChl <= meanChl) && (meanChl <= maxChl) &&
    (minCond <= meanCond) && (meanCond <= maxCond) &&
    (minFlu <= meanFlu) && (meanFlu <= maxFlu) &&
    (minNit <= meanNit) && (meanNit <= maxNit) &&
    (minpH <= meanpH) && (meanpH <= maxpH) &&
    (minTurb <= meanTurb) && (meanTurb <= maxTurb)
  ) {
    // If the water is safe, then return 1
    return 1;
  } else {
    // If the water is unsafe, then return 0
    return 0;
  }
}

/**
 * @brief Calculates the mean of an array of readings.
 * 
 * @param resultsArray The array of readings to calculate the mean of
 * @param startIndex The index to start calculating the mean from
 * @param numOfReadings The number of readings in the array
 * 
 * @return float The mean of the array of readings 
 */
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



//##################################################################
/////////////////////////// SENSORS ////////////////////////////////
//##################################################################

/**
 * @brief Read the Chloride sensor.
 * 
 * @return float The Chloride reading
 */
float readChl() {
  return 0;
}

/**
 * @brief Read the Conductivity sensor.
 * 
 * @return float The Conductivity reading
 */
float readCond() {
  return 0;

  // THIS IS ALL WONG AND NEEDS TO BE FIXED

  // // put your main code here, to run repeatedly:
  // // Set all variables to zero.
  // float Cond_Voltage1 = 0; // A1 - Output voltage 1.
  // float Cond_Voltage2 = 0; // A4 - Output voltage 2.
  // float Cond_Temp1 = 0;    // A2 - Temperature 1.
  // float Cond_Temp2 = 0;    // A3 - Temperature 2.

  // float resistance = 1000; // Resistance (Ohms).
  // float length = 1;        // Length of the solution (m).

  // // Read values on input pins.
  // Cond_Voltage1 += ((float)analogRead(condPin1)/1023)*5;
  // Cond_Temp1 += ((float)analogRead(condTempPin1)/1023)*5;
  // Cond_Temp2 += ((float)analogRead(condTempPin2)/1023)*5;
  // Cond_Voltage2 += ((float)analogRead(condPin1)/1023)*5;

  // // Calculate current and average variables.
  // float Cond_Current1 = (float)(Cond_Voltage1/resistance);
  // float Cond_Current2 = (float)(Cond_Voltage2/resistance);
  // float Avg_Temp = (float)(Cond_Temp1 + Cond_Temp2)/2;
  // float Avg_Voltage = (float)(Cond_Voltage1 + Cond_Voltage2)/2;
  // float Avg_Current = (float)((Cond_Current1 + Cond_Current2)/2);

  // // Calculate conductance and conductivity.
  // float Conductance = Avg_Current/Avg_Voltage;
  // float Conductivity = (Conductance * length) / (length + (Conductance * 0.4) + (Avg_Temp - 25));    // Define length.

  // Serial.println("Conductivity");
  // Serial.println(Conductivity);
  // return Conductivity;
}

/**
 * @brief Read the Fluoride sensor.
 * 
 * @return float The Fluoride reading
 */
float readFlu() {
  return 0;
}

/**
 * @brief Read the Nitrate sensor.
 * 
 * @return float The Nitrate reading
 */
float readNit() {
  return 0;
}

/**
 * @brief Read the pH sensor.
 * 
 * @return float The pH reading
 */
float readpH() {
  return 0;
}

/**
 * @brief Read the Turbidity sensor.
 * 
 * @return float The Turbidity reading
 */
float readTurb(int numberOfReadings) {
  // Set the initial voltage to 0
  float TurbiditySensorVoltage = 0;

  // Define the number of samples to take
  int samples = (int)ceil(10000 / numberOfReadings);

  // Take the average of the samples
  for (int i=0; i<samples; i++) {
    // Conversion of raw sensor reading to a voltage
    TurbiditySensorVoltage += ((float)analogRead(turbPin)/1023)*5;
  }
  // Average of the samples
  TurbiditySensorVoltage = TurbiditySensorVoltage/samples;

  // Used to set the precision of the voltage measurement
  float multiplier = powf(10.0f, 6);
  TurbiditySensorVoltage = roundf(TurbiditySensorVoltage * multiplier) / multiplier;

  // 2.5V is the lower bound of the voltage range of the sensor therefore if the
  // voltage is less than 2.5V, then the NTU is 3000.
  if (TurbiditySensorVoltage < 2.5) {
    return 3000;
  } else {
    // Conversion from voltage to NTU
    float ntu = -1120.4*square(TurbiditySensorVoltage)+ 6257.7*TurbiditySensorVoltage - 5732.9;
    
    // Calibration Equation Based from Turbidity Meter Correlation
    float ActualNTU = ((ntu - 1576)/4.617);
    
    // If the NTU is less than 0, then set it to 0
    if (ActualNTU < 0) {
      ActualNTU = 0;
    }

    // Return the NTU
    return ActualNTU;
  }
}

/**
 * @brief Checks if the depth sensor is in water or not.
 * 
 * @return int 
 */
int inWater() {
  int value = digitalRead(WATER_SENSOR);
  if (value == 1) {
    Serial.print("Not in water - ");
    Serial.println(value);
    return 0;
  } else {
    Serial.print("In water - ");
    Serial.println(value);
    return 1;
  }
}

/**
 * @brief Read the temperature sensor.
 * 
 * @return float The temperature reading
 */
float readTemp() {
  tempSensor.requestTemperatures();
  float temperature = tempSensor.getTempCByIndex(0);
  return temperature;
}