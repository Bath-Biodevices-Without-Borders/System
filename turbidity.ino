// Initialise Turbidity Signal Pin
int Turbidity_Sensor_Pin = A0;

// Variable for Turbidity Sensor Voltage
float TurbiditySensorVoltage;

// Number of samples for an average
int samples = 10000;

// Nephelometric Turbidity Units
float ntu;

// Calibrated NTU Variable
float ActualNTU;


void setup() {
  Serial.begin(9600);
  pinMode(Turbidity_Sensor_Pin, INPUT);
}

void loop() {
  TurbiditySensorVoltage = 0;
  for (int i=0; i<samples; i++) {
    // Conversion of raw sensor reading to a voltage
    TurbiditySensorVoltage += ((float)analogRead(Turbidity_Sensor_Pin)/1023)*5;
  }
  TurbiditySensorVoltage = TurbiditySensorVoltage/samples;
  //Used to set the precision of the voltage measurement
  TurbiditySensorVoltage = round_to_dp(TurbiditySensorVoltage,6);


  // 2.5V is the lower bound of the voltage range
  if (TurbiditySensorVoltage < 2.5) {
    ntu = 3000;
  } else {
    // Conversion from voltage to NTU
    ntu = -1120.4*square(TurbiditySensorVoltage)+ 6257.7*TurbiditySensorVoltage - 5732.9;
    
    // Calibration Equation Based from Turbidity Meter Correlation
    ActualNTU = ((ntu - 1376)/13.822);
    
    // if (ActualNTU < 0) {
    //   ActualNTU = 0;
    // }
  }
  
  // Used to display the NTU Values in the serial monitor
  // Serial.println(ntu);
  // Serial.println("NTU:");
  // delay(1000);

  // Used to display the Calibrated NTU Values in the serial monitor
  Serial.println(ActualNTU);
  Serial.println("Calibrated NTU:");
  delay(1000);
}

float round_to_dp( float in_value, int decimal_place ) {
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}