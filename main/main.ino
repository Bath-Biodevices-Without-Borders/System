// Initialise Variables
#define condPin1 A1
#define condTempPin1 A2
#define condTempPin2 A3
#define condPin1 A4

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(condPin1, INPUT);           // A1 - Output current 1.
  pinMode(condTempPin1, INPUT);      // A2 - Temperature 1.
  pinMode(condTempPin2, INPUT);      // A3 - Temperature 2.
  pinMode(condPin1, INPUT);           // A4 - Output current 2.
}

void loop() {
  float conductivity = readConductivity();
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
