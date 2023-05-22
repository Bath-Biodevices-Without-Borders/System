#define WATER_SENSOR 2

int value = 0;             // Used to read sensor value.

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  pinMode(WATER_SENSOR, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  value = (digitalRead(WATER_SENSOR));
  if (value == 1){
    Serial.print("Not in water - ");
    Serial.println(value);
  }

    if (value == 0){
    Serial.print("In water - ");
    Serial.println(value);
  }
  delay(500);
}
