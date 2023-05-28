#define WATER_SENSOR 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin (9600);
  pinMode(WATER_SENSOR, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  inWater();
  delay(500);
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
  return value;
}
