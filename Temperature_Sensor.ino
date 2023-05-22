#include <OneWire.h>

int DS18S20_Pin = 2;  // Signal pin = Digital pin 2.

// Temperature Chip i/o
OneWire ds(DS18S20_Pin); // Digital Pin 2.

void setup(void) {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop(void) {
  // put your main code here, to run repeatedly:
  float temperature = getTemp();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" degrees Celsius.");

  delay(100); // makes Serial Monitor more readable.
}


float getTemp(){
  // returns the temperature from one DS18S20 in Celsius.

  byte data[12];
  byte addr[8];

  if( !ds.search(addr)) {
    // no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognised");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // Start conversion, with parasitic power on at the end.

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad.
  for (int i = 0; i < 9; i++) { // Need 9 bytes.
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); // Using two's compliment.
  float TemperatureSum = tempRead/16;

  return TemperatureSum;

}
