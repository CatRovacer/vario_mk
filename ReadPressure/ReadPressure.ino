/*
  LPS22HB - Read Pressure

  This example reads data from the on-board LPS22HB sensor of the
  Nano 33 BLE Sense and prints the temperature and pressure sensor
  value to the Serial Monitor once a second.

  The circuit:
  - Arduino Nano 33 BLE Sense

  This example code is in the public domain.
*/

#include <Arduino_LPS22HB.h>
float referencePressure = 1013.25;   // Reference pressure for altitude (hPa)

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
}

void loop() {
  // read the sensor value
  float pressure = BARO.readPressure(MILLIBAR);

  // print the sensor value
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.print(" kPa; Altitude = ");
  Serial.println(44330.0 * (1.0 - pow(pressure / referencePressure, 0.19029)));
  
  float temperature = BARO.readTemperature();

  // print the sensor value
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" C");

  // print an empty line
  Serial.println();

  // wait 1 second to print again
  delay(1000);
}
