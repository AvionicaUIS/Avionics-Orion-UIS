#include <Wire.h>
#include "MS5611_baro.h"

MS5611_baro ms5611;

void setup() 
{
  Serial.begin(115200);
  ms5611.begin();
}

void loop()
{
  // Read true temperature & Pressure (without compensation)
  double Temperatura = ms5611.getTemperature();
  long Presion = ms5611.getPressure();
  //double realAltitude = ms5611.getAltitude(realPressure);

  // Read true temperature & Pressure (with compensation)
  //double realTemperature2 = ms5611.readTemperature(true);
  //long realPressure2 = ms5611.readPressure(true);
  //double realAltitude2 = ms5611.getAltitude(realPressure2);

  // Output
  Serial.print(Temperatura);
  Serial.print(":");
  Serial.print(Presion);
  Serial.print(":");
  Serial.println();
}
