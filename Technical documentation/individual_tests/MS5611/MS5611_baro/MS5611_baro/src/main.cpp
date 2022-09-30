#include "Arduino.h"
#include <Wire.h>
#include "MS5611_baro.h"

#include "SD.h"
#include "SPI.h"

File myFile;
int pinCS = 10;


MS5611_baro ms5611;
double pref = 0;
void setup() 
{
  Serial.begin(115200);
  ms5611.begin();


  pinMode(pinCS, OUTPUT);
    if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
}
// double getReferencePressure()
// {
//   double Pactual, p0, referencePressure;
//     for (int i = 0; i < 1000; i++)
//   {
//     delay(5);
//     Pactual = ms5611.getPressure(true);
//     p0 += Pactual;
//     Serial.println(Pactual);
//   }
//   referencePressure = p0/1000;
//   return referencePressure;
// }
void loop()
{
  // pref = getReferencePressure();
  // Read true temperature & Pressure (without compensation)
  double Temperatura = ms5611.getTemperature(true);
  float Presion = ms5611.getPressure(true);
  float Altura = (44330.0f * (1.0f - pow((Presion/pref), 0.1902949f)));
  //double realAltitude = ms5611.getAltitude(realPressure);

  // Read true temperature & Pressure (with compensation)
  //double realTemperature2 = ms5611.readTemperature(true);
  //long realPressure2 = ms5611.readPressure(true);
  //double realAltitude2 = ms5611.getAltitude(realPressure2);

  // Output
  Serial.print("Temperatura: ");
  Serial.print(Temperatura);
  Serial.print("Presion: ");
  Serial.print(Presion);
  Serial.print("Altura: ");
  Serial.println(Altura);
  Serial.println();
}

