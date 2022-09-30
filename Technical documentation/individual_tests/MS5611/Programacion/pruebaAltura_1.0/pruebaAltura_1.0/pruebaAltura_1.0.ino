//
//    FILE: MS5611_test.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo application
//    DATE: 2014-okt-16
//     URL: https://github.com/RobTillaart/MS5611


#include "MS5611.h"

//  BREAKOUT  MS5611  aka  GY63 - see datasheet
//
//  SPI    I2C
//              +--------+
//  VCC    VCC  | o      |
//  GND    GND  | o      |
//         SCL  | o      |
//  SDI    SDA  | o      |
//  CSO         | o      |
//  SDO         | o L    |   L = led
//          PS  | o    O |   O = opening  PS = protocol select
//              +--------+
//
//  PS to VCC  ==>  I2C  (GY-63 board has internal pull up, so not needed)
//  PS to GND  ==>  SPI
//  CS to VCC  ==>  0x76
//  CS to GND  ==>  0x77

MS5611 MS5611(0x77);

#ifndef LED_BUILTIN
#define LED_BUILTIN    13
#endif

uint32_t start, stop;
double presion0 = 0;
double presionActual = 0;


void setup()
{
  Serial.begin(115200);           // Inicializo la comunicación serial.
  while (!Serial);                // Hasta que no abra el puerto serial no iniciará a ejecutar nada.

  pinMode(LED_BUILTIN, OUTPUT);   // Declaro como salida el led incorporado de la placa, independiente de cual sea (ver lógica en ifndef led_buildin).

  Serial.println();               // Salto de línea.
  Serial.println(__FILE__);       // Imprime la ubicación del archivo.
  Serial.print("MS5611_LIB_VERSION: "); // Imprime mensaje para visualizar la versión de la librería MS5611 (INNECESARIO).
  Serial.println(MS5611_LIB_VERSION);   // Imprime la versión de la librería. (INNECESARIO).

  if (MS5611.begin() == true)     // Condicional, sí la función begin de la librería es verdadera. Tal función es para indicar si detectó o no el sensor (0x77 ó 0x76).
  {
    Serial.println("MS5611 found.");  // Mensaje si fue verdadera la ubicación del sensor.
  }
  else
  {
    Serial.println("MS5611 not found. halt.");  // Si fue falsa, osea no encontró el sensor.
    while (1) 
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(1000);
      digitalWrite(LED_BUILTIN, LOW);
      delay(1000);
    }
  }
  Serial.println();

  MS5611.setOversampling(OSR_HIGH);       // Setear la tasa de muestreo en la más alta resolución.
  delay(9);
  for (int cal_int = 0; cal_int < 512 ; cal_int ++) {
//    delay(10);
    test();
    presionActual = MS5611.getPressure();
//  Serial.println(presionActual);
    presion0 += presionActual;
  }
  presion0 = presion0 / 512;
  //Serial.println(presion0);
  
}



/*
  There are 5 oversampling settings, each corresponding to a different amount of milliseconds
  The higher the oversampling, the more accurate the reading will be, however the longer it will take.
  OSR_ULTRA_HIGH -> 8.22 millis
  OSR_HIGH       -> 4.11 millis
  OSR_STANDARD   -> 2.1 millis
  OSR_LOW        -> 1.1 millis
  OSR_ULTRA_LOW  -> 0.5 millis   Default = backwards compatible
*/
void loop()
{
//  digitalWrite(LED_BUILTIN, HIGH);
//  MS5611.setOversampling(OSR_ULTRA_LOW);
//  test();
//  digitalWrite(LED_BUILTIN, LOW);
//  delay(1000);
//
//  digitalWrite(LED_BUILTIN, HIGH);
//  MS5611.setOversampling(OSR_LOW);
//  test();
//  digitalWrite(LED_BUILTIN, LOW);
//  delay(1000);
//
//  digitalWrite(LED_BUILTIN, HIGH);
//  MS5611.setOversampling(OSR_STANDARD);
//  test();
//  digitalWrite(LED_BUILTIN, LOW);
//  delay(1000);
//
//  digitalWrite(LED_BUILTIN, HIGH);
//  MS5611.setOversampling(OSR_HIGH);
//  test();
//  digitalWrite(LED_BUILTIN, LOW);
//  delay(1000);



//  digitalWrite(LED_BUILTIN, HIGH);
//  MS5611.setOversampling(OSR_ULTRA_HIGH);
  test();
//  digitalWrite(LED_BUILTIN, LOW);
//  delay(200);
  Serial.println();
  presionActual = MS5611.getPressure();
//  float Alt_cm = log((presion0/100) / (presionActual/100)) * 723800.3;
  float Alt_cm = (44330.0f * (1.0f - pow(presionActual / presion0, 0.1902949f)));
//  float Alt_cm = (44330.0*(1-pow(presionActual/presion0,1/5.255)));
//  Serial.print("Presión referencia: ");
//  Serial.print(presion0);
//  Serial.print(", Presion actual: ");
//  Serial.print(presionActual);
//  Serial.print(", Altura [cm]: ");
  Serial.println(Alt_cm);
}


void test()
{
  start = micros();
  int result = MS5611.read();
  stop = micros();
  if (result != MS5611_READ_OK)
  {
    Serial.print("Error in read: ");
    Serial.println(result);
  }
  else
  {
//    Serial.print("T:\t");
//    Serial.print(MS5611.getTemperature(), 2);
//    Serial.print("\tP:\t");
//    Serial.print(MS5611.getPressure(), 2);
//    Serial.print("\tt:\t");
//    Serial.print(stop - start);
//    Serial.println();
  }
}


// -- END OF FILE --
