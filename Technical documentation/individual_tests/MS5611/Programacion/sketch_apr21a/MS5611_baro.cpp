#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include "MS5611_baro.h"

/*
Se definen actividades exclusivas para el proyecto Avionica - UIS:
- Actividad principal: Calcular altura respecto al nivel inicial y altitud respecto al nivel del mar
alcanzada por el cohete. Para ello se basa en el principio físico de la presión atmosférica. Es necesario
definir actividades propias del sensor basados en el datasheet.
    1- Transmisión: Definir la dirección serial usada por el sensor y crear función que permita acceder 
    a transmisión sin ningún problema.
    2- Tasa de muestreo: El sensor tiene diferentes resoluciones, se deberá poder seleccionar la resolución.
    3- Reset: Funcion reset.
    4- Lectura en bruto: Función que permita leer los datos de 24 bits digitales del sensor y traducirlos
    correctamente y sin perder información.
    5- Memoria: Acceso y escritura sobre la memoria, es necesario para algunas funciones propias del sensor.
*/
    // Método/funcion para establecer el valor de X

    bool MS5611_baro::begin(MS5611_OSR osr)
    {
        Wire.begin();
        reset();
        setOversampling(osr);
        delay(50);
        readPROM();
        return true;
    }

    void MS5611_baro::setOversampling(MS5611_OSR osr)
{
    switch (osr)
    {
  case MS5611_BARO_OSR_256:
      convTime = 1;
      break;
  case MS5611_BARO_OSR_512:
      convTime = 2;
      break;
  case MS5611_BARO_OSR_1024:
      convTime = 3;
      break;
  case MS5611_BARO_OSR_2048:
      convTime = 5;
      break;
  case MS5611_BARO_OSR_4096:
      convTime = 10;
      break;
    }

    uosr = osr;
};

MS5611_OSR MS5611_baro::getOversampling(void)
{
    return (MS5611_OSR)uosr;
}


void MS5611_baro::reset(void)
{
    Wire.beginTransmission(MS5611_BARO_ADDRESS);
  Wire.write(MS5611_BARO_RESET);
    Wire.endTransmission();
}

void MS5611_baro::readPROM(void)
{
    for (uint8_t i = 0; i < 6; i++)
    {
  c[i] = readRegister16(MS5611_BARO_READ_PROM + (i * 2));
    }
}

uint32_t MS5611_baro::RawTemperature(void)
{
    Wire.beginTransmission(MS5611_BARO_ADDRESS);
  Wire.write(MS5611_BARO_D2 + uosr);
    Wire.endTransmission();
    delay(convTime);
    return readRegister24(MS5611_BARO_ADC);
}

uint32_t MS5611_baro::RawPressure(void)
{
    Wire.beginTransmission(MS5611_BARO_ADDRESS);
  Wire.write(MS5611_BARO_D1 + uosr);
    Wire.endTransmission();
    delay(convTime);
    return readRegister24(MS5611_BARO_ADC);
}






int32_t MS5611_baro::getPressure(bool compensation)
{
    uint32_t D1 = RawPressure();
    uint32_t D2 = RawTemperature();
    int32_t dT = D2 - (uint32_t)c[4] * 256;
    int64_t OFF = (int64_t)c[1] * 65536 + (int64_t)c[3] * dT / 128;
    int64_t SENS = (int64_t)c[0] * 32768 + (int64_t)c[2] * dT / 256;

    if (compensation)
    {
  int32_t TEMP = 2000 + ((int64_t) dT * c[5]) / 8388608;
  OFF2 = 0;
  SENS2 = 0;
      if (TEMP < 2000)
      {
          OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
          SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
      }
      if (TEMP < -1500)
      {
          OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
          SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
      }
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;
    }
    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;
    return P;
}


double MS5611_baro::getTemperature(bool compensation)
{
    uint32_t D2 = RawTemperature();
    int32_t dT = D2 - (uint32_t)c[4] * 256;
    int32_t TEMP = 2000 + ((int64_t) dT * c[5]) / 8388608;
    TEMP2 = 0;
        if (compensation)
        {
      if (TEMP < 2000)
      {
          TEMP2 = (dT * dT) / (2 << 30);
      }
        }
    TEMP = TEMP - TEMP2;
    return ((double)TEMP/100);
}



uint16_t MS5611_baro::readRegister16(uint8_t reg)
{
    uint16_t value;
    Wire.beginTransmission(MS5611_BARO_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_BARO_ADDRESS);
    Wire.requestFrom(MS5611_BARO_ADDRESS, 2);
    while(!Wire.available()) {};
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
    Wire.endTransmission();
    value = vha << 8 | vla;
    return value;
}





uint32_t MS5611_baro::readRegister24(uint8_t reg)
{
    uint32_t value;
    Wire.beginTransmission(MS5611_BARO_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_BARO_ADDRESS);
    Wire.requestFrom(MS5611_BARO_ADDRESS, 3);
    while(!Wire.available()) {};
        uint8_t vxa = Wire.read();
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    Wire.endTransmission();
    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;
    return value;
}
