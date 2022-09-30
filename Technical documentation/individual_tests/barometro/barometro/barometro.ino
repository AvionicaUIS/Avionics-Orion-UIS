#include "mySD.h"
#include <SPI.h>
#include <Wire.h>
#include "MS5611.h"

#define LED 25
#define cssd 4                // Definiendo pines para SD
#define BUZZER 26             // Definiendo pines para dispositivos alternos
String trama;                 // Variables trama de datos
String BEG = "{";
String END = "}";
String SEP = ";";
float presionActual = 0;      // Variables barométrico
float presion0 = 0;
float Temperatura = 0;
float Presion = 0;
float Altura = 0;
int counter =0;

volatile unsigned tiempoActual = 0;
volatile unsigned tiempoAnterior = 0;
volatile unsigned intervaloTiempo = 0;

MS5611 MS5611(0x77);
File myFile;                                      // SD file name



void setup()
{
 Serial.begin(115200);
 MS5611.begin();
 SPI.begin();
 
 
 while (!Serial);

  pinMode(LED, OUTPUT);
  pinMode(cssd, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  Serial.println();
  if (MS5611.begin() == true)
  {
    Serial.println("MS5611 found.");
     digitalWrite(BUZZER,HIGH);
     delay(500);
     digitalWrite(BUZZER,LOW);
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
  }
  

  //--------------------------------------------------------------------------------------------------------------
  if (SD.begin(cssd))  // Validación status SD
    {
    Serial.println("TARJETA SD DETECTADA - READY.");
     digitalWrite(BUZZER,HIGH);
     delay(500);
     digitalWrite(BUZZER,LOW);
    }
    else{
    Serial.println("TARJETA SD FAILED.");
    return;
  }

  //--------------------------------------------------------------------------------------------------------------
    for (int cal_int = 0; cal_int < 500 ; cal_int ++)  // Calculando la presión ambiente "pref"
    {
    MS5611.read();          
    delay(10);
    presionActual = MS5611.getPressure();
    presion0 += presionActual;
    //Serial.println(presionActual);
    }
    presion0 = presion0 / 500;
//    Serial.println(presion0);
}

void loop()
{
  trama += String(counter) + SEP + "\t" + String(tiempoAnterior);
  tiempoActual = millis()/1000;
  trama += SEP + "\t" + String(tiempoActual); 
  intervaloTiempo = (double) tiempoActual - tiempoAnterior;
  tiempoAnterior = tiempoActual;
  
  MS5611.read();
  Temperatura = MS5611.getTemperature();
  Presion = MS5611.getPressure();
  Altura = (44330.0 * (1.0 - pow(Presion/presion0, 0.1903)));

  trama += SEP + "\t" + String(intervaloTiempo)+SEP+"\t" + String(Temperatura)+SEP+"\t"+String(Presion)+SEP+"\t"+String(Altura);
  Serial.println(trama);

  myFile = SD.open("NUCLEO0.txt", FILE_WRITE);
  if (myFile) {
  myFile.println(trama); 
  //myFile.println(String(Altura) + "\t" + String(apogee_detection) + "\t" +String(apogeo_imu));
  myFile.close(); // close the file
  }
  


  counter++;
  trama = "";
  delay(10);
  Serial.println(trama);
}


// -- END OF FILE --
