#include "mySD.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define LED 25
#define cssd 4                // Definiendo pines para SD
#define BUZZER 26             // Definiendo pines para dispositivos alternos
String trama;                 // Variables trama de datos
String BEG = "{";
String END = "}";
String SEP = ";";
float x_acc = 0;
float y_acc = 0;
float z_acc = 0;
float x_gyr = 0;
float y_gyr = 0;
float z_gyr = 0;
float x_magneto = 0;
float y_magneto = 0;
float z_magneto = 0;
int counter =0;



volatile unsigned tiempoActual = 0;
volatile unsigned tiempoAnterior = 0;
volatile unsigned intervaloTiempo = 0;
#define BNO055_SAMPLERATE_DELAY_MS (50)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
File myFile;                                      // SD file name


void setup(void)
{

  pinMode(LED, OUTPUT);
  pinMode(cssd, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  Serial.begin(115200);
     //--------------------------------------------------------------------------------------------------------------
  if (SD.begin(cssd))  // Validación status SD
    {
    Serial.println("TARJETA SD DETECTADA - READY.");
     digitalWrite(BUZZER,HIGH);
     digitalWrite(LED,HIGH);
     delay(500);
     digitalWrite(BUZZER,LOW);
     digitalWrite(LED,LOW);
    }
    else{
    Serial.println("TARJETA SD FAILED.");
    return;
  }
//--------------------------------------------------------------------------------------------------------------
  if(!bno.begin())      // Validación status IMU
  {
    Serial.print("BNO055 FAILED");
    while(1);
  }
     Serial.println("BNO055 DETECTADO - READY");
     digitalWrite(BUZZER,HIGH);
     digitalWrite(LED,HIGH);
     delay(500);
     digitalWrite(BUZZER,LOW);
     digitalWrite(LED,LOW);

  bno.setExtCrystalUse(true);


}

void loop(void)
{

  trama += String(counter) + SEP + "\t" + String(tiempoAnterior);
  tiempoActual = millis();
  trama += SEP + "\t" + String(tiempoActual); 
  intervaloTiempo = (double) tiempoActual - tiempoAnterior;
  tiempoAnterior = tiempoActual;
  trama += SEP + "\t" + String(intervaloTiempo);
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> magneto = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  x_acc = accel.x();
  y_acc = accel.y();
  z_acc = accel.z();
  x_gyr = gyro.x();
  y_gyr = gyro.y();
  z_gyr = gyro.z();
  x_magneto = magneto.x();
  y_magneto = magneto.y();
  z_magneto = magneto.z();
  //trama += SEP+"\t"+ String(x_acc) + SEP +"\t" + String(y_acc) + SEP +"\t" + String(z_acc);
  //trama += SEP+"\t"+ String(x_gyr) + SEP +"\t" + String(y_gyr) + SEP +"\t" + String(z_gyr);
  trama += SEP + "\t" + String(x_magneto) + SEP + "\t" + String(y_magneto) + SEP + "\t" + String(z_magneto);
  Serial.println(trama);
  
  myFile = SD.open("NUCLEO0.txt", FILE_WRITE);
  if (myFile) {
  myFile.println(trama); 
  //myFile.println(String(Altura) + "\t" + String(apogee_detection) + "\t" +String(apogeo_imu));
  myFile.close(); // close the file
  }

  
  counter++;
  trama = "";
  Serial.println(trama);
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
