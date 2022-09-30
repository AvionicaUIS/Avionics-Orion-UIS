#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <MS5611.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "mySD.h"
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


#define ss 5      // Definiendo pines para LoRa
#define rst 14
#define dio0 2
#define cssd 4    // Definiendo pines para SD
#define BUZZER 26 // Definiendo pines para dispositivos alternos
#define LED 25

//------- MULTITASKING
TaskHandle_t sendTaskC0;


MS5611 MS5611(0x77);                              // LoRa I2C Address
File myFile;                                      // SD file name
#define BNO055_SAMPLERATE_DELAY_MS (100)          // IMU BNO055 - tasa de muestreo
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);  // IMU BNO055 - I2C address

int counter = 0;          // Algunas variables necesarias
float presionActual = 0;
float presion0 = 0;
String trama;
String BEG = "{";
String END = "}";
String SEP = ",";
float Temperatura = 0;
float Presion = 0;
float Altura = 0;
float x_acc = 10;
float y_acc = 12;
float z_acc = 10;
float x_gyr = 10;
float y_gyr = 0;
float z_gyr = 0;

bool to_send = false;
String trama_to_send;


void setup() // Búcle setup
{
// Inicializando comunicacion serial, sensores y...
Serial.begin(115200);
SPI.begin();
MS5611.begin();
pinMode(cssd, OUTPUT); 
while(!Serial);
bno.setExtCrystalUse(true);                       // IMU BNO055 - Configuración necesaria

                    //CONFIGURACIÓN NECESARIA PARA USAR 2 NÚCLEOS
//--------------------------------------------------------------------------------------------------------------
//xTaskCreatePinnedToCore(readTaskC1, "readSensors", 10000, NULL, 1, &readTaskC1,  1);   //Núcleo 1
//  delay(500); 

xTaskCreatePinnedToCore(sendInfo, "sendTrama", 10000, &sendTaskC0, 0, &sendTaskC0,  0);     //Núcleo 0
    delay(500); 
//--------------------------------------------------------------------------------------------------------------
  if(!bno.begin())      // Validación status IMU
  {
    Serial.print("BNO055 NO DETECTADO - revisa conexión o dirección I2C.");
    while(1);
  }
     Serial.println("BNO055 DETECTADO -> Estado de la calibración: 0=uncalibrated, 3=fully calibrated");
//--------------------------------------------------------------------------------------------------------------
  LoRa.setPins(ss, rst, dio0);   // Validación status LoRa
  if(!LoRa.begin(433E6))
  {
    Serial.println("LORA NO DETECTADO - revisa conexión o dirección I2C.");
    while (1);
  }
    Serial.println("LORA DETECTADO");
//--------------------------------------------------------------------------------------------------------------
  if (SD.begin(cssd))  // Validación status SD
  {
    Serial.println("TARJETA SD DETECTADA - Lista para usar");}
    else{
    Serial.println("TARJETA SD NO DETECTADA - Inserte tarjeta o revise conexión.");
    return;
  }
//--------------------------------------------------------------------------------------------------------------
  if (MS5611.begin() == true)     // Validación status MS5611
  {
    Serial.println("MS5611 DETECTADO - Listo para usar.");}
    else{
    Serial.println("MS5611 NO DETECTADO - Revise conexión.");
    while (1){
  }}
//--------------------------------------------------------------------------------------------------------------
    for (int cal_int = 0; cal_int < 270 ; cal_int ++)  // Calculando la presión ambiente "pref"
    {          
      delay(10);
      presionActual = MS5611.getPressure();
      presion0 += presionActual;
    //Serial.println(presionActual);
    }
    presion0 = presion0 / 270;
//--------------------------------------------------------------------------------------------------------------
}   // Fin Void Setup




void loop() // Loop principal, nucleo 1.
{
  // Sensor MS5611
  MS5611.read();
  Temperatura = MS5611.getTemperature();
  Presion = MS5611.getPressure();
  Altura = (44330.0f * (1.0f - pow((Presion/presion0), 0.1902949f)));

  trama += BEG + String(Temperatura) + SEP + String(Presion) + SEP + String(Altura) + SEP;
  
  // Sensor IMU
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  x_acc = accel.x();
  y_acc = accel.y();
  z_acc = accel.z();
  
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  x_gyr = gyro.x();
  y_gyr = gyro.y();
  z_gyr = gyro.z();
  
  trama += String(x_acc) + SEP + String(y_acc) + SEP + String(z_acc) + SEP + String(x_gyr) + SEP + String(y_gyr) + SEP + String(z_gyr);
  trama_to_send = trama;
  to_send = true;

  
  
  // ECUACIONES DE APOGEO Y LÓGICA DE CONTROL DE RECUPERACIÓN
  
  // Lo que se visualizará en el monitor Serial
 Serial.println(trama);
   trama = "";
 // Serial.println("Sending packet: " + String(counter) + " - Temperatura: " + String(Temperatura) + " - Presion: " + String(Presion) + " - Altura: " + String(Altura) + " - X_acc: " + String(x_acc) + " - Y_acc: " + String(y_acc) + " - Z_acc: " + String(z_acc) + " - X_gyr: " + String(x_gyr) + " - Y_gyr: " + String(y_gyr) + " Z_gyr: " + String(z_gyr));
//  Serial.print(counter);
//  Serial.print(" - Temperatura: ");
//  Serial.print(Temperatura);
//  Serial.print(", Presion: ");
//  Serial.print(Presion);
//  Serial.print(", Altura: ");
//  Serial.print(Altura);
//  Serial.print(", X_acc: ");
//  Serial.print(euler.x());
//  Serial.print(", Y_acc: ");
//  Serial.print(y_acc);
//  Serial.print(", Z_acc: ");
//  Serial.print(z_acc);
//  Serial.println();

 

// //SD
// myFile = SD.open("PRUEBA.txt", FILE_WRITE);
//  if (myFile) {
//    //Serial.print("Escribiendo en SD...");
//    myFile.print("Sending packet: "); 
//    myFile.print(counter);
//    myFile.print(" - Temperatura: "); 
//    myFile.print(Temperatura);
//    myFile.print(" - Presion: ");    
//    myFile.print(Presion);
//    myFile.print(" - Altura: ");
//    myFile.print(Altura);
//    myFile.print(", X_acc: ");
//    myFile.print(x_acc);
//    myFile.print(", Y_acc: ");
//    myFile.print(y_acc);
//    myFile.print(", Z_acc: ");
//    myFile.print(z_acc);
//    myFile.close(); // close the file
//  }
//  else {
//    Serial.println("Error de escritura en SD.");
//  }

  counter++;

  

 
  delay(10);
}

void sendInfo(void *paremeter) // Loop secundario, nucleo 0.
{
  for(;;){
    if(to_send){
       LoRa.beginPacket();
       LoRa.print("Sending packet: " + String(counter) + " - Temperatura: " + String(Temperatura) + " - Presion: " + String(Presion) + " - Altura: " + String(Altura) + " - X_acc: " + String(x_acc) + " - Y_acc: " + String(y_acc) + " - Z_acc: " + String(z_acc) + " - X_gyr: " + String(x_gyr) + " - Y_gyr: " + String(y_gyr) + " Z_gyr: " + String(z_gyr));
//  LoRa.print("Sending packet: ");
//  LoRa.print(counter);
//  LoRa.print(" - Temperatura: ");
//  LoRa.print(Temperatura);
//  LoRa.print(", Presion: ");
//  LoRa.print(Presion);
//  LoRa.print(", Altura: ");
//  LoRa.print(Altura);
//  LoRa.print(", X_acc: ");
//  LoRa.print(euler.x());
//  LoRa.print(", Y_acc: ");
//  LoRa.print(y_acc);
//  LoRa.print(", Z_acc: ");
//  LoRa.print(z_acc);
  LoRa.println();
  LoRa.endPacket();

  to_send = false;
  trama_to_send = "";
    }
    }
vTaskDelay(10);
}
