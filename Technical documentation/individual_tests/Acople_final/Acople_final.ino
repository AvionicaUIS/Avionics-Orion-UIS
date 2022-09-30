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
bool apogeo_imu = false;
bool apogeo_barometer = false;
float x_acc = 0;
float y_acc = 0;
float z_acc = 0;
float x_gyr = 0;
float y_gyr = 0;
float z_gyr = 0;
float x_magneto = 0;
float y_magneto = 0;
float z_magneto = 0;
double apogee_detection = 0;
double media_cuadratica = 0;

bool to_send = false;
String trama_to_send;


unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;
unsigned long tiempoSegundos = 0;


void setup() // Búcle setup
{
// Inicializando comunicacion serial, sensores y...
Serial.begin(115200);
SPI.begin();
MS5611.begin();
pinMode(cssd, OUTPUT);
pinMode(BUZZER, OUTPUT);
 
while(!Serial);
  tiempo1 = millis();

                    //CONFIGURACIÓN NECESARIA PARA USAR 2 NÚCLEOS
//--------------------------------------------------------------------------------------------------------------
//xTaskCreatePinnedToCore(readTaskC1, "readSensors", 10000, NULL, 1, &readTaskC1,  1);   //Núcleo 1
//  delay(500); 

xTaskCreatePinnedToCore(sendInfo, "sendTrama", 10000, &sendTaskC0, 0, &sendTaskC0,  0);     //Núcleo 0
  delay(500); 
//--------------------------------------------------------------------------------------------------------------
  digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  if(!bno.begin())      // Validación status IMU
  {
    Serial.print("BNO055 FAILED");
    while(1);
  }
     Serial.println("BNO055 DETECTADO - READY");
//--------------------------------------------------------------------------------------------------------------
    digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  LoRa.setPins(ss, rst, dio0);   // Validación status LoRa
  if(!LoRa.begin(433E6))
  {
    Serial.println("LORA FAILED.");
    while (1);
  }
    Serial.println("LORA DETECTADO - READY.");
//--------------------------------------------------------------------------------------------------------------
    digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  if (SD.begin(cssd))  // Validación status SD
  {
    Serial.println("TARJETA SD DETECTADA - READY.");}
    else{
    Serial.println("TARJETA SD FAILED.");
    return;
  }
//--------------------------------------------------------------------------------------------------------------
    digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  if (MS5611.begin() == true)     // Validación status MS5611
  {
    Serial.println("MS5611 DETECTADO - READY.");}
    else{
    Serial.println("MS5611 FAILED.");
    while (1){
  }}
  MS5611.setOversampling(OSR_ULTRA_HIGH);
//--------------------------------------------------------------------------------------------------------------
    for (int cal_int = 0; cal_int < 270 ; cal_int ++)  // Calculando la presión ambiente "pref"
    {
    MS5611.read();          
    delay(10);
    presionActual = MS5611.getPressure();
    presion0 += presionActual;
    Serial.println(presionActual);
    }
    presion0 = presion0 / 270;
    Serial.println(presion0);
//--------------------------------------------------------------------------------------------------------------
bno.setExtCrystalUse(true);                       // IMU BNO055 - Configuración necesaria

}   // Fin Void Setup




void loop() // Loop principal, nucleo 1.
{
  // Contador de tiempo segundos / milisegundos
    tiempo2 = millis();
    if(tiempo2 > (tiempo1+1000)){  //Si ha pasado 1 segundo ejecuta el IF
    tiempo1 = millis(); //Actualiza el tiempo actual
    tiempoSegundos = tiempo1/1000;}
  //--------------------------------------------------------------------------- Sensor MS5611
  MS5611.read();
  Temperatura = MS5611.getTemperature();
  Presion = MS5611.getPressure();
  float Altura = (44330.0 * (1.0 - pow(Presion/presion0, 0.1903)));
  int alturaInicial = 0;//alturaFinal;
  int alturaFinal = 0;//Altura;
  
  
//  apogee_alturas = alturaFinal - alturaInicial;
  //if(apogee_alturas < 1){
  //  apogeo_barometer == true;
   // }    

  
  // Sensor IMU
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  x_acc = accel.x();
  y_acc = accel.y();
  z_acc = accel.z();
  media_cuadratica = sqrt(pow(x_acc,2)+pow(y_acc,2)+pow(z_acc,2));
//  if (media_cuadratica <1.30){
//    apogeo_imu == true;
//    digitalWrite(BUZZER,HIGH);
//    delay(500);
//    digitalWrite(BUZZER,LOW);
//    }
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  x_gyr = gyro.x();
  y_gyr = gyro.y();
  z_gyr = gyro.z();
  imu::Vector<3> magneto = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  x_magneto = magneto.x();
  y_magneto = magneto.y();
  z_magneto = magneto.z();
  
  trama += BEG + String(apogeo_imu)+SEP+String(apogeo_barometer)+SEP+String(tiempoSegundos) +SEP+ String(Temperatura) + SEP +"pres: "+ String(Presion) + SEP + String(Altura) + SEP;
  
  trama += String(x_acc) + SEP + String(y_acc) + SEP + String(z_acc) + SEP + String(x_gyr) + SEP + String(y_gyr) + SEP + String(z_gyr);
  trama_to_send = trama;
  to_send = true;

//        myFile = SD.open("NUCLEO0.txt", FILE_WRITE);
//        if (myFile) {
//        myFile.println(trama); 
//        myFile.close(); // close the file     
//        }
  
  // ECUACIONES DE APOGEO Y LÓGICA DE CONTROL DE RECUPERACIÓN
  
  // Lo que se visualizará en el monitor Serial
 Serial.println(trama);

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
//


// //SD

  counter++;
  trama = "";
  delay(10);
}

void sendInfo(void *paremeter) // Loop secundario, nucleo 0.
{
  for(;;){
  //  Serial.println(String(counter) + "\t\t\t\t\t\t\t\t\t\t\t" + trama_to_send);
  Serial.println();
    if(to_send){
              myFile = SD.open("NUCLEO0.txt", FILE_WRITE);
        if (myFile) {
        //myFile.println(trama_to_send); 
        myFile.println(String(Altura) + "\t" + String(apogee_detection) + "\t" +String(apogeo_imu));
        myFile.close(); // close the file     
        }
      LoRa.beginPacket();
      LoRa.println(String(counter)+ trama_to_send);
      LoRa.endPacket();
    }
    to_send = false;
    trama_to_send = "";
    }
vTaskDelay(10);
}
