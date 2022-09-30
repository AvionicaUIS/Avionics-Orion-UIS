#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <MS5611_baro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <mySD.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Definiendo pines para LoRa
#define ss 5
#define rst 14
#define dio0 2

//Definiendo pines y demás de la IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// Definiendo pines y demás del MS5611
MS5611_baro ms5611;

// Defininendo pines y demás de la SD
File myFile;
const int CS_PIN = 2;

// Algunas variables necesarias
int counter = 0;
float presionActual = 0;
float presion0 = 0;



// Búcle setup
void setup() 
{
 // Inicializando comunicacion serial, sensores y...
  Serial.begin(115200);
  ms5611.begin();
  bno.begin();
  bno.setExtCrystalUse(true);

  // Proceso para comunicación SPI
  pinMode(ss, OUTPUT); // LoRa
  digitalWrite(ss,HIGH); // Se ponen en estado alto para que no se activen
  pinMode(CS_PIN, OUTPUT);  // SD Card
  digitalWrite(CS_PIN, HIGH); // Se ponenen en estado alto para que no se activen hasta mi señal.


  //   if(!bno.begin())
  // {
  //   /* There was a problem detecting the BNO055 ... check your connections */
  //   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while(1);
  // }
    // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


  while (!Serial);
  // Validación status LoRa
  Serial.println("LoRa Sender");
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");

  // Validación status SD
  if (SD.begin(2,23,19,18))
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }

  // Calculando la presión ambiente "pref"
  for (int cal_int = 0; cal_int < 270 ; cal_int ++) {
    delay(10);
    presionActual = ms5611.getPressure();
    presion0 += presionActual;
    //Serial.println(presionActual);
  }
  presion0 = presion0 / 270;

}
 
void loop() 
{
  // Sensor MS5611
  double Temperatura = ms5611.getTemperature(true);
  float Presion = ms5611.getPressure(true);
  float Altura = (44330.0f * (1.0f - pow((Presion/presion0), 0.1902949f)));

  // Sensor IMU
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  double x_acc = euler.x();
  double y_acc = euler.y();
  double z_acc = euler.z();

  // Lo que se visualizará en el monitor Serial
  Serial.print("Sending packet: ");
  Serial.print(counter);
  Serial.print(" - Temperatura: ");
  Serial.print(Temperatura);
  Serial.print(", Presion: ");
  Serial.print(Presion);
  Serial.print(", Altura: ");
  Serial.print(Altura);
  Serial.print(", X_acc: ");
  Serial.print(x_acc);
  Serial.print(", Y_acc: ");
  Serial.print(y_acc);
  Serial.print(", Z_acc: ");
  Serial.print(z_acc);

  // Serial.print("Altura: ");
  // Serial.println(Altura);
  Serial.println();

 
 // Lo que se enviará a través del LoRa
 digitalWrite(ss,LOW);
 digitalWrite(CS_PIN, HIGH);
 delay(1);
 SPI.begin();
  LoRa.beginPacket();
  LoRa.print(counter);
  LoRa.print(" - Temperatura: ");
  LoRa.print(Temperatura);
  LoRa.print(", Presion: ");
  LoRa.print(Presion);
  LoRa.print(", Altura: ");
  LoRa.print(Altura);
  LoRa.print(", X_acc: ");
  LoRa.print(x_acc);
  LoRa.print(", Y_acc: ");
  LoRa.print(y_acc);
  LoRa.print(", Z_acc: ");
  LoRa.print(z_acc);
  LoRa.endPacket();
  SPI.end();

 
  counter++;

  // Guardando datos en SD
  digitalWrite(CS_PIN, LOW);
  digitalWrite(ss, HIGH);
  delay(1);
    SPI.begin(); // Inicializo comunicación SPI (debe ir después de usar digitalWrite)

  myFile = SD.open("TEST1.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(counter);
    myFile.print(" - Temperatura: "); 
    myFile.print(Temperatura);
    myFile.print(", Presion: ");    
    myFile.print(Presion);
    myFile.print(", Altura: ");
    myFile.print(Altura);
    myFile.print(", X_acc: ");
    myFile.print(x_acc);
    myFile.print(", Y_acc: ");
    myFile.print(y_acc);
    myFile.print(", Z_acc: ");;
    myFile.print(z_acc);
 
    myFile.close(); // close the file
  }
  else {
    Serial.println("error opening test.txt");
  }
  SPI.end();


 
  delay(100);
}
