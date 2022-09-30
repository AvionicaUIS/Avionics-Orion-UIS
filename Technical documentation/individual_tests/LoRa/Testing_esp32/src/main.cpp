#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include <MS5611_baro.h>
#include <

// Definiendo pines para LoRa
#define ss 5
#define rst 14
#define dio0 2

//Definiendo pines y demás de la IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

// Inicializando constructor para MS5611
MS5611_baro ms5611;

// Algunas variables necesarias
int counter = 0;

// Búcle setup
void setup() 
{
  Serial.begin(115200); 
  ms5611.begin();

  while (!Serial);
  Serial.println("LoRa Sender");
 
  LoRa.setPins(ss, rst, dio0);    //setup LoRa transceiver module
  
  while (!LoRa.begin(433E6))     //433E6 - Asia, 866E6 - Europe, 915E6 - North America
  {
    Serial.println(".");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LoRa Initializing OK!");
}
 
void loop() 
{

  double Temperatura = ms5611.getTemperature(true);
  float Presion = ms5611.getPressure(true);
  //float Altura = (44330.0f * (1.0f - pow((Presion/pref), 0.1902949f)));

  // Lo que se visualizará en el monitor Serial
  Serial.print("Sending packet: ");
  Serial.print(counter);
  Serial.print(" - Temperatura: ");
  Serial.print(Temperatura);
  Serial.print(", Presion: ");
  Serial.print(Presion);
  // Serial.print("Altura: ");
  // Serial.println(Altura);
  Serial.println();

 
  LoRa.beginPacket();   //Send LoRa packet to receiver
  LoRa.print(counter);
  LoRa.print(" - Temperatura");
  LoRa.print(Temperatura);
  LoRa.print(", Presion: ");
  LoRa.print(Presion);
  LoRa.endPacket();
 
  counter++;
 
  delay(100);
}