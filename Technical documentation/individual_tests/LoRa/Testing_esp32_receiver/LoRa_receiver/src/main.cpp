#include <Arduino.h>
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>
#include <MS5611_baro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/math.h>

// Definiendo pines y demás del LoRa
#define ss 5
#define rst 14
#define dio0 2


//Definiendo pines y demás de la IMU
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


// Definiendo pines y demás del MS5611
MS5611_baro ms5611;
 
void setup() 
{
  // Inicializando comunicacion serial, sensores y...
  Serial.begin(115200);
  ms5611.begin();
  bno.begin();
  bno.setExtCrystalUse(true);

  //   if(!bno.begin())
  // {
  //   /* There was a problem detecting the BNO055 ... check your connections */
  //   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //   while(1);
  // }
    // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


  while (!Serial);
  Serial.println("LoRa Receiver");
 
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
  // Sensor MS5611
  double Temperatura = ms5611.getTemperature(true);
  float Presion = ms5611.getPressure(true);
  float Altura = (44330.0f * (1.0f - pow((Presion/pref), 0.1902949f)));


    // Sensor IMU
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    double x_acc = euler.x();
    double y_acc = euler.y();
    double z_acc = euler.z();
  

  int packetSize = LoRa.parsePacket();    // try to parse packet
  if (packetSize) 
  {
    
    Serial.print("Received packet: ");
 
    while (LoRa.available())              // read packet
    {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData); 
    }
    Serial.print("' with RSSI ");         // print RSSI of packet
    Serial.println(LoRa.packetRssi());
  }
}