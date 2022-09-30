//------------------------------------
//--------------- LIBRARIES
//------------------------------------
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
//------------------------------------
//--------------- DEFINICIÓN DE PINES Y VARIABLES
//------------------------------------
#define ss 5                  // Definiendo pines para LoRa
#define rst 14
#define dio0 2
#define cssd 4                // Definiendo pines para SD
#define BUZZER 26             // Definiendo pines para dispositivos alternos
#define LED 25
#define PYRO_APOGEE 12
#define PYRO_MAIN 13

int counter = 0;              // Algunas variables necesarias
String trama;                 // Variables trama de datos
String BEG = "{";
String END = "}";
String SEP = ",";
float presionActual = 0;      // Variables barométrico
float presion0 = 0;
float Temperatura = 0;
float Presion = 0;
float Altura = 0;
bool apogeo_barometer = false;
bool main_parachute = false;
bool apogeo_imu = false;      // Variables IMU
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
float alt_anterior =0;
float alt_actual=0;
float altmax = 0;
const int margen_altura = 2;
float apogeo2;
float altura_1=0;
float altura_2=0;
String outgoing;              // LoRa Callback
String Abort = "BORT!";
byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;
long lastSendTime = 0;        // last send time
int interval = 200;           // interval between sends
String Data;
int emergencia = 13; // acá se pone el pin de salida de emergencia 12 o 13 para el main parachute


bool to_send = false;         // Banderas de activación para el uso de 2 núcleos
String trama_to_send;

unsigned long tiempo1 = 0;    // Variables para crear registro de tiempo
unsigned long tiempo2 = 0;
unsigned long tiempoSegundos = 0;

//------------------------------------
//--------------- CONSTRUCTORES Y DEMÁS
//------------------------------------
MS5611 MS5611(0x77);                              // LoRa I2C Address
File myFile;                                      // SD file name
#define BNO055_SAMPLERATE_DELAY_MS (100)          // IMU BNO055 - tasa de muestreo
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);  // IMU BNO055 - I2C address


//------------------------------------
//--------------- BÚCLE SETUP
//------------------------------------
void setup(){
// Inicializando comunicacion serial, sensores y...
Serial.begin(115200);
SPI.begin();
MS5611.begin();
pinMode(cssd, OUTPUT);
pinMode(BUZZER, OUTPUT);
pinMode(PYRO_APOGEE, OUTPUT);
pinMode(PYRO_MAIN, OUTPUT);


 
while(!Serial);
tiempo1 = millis();

                    //CONFIGURACIÓN NECESARIA PARA USAR 2 NÚCLEOS
//--------------------------------------------------------------------------------------------------------------
//xTaskCreatePinnedToCore(readTaskC1, "readSensors", 10000, NULL, 1, &readTaskC1,  1);   //Núcleo 1
//  delay(500); 

//3xTaskCreatePinnedToCore(sendInfo, "sendTrama", 10000, &sendTaskC0, 0, &sendTaskC0,  0);     //Núcleo 0
  delay(500); 
//--------------------------------------------------------------------------------------------------------------
  if(!bno.begin())      // Validación status IMU
  {
    Serial.print("BNO055 FAILED");
    while(1);
  }
     Serial.println("BNO055 DETECTADO - READY");
     digitalWrite(BUZZER,HIGH);
     delay(500);
     digitalWrite(BUZZER,LOW);
//--------------------------------------------------------------------------------------------------------------
  LoRa.setPins(ss, rst, dio0);   // Validación status LoRa
  if(!LoRa.begin(433E6))
  {
    Serial.println("LORA FAILED.");
    while (1);
  }
     pinMode(emergencia, OUTPUT);
    Serial.println("LORA DETECTADO - READY.");    
    digitalWrite(BUZZER,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
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
  if (MS5611.begin() == true)     // Validación status MS5611
  {
    Serial.println("MS5611 DETECTADO - READY.");
    digitalWrite(BUZZER,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);
  }
    else{
    Serial.println("MS5611 FAILED.");
    while (1){
  }}
  MS5611.setOversampling(OSR_ULTRA_HIGH);
//--------------------------------------------------------------------------------------------------------------
    for (int cal_int = 0; cal_int < 500 ; cal_int ++)  // Calculando la presión ambiente "pref"
    {
    MS5611.read();          
    delay(10);
    presionActual = MS5611.getPressure();
    presion0 += presionActual;
    Serial.println(presionActual);
    }
    presion0 = presion0 / 500;
    digitalWrite(BUZZER,HIGH);
    delay(100);
    digitalWrite(BUZZER,LOW);
//    Serial.println(presion0);
//--------------------------------------------------------------------------------------------------------------
bno.setExtCrystalUse(true);                       // IMU BNO055 - Configuración necesaria
}   // Fin Void Setup

//------------------------------------
//--------------- BÚCLE LOOP PRINCIPAL
//------------------------------------

void loop() // Loop principal, nucleo 1.
{
  // Contador de tiempo segundos / milisegundos
    tiempo2 = millis();
    if(tiempo2 > (tiempo1+1000)){  //Si ha pasado 1 segundo ejecuta el IF
    tiempo1 = millis(); //Actualiza el tiempo actual
    tiempoSegundos = tiempo1/1000;}
//--------------------------------------------------------------------------------------------------------------
  MS5611.read();
  Temperatura = MS5611.getTemperature();
  Presion = MS5611.getPressure();
  Altura = (44330.0 * (1.0 - pow(Presion/presion0, 0.1903)));

  trama += BEG + String(Temperatura)+SEP+String(Presion)+SEP+String(Altura);
  alt_anterior=alt_actual;
  alt_actual=Altura;
  if(alt_actual>altmax){
  altmax=alt_actual;
  }
  trama += SEP + String(altmax);
  apogeo2 = ((alt_actual+altura_1+altura_2)/3) + margen_altura - altmax;
  if(apogeo2<=-1){
    apogeo_barometer = true;
    digitalWrite(BUZZER,HIGH);
    digitalWrite(PYRO_APOGEE, HIGH);
    delay(8000);
    digitalWrite(PYRO_APOGEE, LOW);
    digitalWrite(BUZZER,LOW);
    }
  if((Altura <= 550)&&(apogeo_barometer==true)){
    main_parachute = true;
     digitalWrite(BUZZER,HIGH);
     digitalWrite(PYRO_MAIN, HIGH);
     delay(8000);
     digitalWrite(PYRO_MAIN, LOW);
     digitalWrite(BUZZER,LOW);
    }
  trama += SEP + String(apogeo2) + SEP +String(apogeo_barometer)+ SEP+ String(main_parachute);
  altura_2 = altura_1;
  altura_1=alt_actual;

  // Sensor IMU
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
  media_cuadratica = sqrt(pow(x_acc,2)+pow(y_acc,2)+pow(z_acc,2));
  trama += SEP+ String(x_acc) + SEP + String(y_acc) + SEP + String(z_acc) + SEP + String(media_cuadratica);
  trama += SEP+ String(x_gyr) + SEP + String(y_gyr) + SEP + String(z_gyr) + SEP + String(x_magneto) + SEP + String(y_magneto) + SEP + String(z_magneto)+END;
  
  //------- TARJETA SD

  myFile = SD.open("NUCLEO0.txt", FILE_WRITE);
  if (myFile) {
  myFile.println(trama); 
  //myFile.println(String(Altura) + "\t" + String(apogee_detection) + "\t" +String(apogeo_imu));
  myFile.close(); // close the file

  //-------- LoRa
  if (millis() - lastSendTime > interval) {
    Data=trama; // Alisto los datos deseados (trama)
    sendMessage(Data);              // Envío los datos deseados a través de la función send Message
    //Serial.println("enviando " + Data); // SOLO PARA VERIFICAR, SE PUEDE QUITAR
    lastSendTime = millis();            // timestamp the message
    interval = random(100) + 50;    // 2-3 seconds
    counter++;
  }
  onReceive(LoRa.parsePacket());
  
  counter++;
  trama = "";
  delay(10);
  Serial.println(trama);
  apogeo_barometer == false;
  main_parachute == false;
}
}


//------------------------------------
//--------------- FUNCION SEND MESSAGE
//------------------------------------
void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
}
//------------------------------------
//--------------- FUNCIÓN ON RECEIVE
//------------------------------------
void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) {
    incoming += (char)LoRa.read();
  }
  if (incoming == Abort){
//    Serial.println("ABORTARRRR");
    digitalWrite(emergencia, HIGH);
    digitalWrite(PYRO_APOGEE, HIGH);
  }
  if (incoming != Abort){
    //Serial.println("ABORTARRRR");
    digitalWrite(emergencia, LOW);
  }
  if (incomingLength != incoming.length()) {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  Serial.println("Message: " + incoming);
}
