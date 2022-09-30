#include <LibPrintf.h>
#include <SPI.h>
#include <LoRa.h>

const int csPin = 5;         // LoRa radio chip select
const int resetPin = 3;       // LoRa radio reset
const int irqPin = 4;         // change for your board; must be a hardware interrupt pin

  float temperatura;
  float presion;
  float altura;
  float altura_maxima;
  float s_apogeo;
  float s_main;
  float x_acc;
  float y_acc;
  float z_acc;
  float media_cuadratica;
  float x_gyr;
  float y_gyr;
  float z_gyr;
  float x_mag;
  float y_mag;
  float z_mag;

String LoRaData;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  Serial.println("Lora Ready");
  if (!LoRa.begin(433E6)) {
    while (1);
  }
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    //Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      //Serial.println(LoRaData);
    }
    // Encontrando posiciones específicas.  
  int pos1=LoRaData.indexOf(',');
  int pos2=LoRaData.indexOf(',',pos1+1);
  int pos3=LoRaData.indexOf(',',pos2+1);
  int pos4=LoRaData.indexOf(',',pos3+1);
  int pos5=LoRaData.indexOf(',',pos4+1);
  int pos6=LoRaData.indexOf(',',pos5+1);
  int pos7=LoRaData.indexOf(',',pos6+1);
  int pos8=LoRaData.indexOf(',',pos7+1);
  int pos9=LoRaData.indexOf(',',pos8+1);
  int pos10=LoRaData.indexOf(',',pos9+1);
  int pos11=LoRaData.indexOf(',',pos10+1);
  int pos12=LoRaData.indexOf(',',pos11+1);
  int pos13=LoRaData.indexOf(',',pos12+1);
  int pos14=LoRaData.indexOf(',',pos13+1);
  int pos15=LoRaData.indexOf(',',pos14+1);
  int pos16=LoRaData.indexOf(',',pos15+1);
  int pos17=LoRaData.indexOf(',',pos16+1);
  int pos18=LoRaData.indexOf(',',pos17+1);
  int pos19=LoRaData.indexOf(',',pos18+1);
  int pos20=LoRaData.indexOf(',',pos19+1);
  int pos21=LoRaData.indexOf('}',pos20+1);

  // Creando substrings de cada variable
  String _temperatura = LoRaData.substring(pos4+1, pos5);
  String _presion = LoRaData.substring(pos5+1, pos6);
  String _altura = LoRaData.substring(pos6+1, pos7);
  String _altura_maxima = LoRaData.substring(pos7+1, pos8);
  String _s_apogeo = LoRaData.substring(pos9+1, pos10);
  String _s_main = LoRaData.substring(pos10+1, pos11);
  String _x_acc = LoRaData.substring(pos11+1, pos12);
  String _y_acc = LoRaData.substring(pos12+1, pos13);
  String _z_acc = LoRaData.substring(pos13+1, pos14);
  String _media_cuadratica = LoRaData.substring(pos14+1, pos15);
  String _x_gyr = LoRaData.substring(pos15+1, pos16);
  String _y_gyr = LoRaData.substring(pos16+1, pos17);
  String _z_gyr = LoRaData.substring(pos17+1, pos18);
  String _x_mag = LoRaData.substring(pos18+1, pos19);
  String _y_mag = LoRaData.substring(pos19+1, pos20);
  String _z_mag = LoRaData.substring(pos20+1, pos21);

  // Convirtiendo subStrings a flotantes
  temperatura=_temperatura.toFloat();
  presion=_presion.toFloat();
  altura=_altura.toFloat();
  altura_maxima=_altura_maxima.toFloat();
  s_apogeo=_s_apogeo.toFloat();
  s_main=_s_main.toFloat();
  x_acc=_x_acc.toFloat();
  y_acc=_y_acc.toFloat();
  z_acc=_z_acc.toFloat();
  media_cuadratica=_media_cuadratica.toFloat();
  x_gyr=_x_gyr.toFloat();
  y_gyr=_y_gyr.toFloat();
  z_gyr=_z_gyr.toFloat();
  x_mag=_x_mag.toFloat();
  y_mag=_y_mag.toFloat();
  z_mag=_z_mag.toFloat();

  // Para visualizar
  //Serial.println(x_acc);
  char text1[200];
  sprintf(text1,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",temperatura, presion, altura, altura_maxima, s_apogeo, s_main, x_acc, y_acc, z_acc, media_cuadratica, x_gyr, y_gyr, z_gyr, x_mag, y_mag, z_mag);
  Serial.println(text1);
  }
}
