/*
       CANASAT GUANESAT SCUA
*/
//LIBRARIES
//#include <SPI.h>
//#include <LoRa.h>
//#include <Adafruit_BMP085.h>

//BOARD PINOUT
//#define LMISO 19
//#define LMOSI 27
//#define LSCK 5
//#define LIRQ 26
//#define LCS 18
//#define LRST 12
//#define SDCS 2
//#define M1 33
//#define M2 32
//#define M3 25
//#define S1 14
//#define S2 15
//#define S3 13
#define BUZ 26
//#define GTX 17
//#define GRX 16
#define LED 25
//#define BAT 36
//#define NH3 39
//#define CO 34
//#define NO2 35

//OBJECTS
//Adafruit_BMP085 bmp;


//VARIABLES

void setup() {
  Serial.begin(115200);
  pinMode(BUZ, OUTPUT);
  pinMode(LED, OUTPUT);
  Serial.println("CANSAT MCC BEGIN");
  ledcSetup(0,1000,8);
  ledcAttachPin(BUZ,0);
}

void loop() {
  //BMP180
  digitalWrite(LED,HIGH);
  ledcWrite(0,255);
  delay(200);
  digitalWrite(LED,LOW);
  ledcWrite(0,0);
  delay(200); 
}
