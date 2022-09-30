#include <SPI.h>
#include <LoRa.h>
#define ss 5
#define rst 14
#define dio0 2
int counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  LoRa.setPins(ss, rst, dio0);  
  Serial.println("LoRa Sender");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("hello ");
  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(500);
}
