#include <SPI.h>
#include <LoRa.h>

int counter = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

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
  LoRa.print(counter);
  LoRa.print("La idea es graduarnos, xd.");
  LoRa.print("| entre más largo el mensaje, mejor.");
  LoRa.print(" aiuda!!!");

  LoRa.endPacket();

  counter++;

  delay(0.2);
}
