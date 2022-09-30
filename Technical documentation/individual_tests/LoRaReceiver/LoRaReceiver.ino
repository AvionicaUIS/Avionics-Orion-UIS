#include <SPI.h>
#include <LoRa.h>

const int csPin = 5;         // LoRa radio chip select
const int resetPin = 3;       // LoRa radio reset
const int irqPin = 4;         // change for your board; must be a hardware interrupt pin

void setup() {
  Serial.begin(115200);
  while (!Serial);
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  Serial.println("LoRa Receiver");
  

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
  }
}
