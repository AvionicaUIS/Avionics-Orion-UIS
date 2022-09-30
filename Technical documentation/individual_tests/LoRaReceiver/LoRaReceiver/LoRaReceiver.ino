// CODIGO QUE VA EN EL COHETE CON CALLBACK
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <Wire.h>
#include "mySD.h"

#define csPin 5
#define resetPin 14
#define irqPin 2

String outgoing;              // outgoing message
String Abort = "BORT!";
byte msgCount = 0;    // count of outgoing messages
//byte localAddress = 0xBB;     // address of this device
//byte destination = 0xFF;      // destination to send to
byte localAddress = 0xFF;     // address of this device
byte destination = 0xBB;

long lastSendTime = 0;        // last send time
int interval = 200;          // interval between sends

String Data;
int emergencia = 25; // acá se pone el pin de salida de emergencia 12 o 13 para el main parachute

// SD
//File myFile;
int counter = 0;

void setup() {
  Serial.begin(115200); // initialize serial
  delay(1000);
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin
  if (!LoRa.begin(433E6)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
   pinMode(emergencia, OUTPUT);
   Serial.println("LoRa CONECTÓ.");
}

void loop() {
  if (millis() - lastSendTime > interval) {
    Data="ACÁ SE IMPRIME LA TRAMA"; // Alisto los datos deseados (trama)
    sendMessage(Data);              // Envío los datos deseados a través de la función send Message
    Serial.println("enviando " + Data); // SOLO PARA VERIFICAR, SE PUEDE QUITAR
    lastSendTime = millis();            // timestamp the message
    interval = random(200) + 100;    // 2-3 seconds
    counter++;
  }
  onReceive(LoRa.parsePacket());
  }
  


void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
//  msgCount++;                           // increment message ID
}

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

  // if message is for this device, or broadcast, print details:
//  Serial.println("Received from: 0x" + String(sender, HEX));
//  Serial.println("Sent to: 0x" + String(recipient, HEX));
//  Serial.println("Message ID: " + String(incomingMsgId));
//  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
//  Serial.println("RSSI: " + String(LoRa.packetRssi()));
//  Serial.println("Snr: " + String(LoRa.packetSnr()));
//  Serial.println();
}
