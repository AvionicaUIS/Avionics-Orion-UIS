/*
 PID control with Thermocuple sensor
 Banco de Laboratorio de control
 Aux. David Eduardo Becerra S. 
*/
#include "mySD.h"
#include <Thermocouple.h>
#include <MAX6675_Thermocouple.h>

#define SCK_PIN_1 10
#define CS_PIN_1 9
#define SO_PIN_1 8

#define SCK_PIN_2 6
#define CS_PIN_2 5
#define SO_PIN_2 4

#define cssd 2    // Definiendo pines para SD
File myFile;                                      // SD file name




Thermocouple* Load_Thermo;
Thermocouple* Air_Thermo;


// the setup function runs once when you press reset or power the board
void setup() {
Serial.begin(115200);
SPI.begin();
pinMode(cssd, OUTPUT);

//--------------------------------------------------------------------------------------------------------------
  if (SD.begin(cssd))  // Validación status SD
  {
    Serial.println("TARJETA SD DETECTADA - Lista para usar");}
    else{
    Serial.println("TARJETA SD NO DETECTADA - Inserte tarjeta o revise conexión.");
    return;
  }
//--------------------------------------------------------------------------------------------------------------

  Load_Thermo = new MAX6675_Thermocouple(SCK_PIN_1, CS_PIN_1, SO_PIN_1);
  Air_Thermo = new MAX6675_Thermocouple(SCK_PIN_2, CS_PIN_2, SO_PIN_2);
}

// the loop function runs over and over again forever
void loop() {
  // Reads temperature
  const double celsius_load = Load_Thermo->readCelsius();
  const double celsius_air = Air_Thermo->readCelsius();
  
  
  // Output of information
  
  Serial.println(celsius_load);
  delay(1000);
  Serial.println(celsius_air);

  myFile = SD.open("NUCLEO0.txt", FILE_WRITE);
  if (myFile) {
  myFile.println(); 
  myFile.close(); }// close the file }
  delay(1000); // optionally, only to delay the output of information in the example.
}
