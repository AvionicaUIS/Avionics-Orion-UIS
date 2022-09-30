#include "HX711.h"
#include <Wire.h> 
#include <Thermocouple.h>
#include <MAX6675_Thermocouple.h>
#include "SD.h"


//#include <LiquidCrystal_I2C.h> //Download here: https://electronoobs.com/eng_arduino_liq_crystal.php
//LiquidCrystal_I2C lcd(0x27,16,2); //sometimes the LCD adress is not 0x3f. Change to 0x27 if it dosn't work.

#define cssd 3    // Definiendo pines para SD
File myFile;                                      // SD file name

#define DOUT  5
#define CLK  8

#define SO_PIN 4
#define SCK_PIN 6

#define CS_PIN_1 9
#define CS_PIN_2 7

Thermocouple* Load_Thermo;
Thermocouple* Air_Thermo;




HX711 scale;
//270.5; //=75g
float calibration_factor = 6140; //=75g
float output;
float Thrust;
int readIndex;
float total=0;
float average=0;
float average_last=0;
const int cycles=20;
float readings[cycles];

void setup() {
  Serial.begin(115200);
  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare();  //Reset the scale to 0


  Load_Thermo = new MAX6675_Thermocouple(SCK_PIN, CS_PIN_1, SO_PIN);
  Air_Thermo = new MAX6675_Thermocouple(SCK_PIN, CS_PIN_2, SO_PIN);
  //lcd.init();                         //Init the LCD
  //lcd.backlight();                    //Activate backlight
pinMode(cssd, OUTPUT); 

//--------------------------------------------------------------------------------------------------------------
  if (SD.begin(cssd))  // Validación status SD
  {
    Serial.println("TARJETA SD DETECTADA - Lista para usar");}
    else{
    Serial.println("TARJETA SD NO DETECTADA - Inserte tarjeta o revise conexión.");
    return;
  }
}

void loop() {
  //-------------------------------------------------- CELDA
//  scale.set_scale(calibration_factor); //Adjust to this calibration factor
//  output=scale.get_units(), 2;
  //Serial.println(output);

  //--------------------------------------------------- TERMOS
  const double celsius_load = Load_Thermo->readCelsius();
//  const double celsius_air = Air_Thermo->readCelsius();
  Serial.println(celsius_load);
  delay(500);
//  Serial.println(celsius_air);

  //---------------------------------------------------- SD
//                myFile = SD.open("BANCO.txt", FILE_WRITE);
//        if (myFile) {
//        myFile.println(output); 
//        myFile.close(); // close the file     
//        }
//  
  //lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print(" Thrust: ");
  //lcd.setCursor(0,1);
  //lcd.print(output);

}
