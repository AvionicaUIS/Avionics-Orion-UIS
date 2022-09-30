#include <Wire.h>
#include "Arduino.h"

#include "mySD.h"
#include "SPI.h"


/* Set the delay between fresh samples */


// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address


//sensor barometrico

double pref = 0;
double Alt_cm, AltFilt, presion0, presionActual; 
float R = 287.0f;  // Gas const. [J / kg.K]
float g = 9.81f;  // Grav. accel. [m/s/s]


//Calcular apogeo
float apogeo, cont_apogeo;

//Contador segundos
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;
unsigned long tiempoSegundos = 0;

// SD
File myFile;
int CS_PIN = 5;


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);


  //SD
  pinMode(CS_PIN,OUTPUT);
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }

  
  
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{

 //SD
 myFile = SD.open("TEST11.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("holi");
 
    myFile.close(); // close the file
  }
  else {
    Serial.println("error opening test.txt");
    //Serial.println(referencePressure);
  }
  
}




