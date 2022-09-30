#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>

#define RXD2 1
#define TXD2 0
HardwareSerial neogps(1);

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  //Begin serial communication Arduino IDE (Serial Monitor)

  //Begin serial communication Neo6mGPS
  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
    
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (neogps.available())
    {
      if (gps.encode(neogps.read()))
      {
        newData = true;
      }
    }
  }

  //If newData is true
  if(newData == true)
  {
    newData = false;
    Serial.println(gps.satellites.value());
    print_speed();
  }
  else
  {

    Serial.print("No Data");
  }  
  
}

void print_speed()
{
       
  if (gps.location.isValid() == 1)
  {
   //String gps_speed = String(gps.speed.kmph());
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(),6);

    Serial.print("Lng: ");
    Serial.print(gps.location.lng(),6);
//
//    display.setCursor(25, 35);
//    display.print("Speed: ");
//    display.setCursor(65, 35);
//    display.print(gps.speed.kmph());
//    
//    display.setTextSize(1);
//    display.setCursor(0, 50);
//    display.print("SAT:");
//    display.setCursor(25, 50);
//    display.print(gps.satellites.value());
//
//    display.setTextSize(1);
//    display.setCursor(70, 50);
//    display.print("ALT:");
//    display.setCursor(95, 50);
//    display.print(gps.altitude.meters(), 0);
//
//    display.display();
    
  }
  else
  {
    Serial.print("No Data");
  }  

}
