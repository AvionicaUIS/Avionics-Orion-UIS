#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <MS5611.h>
MS5611 MS5611(0x77);                              // LoRa I2C Address
#include "mySD.h"
float Temperatura = 0;
#define BNO055_SAMPLERATE_DELAY_MS (100)          // IMU BNO055 - tasa de muestreo
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);  // IMU BNO055 - I2C address
#define BUZZER 26
#define cssd 4    // Definiendo pines para SD

File myFile;                                      // SD file name


float x_acc = 10;
float y_acc = 12;
float z_acc = 10;
float x_gyr = 10;
float y_gyr = 0;
float z_gyr = 0;
float apogee_detection = 0;
float presionActual = 0;
float presion0 = 0;
float Presion = 0;
float Altura = 0;
bool apogeo_imu = false;
bool apogeo_baro=false;
float alt_anterior =0;
float alt_actual=0;
float altmax = 0;
const int margen_altura = 2;
float apogeo2;
float altura_1=0;
float altura_2=0;


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup()
{
Serial.begin(115200);
  MS5611.begin();
pinMode(BUZZER, OUTPUT);  
pinMode(cssd, OUTPUT);

Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  if (MS5611.begin() == true)     // Validación status MS5611
  {
    Serial.println("MS5611 DETECTADO - READY.");}
    else{
    Serial.println("MS5611 FAILED.");
    while (1){
  }}
  MS5611.setOversampling(OSR_ULTRA_HIGH);
//--------------------------------------------------------------------------------------------------------------
    for (int cal_int = 0; cal_int < 270 ; cal_int ++)  // Calculando la presión ambiente "pref"
    {
    MS5611.read();          
    delay(10);
    presionActual = MS5611.getPressure();
    presion0 += presionActual;
    Serial.println(presionActual);
    }
    presion0 = presion0 / 270;
    Serial.println(presion0);
  bno.setExtCrystalUse(true);
//
//  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
    digitalWrite(BUZZER,HIGH);
  delay(100);
  digitalWrite(BUZZER,LOW);
  if (SD.begin(cssd))  // Validación status SD
  {
    Serial.println("TARJETA SD DETECTADA - READY.");}
    else{
    Serial.println("TARJETA SD FAILED.");
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
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
   MS5611.read();
  Temperatura = MS5611.getTemperature();
  Presion = MS5611.getPressure();
  Altura = (44330.0 * (1.0 - pow(Presion/presion0, 0.1903)));
//  Altura = (44330.0f * (1.0f - pow((Presion/presion0), 0.1902949f)));
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  x_acc = accel.x();
  y_acc = accel.y();
  z_acc = accel.z();
  apogee_detection = sqrt(pow(x_acc,2)+pow(y_acc,2)+pow(z_acc,2));
  Serial.println(apogee_detection);
//  apogee_detection = sqrt((x_acc*x_acc)+(y_acc*y_acc)+(z_acc*z_acc));
  if (apogee_detection < 4.2){
    apogeo_imu == true;
    //digitalWrite(BUZZER,HIGH);
   // delay(500);
    //digitalWrite(BUZZER,LOW);
    }


      //Apogeo Baro
  alt_anterior=alt_actual;
  alt_actual=Altura;
if(alt_actual>altmax){
  altmax=alt_actual;
  }
  
   apogeo2 = ((alt_actual+altura_1+altura_2)/3) + margen_altura - altmax;
  if(apogeo2<=-1){
    digitalWrite(BUZZER,HIGH);
    delay(500);
    digitalWrite(BUZZER,LOW);}
  altura_2 = altura_1;
  altura_1=alt_actual;
  
//float  apogeo2 = alt_actual - alt_anterior;
//  if(apogeo2<-0.5){
//  apogeo_baro = true;
//      digitalWrite(BUZZER,HIGH);
//    delay(500);
//    digitalWrite(BUZZER,LOW);


       delay(10);
  
        myFile = SD.open("NUCLEO9.txt", FILE_WRITE);
        if (myFile) {
        //myFile.println(trama_to_send);
        myFile.println(String(apogeo2));
        //myFile.println(String(apogee_detection));
       // myFile.println(String(Altura) + "\t" + String(apogee_detection) + "\t" +String(apogeo_imu) + "\t" + String(apogeo2) +"\t" + String(apogeo_baro));
        myFile.close(); // close the file     
        }
  
}
