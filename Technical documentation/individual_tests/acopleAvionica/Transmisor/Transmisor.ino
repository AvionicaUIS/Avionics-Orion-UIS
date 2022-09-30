#include <DHT.h>
#include <DHT_U.h>
//Motor
#include <Servo.h>

//Sensor inercial
#include <LSM6.h>
#include <LPS.h>
#include <Wire.h>

//Sensor de luz
#include <AnalogUVSensor.h>

//Para la antena
#include <printf.h>
#include <RF24_config.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//ANTENA DECLARACIÓN
//Declaremos los pines CE y el CSN para la antena
#define CE_PIN 9
#define CSN_PIN 10
 
//Variable con la dirección del canal por donde se va a transmitir
byte direccion[6] ={'c','o','h','e','t','e'};
//const uint64_t canal = 990929;

//creamos el objeto radio (NRF24L01)
RF24 radio(CE_PIN, CSN_PIN);

//Vector con los datos a enviar
float data[5];

//DECLARACIÓN SENSOR DHT
// Definimos el pin digital donde se conecta el sensor de temperatura
#define DHTPIN 2
// Dependiendo del tipo de sensor
#define DHTTYPE DHT11
 
// Inicializamos el sensor DHT11
DHT dht(DHTPIN, DHTTYPE);

//DECLARACIÓN IMU
//Se unicializa el sensor imu
LSM6 imu;
LPS ps;
char report[80];

//DECLARACIÓN SERVO
//Se inicializa el servo
Servo myservo;  // crea el objeto servo
int pos = 0;    // posicion del servo
float aceleracion;
float angular;

 void setup() {
  // Inicializamos comunicación serie
  Serial.begin(9600);
 
  // Comenzamos el sensor DHT
  dht.begin();

  //Comenzamos el sensor de LUZ

   //Comenzamos el sensor de IMU
   Wire.begin();
   myservo.attach(3);  // vincula el servo al pin digital 3
 
  {
    if (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    while (1);
  }
  imu.enableDefault();
 }
 //LPS
 {
   if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }

  ps.enableDefault();
}
  //INICIALIZACIÓN ANTENA
  //inicializamos el NRF24L01 
  radio.begin();
  //inicializamos el puerto serie
  Serial.begin(9600); 
   radio.openWritingPipe(direccion);
   radio.stopListening();  // put radio in TX mode
}

//Función para el sensor de luz
float indexUV(uint16_t pin, int16_t analog_max, float voltage_max)
{
  uint16_t raw = analogRead(pin);
  float millivolt = (voltage_max * raw * 1000) / analog_max;

  if (millivolt < 50)
  {
    return 0.0;
  }
  if (millivolt < 227)
  {
    // linear interpolation between 0..1
    float uvi = 0.0 + (millivolt - 50.0) / (227.0 - 50.0);
    return uvi;
  }
  // linear interpolation between 1..11
  // formula derived with spreadsheet.
  float uvi = 0.0104865310 * millivolt - 1.289154988;
  return uvi;
}
 
void loop() {
  // Esperamos 1 segundos entre medidas
  delay(200);
  

  // Leemos la humedad relativa
  float h = dht.readHumidity();
  // Leemos la temperatura en grados centígrados (por defecto)
  float t = dht.readTemperature();
  // Leemos la temperatura en grados Fahreheit
  //float f = dht.readTemperature(true);
 
  // Comprobamos si ha habido algún error en la lectura
  if (isnan(h) || isnan(t)) {
    Serial.println("Error obteniendo los datos del sensor DHT11");

     // return;
  }
  
  //LECTURA DE LA LUZ
 float rad = indexUV(A1, 5.0, 1023); 
 
    //Lectura del IMU
    imu.read();
    
    snprintf(report, sizeof(report), "A: %6d %6d %6d    G: %6d %6d %6d",
    imu.a.x, imu.a.y, imu.a.z,
    imu.g.x, imu.g.y, imu.g.z);
 // Serial.println(report);
    aceleracion=sqrt(pow(imu.a.x,2)+pow(imu.a.y,2)+pow(imu.a.z,2));
    Serial.println(aceleracion);
    angular=sqrt(pow(imu.g.x,2)+pow(imu.g.y,2)+pow(imu.g.z,2));
    
  float pressure = ps.readPressureMillibars();
  float altitude = ps.pressureToAltitudeMeters(pressure);

 Serial.print(" -------------------------------------------------\n");
  Serial.print("Humedad: ");
  Serial.print(h);
  Serial.print(" kg_w/kg_a \t");
  Serial.print("Temperatura: ");
  Serial.print(t);
  Serial.print(" C  \t  ");
  Serial.print("Radiación: ");
  Serial.print (rad);
  Serial.print(" \t ");  
  Serial.print("Presión: ");
  Serial.print(pressure);
  Serial.print(" mbar\t ");  
   Serial.print("Aceleración: ");
  Serial.println(aceleracion);
  Serial.print(" \t "); 
  Serial.print("Altitud: ");
  Serial.print(altitude);
  Serial.print(" m\t ");  
  Serial.print("Posición angular: ");
  Serial.print(angular);
  Serial.print(" \t ");
  Serial.println (" \t\n  ");

 
{ //LOOP ANTENA
 //cargamos los datos en la variable datos[]
 data[0]= h; //Humedad
 data[1]= t; //Temperatura
 data[2]= rad; //Radiación
 data[3]=aceleracion;//Aceleración
 data[4]=angular;//Posición angular

 //enviamos los datos
 bool ok = radio.write(&data,sizeof(data));
  //reportamos por el puerto serial los datos enviados 
  if(ok)
  {
     Serial.print("Datos enviados: \n"); 
     Serial.print("Humedad: "); 
     Serial.println(data[0]); 
     Serial.print("\nTemperatura: "); 
     Serial.println(data[1]); 
     Serial.print("\nRadiación: "); 
     Serial.println(data[2]); 
     Serial.print("\nAcelelración: "); 
     Serial.println(data[3]); 
     Serial.print("\nPosición angular: "); 
     Serial.println(data[4]); 
  }
  else
  {
     Serial.println("no se ha podido enviar");
  }

}
  
}
