#ifndef __MS5611_BARO_H__
#define __MS5611_BARO_H__

#include "Arduino.h"

#define MS5611_BARO_ADDRESS (0X77) //Address sensor (0x77), important cause could be (0x76), but isn't the case

// Adicionalmente, es necesario tomar diferentes direcciones seriales: adc, leer D1 y D2...
// Estas direcciones se pueden encontrar en la página 10 del datasheet.

#define MS5611_BARO_RESET     (0x1E)
#define MS5611_BARO_D1        (0x40)  // Importante resaltar que la dirección varia segun resolución.
#define MS5611_BARO_D2        (0x50)  // Importante resaltar que la dirección varia segun resolución.
#define MS5611_BARO_ADC       (0x00)
#define MS5611_BARO_READ_PROM (0xA2)


// Defino las posibles resoLuciones a las que puede funcionar el sensor

typedef enum
{
    MS5611_BARO_OSR_4096 = 0x08,
    MS5611_BARO_OSR_2048 = 0x06,
    MS5611_BARO_OSR_1024 = 0x04,
    MS5611_BARO_OSR_512  = 0x02,
    MS5611_BARO_OSR_256  = 0x00
} MS5611_OSR;



class MS5611_baro //Aquí creo la clase llamada MS5611_barometro
{
    public:
        bool begin(MS5611_OSR osr = MS5611_BARO_OSR_4096); // JARZ
        bool isConnected();                 // RobTillard
        uint32_t RawTemperature(void);      // JARZ
	    uint32_t RawPressure(void);         // JARZ
        double getTemperature(bool compensation = true);   // JARZ
	    int32_t getPressure(bool compensation = true);     // JARZ
	    void setOversampling(MS5611_OSR osr);
    	MS5611_OSR getOversampling(void);

    private:
        uint16_t c[6];   // Declaro como variables de 16 bits los coeficientes del vector C. Pág 8.
        uint8_t convTime;    // Variable de tiempo de conversión dependiendo de la resolución.
        uint8_t uosr;
	    int32_t TEMP2;
	    int64_t OFF2, SENS2;

        void reset();
        void readPROM();

        uint16_t readRegister16(uint8_t reg);
    	uint32_t readRegister24(uint8_t reg);


}; 

#endif // __MS5611_BARO_H__