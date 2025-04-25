#ifndef INC_HMC5883_H_
#define INC_HMC5883_H_

#include "sys_def_CM4.h"

#define CONF_A  00  //R/W Configuration Register A
#define CONF_B  01  //R/W Configuration Register B
#define MODE    02  //R/W Mode Register
#define DATA_XH 03  //R Data Output X MSB Register
#define DATA_XL 04  //R Data Output X LSB Register
#define DATA_ZH 05  //R Data Output Z MSB Register
#define DATA_ZL 06  //R Data Output Z LSB Register
#define DATA_YH 07  //R Data Output Y MSB Register
#define DATA_YL 08  //R Data Output Y LSB Register
#define STATUS  09  //R Status Register
#define ID_A    10  //R Identification Register A
#define ID_B    11  //R Identification Register B
#define ID_C    12  //R Identification Register C

#define HMC_I2C_ADR	0x3C
#define MAG_TIMEOUT	2	//czas w milisekundach na odczyt danych z magnetometru. Nominalnie jest to ok xx0us na adres i 3 bajty danych

#define CZULOSC_HMC5883	5e-7	//1 bit odpowiada 5 mGauss a 1 mGauss to 100nT




//definicje funkcji
uint8_t StartujPomiarMagHMC(void);
uint8_t StartujOdczytMagHMC(void);
uint8_t CzytajMagnetometrHMC(void);
uint8_t SprawdzObecnoscHMC5883(void);
uint8_t InicjujMagnetometrHMC(void);


//deklaracje funkcji



#endif /* INC_HMC5883_H_ */
