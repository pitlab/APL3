
#include "sys_def_CM4.h"



//definicje funkcji
unsigned char BMP085_StartTempConversion(void);
float BMP085_ReadTemperature(void);
unsigned char BMP085_StartPresConversion(unsigned char chOversampling);
float BMP085_ReadPressure(void);
unsigned short BMP085_Read16bit(unsigned char chAddress);
unsigned char InitBMP085(void);

//deklaracje funkcji zewn�trznych
extern unsigned char StartI2C0(unsigned char chAdr, unsigned char chSubAdr, unsigned char *chData, unsigned char chLenWr, unsigned char chLenRd);
