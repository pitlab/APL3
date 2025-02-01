
#include "sys_def_CM4.h"


//definicje Poleceń czujnikaBMP581
#define PBMP5_CHIP_ID			0x01	//Reset value 0x50
#define PBMP5_REV_ID			0x02	//Reset value 0x32
#define PBMP5_CHIP_STATUS		0x11
#define PBMP5_DRIVE_CONFIG		0x13
#define PBMP5_TEMP_DATA_XLSB	0x1D
#define PBMP5_TEMP_DATA_LSB		0x1E
#define PBMP5_TEMP_DATA_MSB		0x1F
#define PBMP5_PRESS_DATA_XLSB	0x20
#define PBMP5_PRESS_DATA_LSB	0x21
#define PBMP5_PRESS_DATA_MSB	0x22
#define PBMP5_CMD				0x7E


#define PBMP5_READ				0x80
#define PBMP5_WRITE				0x00

//definicje funkcji
unsigned char BMP085_StartTempConversion(void);
float BMP085_ReadTemperature(void);
unsigned char BMP085_StartPresConversion(unsigned char chOversampling);
float BMP085_ReadPressure(void);

uint8_t InicjujBMP581(void);
uint8_t BMP581_Read8bit(unsigned char chAdres);
int32_t BMP581_Read24bit(unsigned char chAdres);
uint8_t ObslugaBMP581(void);

//deklaracje funkcji zewn�trznych
extern unsigned char StartI2C0(unsigned char chAdr, unsigned char chSubAdr, unsigned char *chData, unsigned char chLenWr, unsigned char chLenRd);
