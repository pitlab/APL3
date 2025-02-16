

#ifndef INC_BMP581_H_
#define INC_BMP581_H_
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
#define PBMP5_INT_STATUS		0x27
#define PBMP5_STATUS			0x28

#define PBMP5_CMD				0x7E




//definicje funkcji
uint8_t InicjujBMP581(void);
uint8_t ObslugaBMP581(void);

#endif /* INC_BMP581_H_ */
