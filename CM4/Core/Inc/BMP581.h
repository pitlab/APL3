

#ifndef INC_BMP581_H_
#define INC_BMP581_H_
#include "SysDefCM4.h"



//definicje poleceń czujnika BMP581
#define BMP5_REG_CHIP_ID      		0x01
#define BMP5_REG_REV_ID             0x02
#define BMP5_REG_CHIP_STATUS        0x11
#define BMP5_REG_DRIVE_CONFIG       0x13
#define BMP5_REG_INT_CONFIG         0x14
#define BMP5_REG_INT_SOURCE         0x15
#define BMP5_REG_FIFO_CONFIG        0x16
#define BMP5_REG_FIFO_COUNT         0x17
#define BMP5_REG_FIFO_SEL           0x18
#define BMP5_REG_TEMP_DATA_XLSB     0x1D
#define BMP5_REG_TEMP_DATA_LSB      0x1E
#define BMP5_REG_TEMP_DATA_MSB      0x1F
#define BMP5_REG_PRESS_DATA_XLSB    0x20
#define BMP5_REG_PRESS_DATA_LSB     0x21
#define BMP5_REG_PRESS_DATA_MSB     0x22
#define BMP5_REG_INT_STATUS         0x27
#define BMP5_REG_STATUS             0x28
#define BMP5_REG_FIFO_DATA          0x29
#define BMP5_REG_NVM_ADDR           0x2B
#define BMP5_REG_NVM_DATA_LSB       0x2C
#define BMP5_REG_NVM_DATA_MSB       0x2D
#define BMP5_REG_DSP_CONFIG         0x30
#define BMP5_REG_DSP_IIR            0x31
#define BMP5_REG_OOR_THR_P_LSB      0x32
#define BMP5_REG_OOR_THR_P_MSB      0x33
#define BMP5_REG_OOR_RANGE          0x34
#define BMP5_REG_OOR_CONFIG         0x35
#define BMP5_REG_OSR_CONFIG         0x36
#define BMP5_REG_ODR_CONFIG         0x37
#define BMP5_REG_OSR_EFF            0x38
#define BMP5_REG_CMD                0x7E



//definicje funkcji
uint8_t InicjujBMP581(void);
uint8_t ObslugaBMP581(void);

#endif /* INC_BMP581_H_ */
