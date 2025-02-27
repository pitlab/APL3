//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Definicje obsługi pamięci FRAM FM25V02 32KB
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
//Pamięć pracuje w trybach SPI 0 i 3 z 8-bitowymi transferami.
//Max zegar to 40MHz

#include "sys_def_CM4.h"

//opcody operacji
#define FRAM_WREN	0x06	//Set Write Enable Latch
#define FRAM_WRDI	0x04	//Write Disable
#define FRAM_RDSR	0x05	//Read Status Register
#define FRAM_WRSR	0x01	//Write Status Register
#define FRAM_READ	0x03	//Read Memory Data
#define FRAM_WRITE	0x02	//Write Memory Data
#define FRAM_FSTRD	0x0B	//Fast Read Memory Data
#define FRAM_SLEEP	0xB9	//Enter sleep mode
#define FRAM_RDID	0x9F	//Read device ID

#define FRAM_V02A_ID	0x7F7F7F7F7F7FC22208


//definicje funkcji sprzętowych
uint8_t CzytajStatusFRAM(void);
void ZapiszStatusFRAM(uint8_t chStatus);
void CzytajIdFRAM(uint8_t* chDaneID);
void ZapiszFRAM(unsigned short sAdres, uint8_t chDane);
uint8_t CzytajFRAM(uint16_t sAdres);
void CzytajBuforFRAM(uint16_t sAdres, uint8_t* chDane, uint16_t sIlosc);
void ZapiszBuforFRAM(uint16_t sAdres, uint8_t* chDane, uint16_t sIlosc);


//definicje funkcji wtórnych
float FramDataReadFloat(unsigned short sAddress);
void FramDataWriteFloat(unsigned short sAddress, float fData);
unsigned char FramDataReadFloatValid(unsigned short sAddress, float *fValue, float fValMin, float fValMax, float fValDef, unsigned char chErrCode);

