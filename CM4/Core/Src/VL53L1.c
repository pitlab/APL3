//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa dalmierzy światła odbitego rodziny VL53Cx
//
// (c) Pit Lab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <VL53L1.h>
#include "WymianaCM4.h"
#include "vl53l1cb.h"
#include "vl53l1_api.h"

static VL53L1CB_Object_t VL53L1CB_Dev;
extern I2C_HandleTypeDef hi2c3;
extern volatile unia_wymianyCM4_t uDaneCM4;




////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujVL53L1(void)
{
	uint8_t cBłąd = BLAD_OK;
	//uint8_t cModelID[3];

	//cBłąd = HAL_I2C_Mem_Read(&hi2c3, 0x52, 0x010F, I2C_MEMADD_SIZE_16BIT, cModelID, 3, 10);

	VL53L1CB_Dev.IO.Address 	= 0x52;
	VL53L1CB_Dev.IO.Init		= TOF_I2C_Init;
	VL53L1CB_Dev.IO.DeInit		= TOF_I2C_DeInit;
	VL53L1CB_Dev.IO.WriteReg	= TOF_WriteReg;
	VL53L1CB_Dev.IO.ReadReg		= TOF_ReadReg;
	VL53L1CB_Dev.IO.GetTick		= TOF_GetTick;

	uint32_t nStatus = VL53L1CB_Init(&VL53L1CB_Dev);
	if (nStatus)
		return BLAD_BRAK_CZUJNIKA;


	//nStatus = VL53L1X_SetInterMeasurementPeriod();
	//nStatus = VL53l1X_SetOffset();

	return cBłąd;
}

//definicje funkcji obsługujących czujnik
int32_t TOF_I2C_Init(void)
{
	return VL53L1_ERROR_NONE;
}

int32_t TOF_I2C_DeInit(void)
{
	return VL53L1_ERROR_NONE;
}

int32_t TOF_GetTick(void)
{
	return (int32_t)HAL_GetTick();
}



////////////////////////////////////////////////////////////////////////////////
// Opakowanie na wywołanie funkcji zapisu do czujnika
// Na pierwszych 2 bajtach pData znajduje się adres rejestru, daje są dane do zapisu
// Parametry:
//  Reg - adres czujnika na magistrali (?)
//  *pData - wskaźnik na dane
//  Length - rozmiar danych
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
int32_t TOF_WriteReg(uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	uint8_t cBłąd = HAL_I2C_Master_Transmit(&hi2c3, Reg, pData, Length, TIMEOUT_VL53C1);
    return (cBłąd == HAL_OK) ? 0 : -1;
}


int32_t TOF_ReadReg(uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	uint8_t cBłąd = HAL_I2C_Master_Receive(&hi2c3, Reg, pData, Length, TIMEOUT_VL53C1);
    return (cBłąd == HAL_OK) ? 0 : -1;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika VL53L1 do wywołania w wyższej warstwie
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaVL53L1(void)
{
	uint8_t cBłąd = BLAD_OK;
	uint8_t cPomiarGotowy;
	VL53L1_RangingMeasurementData_t RMData;

	//sprawdź czy czujnik jest zainicjowany a jeżeli nie, to zainicjuj
	if ((uDaneCM4.dane.nZainicjowano & INIT_VL53L1) != INIT_VL53L1)	//jeżeli czujnik nie jest zainicjowany
	{
		cBłąd = InicjujVL53L1();
		if (cBłąd)
			return cBłąd;
		uDaneCM4.dane.nZainicjowano |= INIT_VL53L1;
		cBłąd = VL53L1_StartMeasurement(&VL53L1CB_Dev);
	}

	//sprawdź czy pomiar jest gotowy. Jeżeli tak, to odczytaj go
	cBłąd = VL53L1_GetMeasurementDataReady(&VL53L1CB_Dev, &cPomiarGotowy);
	if ((cBłąd == BLAD_OK) && cPomiarGotowy)
	{
		cPomiarGotowy = 0;

		cBłąd = VL53L1_GetRangingMeasurementData(&VL53L1CB_Dev, &RMData);
		cBłąd = VL53L1_ClearInterruptAndStartMeasurement(&VL53L1CB_Dev);
	}
	else
	if (cBłąd == VL53L1_ERROR_CONTROL_INTERFACE)
	{
		uint8_t cDane[2];
		cBłąd = HAL_I2C_Master_Receive(&hi2c3, 0x52, cDane, 2, TIMEOUT_VL53C1);
	}

	return cBłąd;
}
