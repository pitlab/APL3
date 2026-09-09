//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa dalmierza światła odbitego VL53C1
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
uint8_t cEtapPomiaruVL53L1;
extern uint8_t cZakonczonoTransmisjeI2C;
extern uint8_t cCzujnikZapisywanyNaI2CExt, cCzujnikOdczytywanyNaI2CExt;
static uint8_t cLicznikPróbInicjalizacji = MAX_PROB_INICJALIZACJI;



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika VL53L1
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujVL53L1(void)
{
	uint8_t cBłąd = BLAD_OK;
	uint32_t nStatus;
	//VL53L1_DetectionConfig_t stConfig;

	VL53L1CB_Dev.IO.Address 	= 0x52;
	VL53L1CB_Dev.IO.Init		= TOF_I2C_Init;
	VL53L1CB_Dev.IO.DeInit		= TOF_I2C_DeInit;
	VL53L1CB_Dev.IO.WriteReg	= TOF_WriteReg;
	VL53L1CB_Dev.IO.ReadReg		= TOF_ReadReg;
	VL53L1CB_Dev.IO.GetTick		= TOF_GetTick;

	nStatus = VL53L1CB_Init(&VL53L1CB_Dev);
	if (nStatus)
		cBłąd = BLAD_BRAK_CZUJNIKA;

	nStatus = VL53L1_SetDistanceMode(&VL53L1CB_Dev, VL53L1_DISTANCEMODE_LONG);
	if (nStatus)
		cBłąd = BLAD_BRAK_CZUJNIKA;

	nStatus = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&VL53L1CB_Dev, 33000);
	if (nStatus)
		cBłąd = BLAD_BRAK_CZUJNIKA;

	/*stConfig.DetectionMode = 1;
	stConfig.Distance.CrossMode = 3;
	stConfig.IntrNoTarget = 0;
	stConfig.Distance.High = 3000;
	stConfig.Distance.Low = 50;
	nStatus = VL53L1_SetThresholdConfig(&VL53L1CB_Dev, &stConfig);*/

	cEtapPomiaruVL53L1 = EPVL53_SPRAWDZ_CZY_ZAINICJOWANY;
	return cBłąd;
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
	uint8_t cPomiarGotowy = 0;
	VL53L1_RangingMeasurementData_t RMData;

	if (uDaneCM4.dane.nBrakCzujnika & INIT_VL53L1)
		return BLAD_OK;

	//jeżeli nie zaończyła się poprzednia transmisja to nie zaczynaj nastepnej
	//if (cCzujnikZapisywanyNaI2CExt)
		//return BLAD_ZA_KROTKI_CZAS;

	switch (cEtapPomiaruVL53L1)
	{
	case EPVL53_SPRAWDZ_CZY_ZAINICJOWANY:
		if ((uDaneCM4.dane.nZainicjowano & INIT_VL53L1) != INIT_VL53L1)	//jeżeli czujnik nie jest zainicjowany
		{
			cBłąd = InicjujVL53L1();
			if (cBłąd)
			{
				cLicznikPróbInicjalizacji--;
				if (!cLicznikPróbInicjalizacji)
					uDaneCM4.dane.nBrakCzujnika |= INIT_VL53L1;
				return cBłąd;
			}
			uDaneCM4.dane.nZainicjowano |= INIT_VL53L1;
		}
		cBłąd = VL53L1_StartMeasurement(&VL53L1CB_Dev);
		if (cBłąd == BLAD_OK)
		{
			cEtapPomiaruVL53L1 = EPVL53_SPRAWDZ_CZY_POMIAR_GOTOWY;
			cEtapPomiaruVL53L1++;
		}
		break;

	case EPVL53_SPRAWDZ_CZY_POMIAR_GOTOWY:	//Transmisja trwa 400us Pierwszy jest zapis, potem odczyt
		cBłąd = VL53L1_GetMeasurementDataReady(&VL53L1CB_Dev, &cPomiarGotowy);
		if (cBłąd == BLAD_OK)
		{
			if (cPomiarGotowy)
				cEtapPomiaruVL53L1++;
		}
		break;

	case EPVL53_ROZPOCZNIJ_ODCZYT_POMIARU:	//trwa6,2ms zapis 2 bajtów, potem odczyt kilkudziesieciu
		cBłąd = VL53L1_GetRangingMeasurementData(&VL53L1CB_Dev, &RMData);
		if (cBłąd == BLAD_OK)
		{
			uDaneCM4.dane.stTOF.cStatusPomiaru = RMData.RangeStatus;
			uDaneCM4.dane.stTOF.cNowyPomiar = RMData.StreamCount;
			uDaneCM4.dane.stTOF.sOdległość = RMData.RangeMilliMeter;
			uDaneCM4.dane.stTOF.fSigma = (float)RMData.SigmaMilliMeter / 65536;
			uDaneCM4.dane.stTOF.fNatężenieTła = (float)RMData.AmbientRateRtnMegaCps / 65536;
			uDaneCM4.dane.stTOF.fReflektancjaCelu = (float)RMData.SignalRateRtnMegaCps / 65536;
			cEtapPomiaruVL53L1++;
		}
		else
			cEtapPomiaruVL53L1 = EPVL53_SPRAWDZ_CZY_ZAINICJOWANY;
		cPomiarGotowy = 0;
		break;

	case EPVL53_SPRAWDZ_CZY_ODCZYT_ZAKONCZONY:
		cEtapPomiaruVL53L1++;
		break;

	case EPVL53_CZYSZCZENIE_I_RESTART_POMIARU:	//pojedyńcza transmisja zapisu trwa 5,3ms
		cBłąd = VL53L1_ClearInterruptAndStartMeasurement(&VL53L1CB_Dev);
		cEtapPomiaruVL53L1++;
		break;

	case EPVL53_CZY_KONIEC_TRANSMISJI_I_RESTARTU:
//		if (cZakonczonoTransmisjeI2C)
			cEtapPomiaruVL53L1 = EPVL53_SPRAWDZ_CZY_POMIAR_GOTOWY;
		break;
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Opakowanie na wywołanie funkcji inicjalizacji czujnika - puste
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
int32_t TOF_I2C_Init(void)
{
	return VL53L1_ERROR_NONE;
}



////////////////////////////////////////////////////////////////////////////////
// Opakowanie na wywołanie funkcji deinicjalizacji czujnika - puste
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
int32_t TOF_I2C_DeInit(void)
{
	return VL53L1_ERROR_NONE;
}



////////////////////////////////////////////////////////////////////////////////
// Opakowanie na wywołanie funkcji pomiaru czasu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
int32_t TOF_GetTick(void)
{
	return (int32_t)HAL_GetTick();
}



////////////////////////////////////////////////////////////////////////////////
// Opakowanie na wywołanie funkcji zapisu do czujnika
// Na pierwszych 2 bajtach pData znajduje się adres rejestru, daje są dane do zapisu
// Parametry:
//  Reg - adres czujnika na magistrali
//  *pData - wskaźnik na dane
//  Length - rozmiar danych
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
int32_t TOF_WriteReg(uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	uint8_t cBłąd;

/*	if (cEtapPomiaruVL53L1 == EPVL53_CZYSZCZENIE_I_RESTART_POMIARU)
	{
		cCzujnikZapisywanyNaI2CExt = TOF_VL53L1;
		cBłąd = HAL_I2C_Master_Transmit_DMA(&hi2c3, Reg, pData, Length);
	}
	else*/
		cBłąd = HAL_I2C_Master_Transmit(&hi2c3, Reg, pData, Length, TIMEOUT_VL53C1);
    return (cBłąd == HAL_OK) ? 0 : -1;
}



////////////////////////////////////////////////////////////////////////////////
// Opakowanie na wywołanie funkcji odczytu z czujnika
// Na pierwszych 2 bajtach pData znajduje się adres rejestru
// Parametry:
//  Reg - adres czujnika na magistrali
//  *pData - wskaźnik na dane
//  Length - rozmiar danych
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
int32_t TOF_ReadReg(uint16_t Reg, uint8_t *pData, uint16_t Length)
{
	uint8_t cBłąd = HAL_I2C_Master_Receive(&hi2c3, Reg, pData, Length, TIMEOUT_VL53C1);
    return (cBłąd == HAL_OK) ? 0 : -1;
}

