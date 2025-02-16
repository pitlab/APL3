//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa magnetometru MMC3416xPJ na magistrali I2C4
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <MMC3416x.h>
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"


ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chDaneMagMMC[6]);	//I2C4 współpracuje z BDMA a on ogarnia tylko SRAM4

extern I2C_HandleTypeDef hi2c4;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern volatile unia_wymianyCM4_t uDaneCM4;

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika MMC34160PJ
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	chDaneMagMMC[0] = PMMC3416_PRODUCT_ID;
	chErr = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chDaneMagMMC, 1, 2);	//wyślij polecenie odczytu rejestru identyfikacyjnego
	if (!chErr)
	{
		chErr =  HAL_I2C_Master_Receive(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 1, 2);		//odczytaj dane
		if (!chErr)
		{
			if (chDaneMagMMC[0] == 0x06)
			{
				chDaneMagMMC[0] = PMMC3416_INT_CTRL0;
				chDaneMagMMC[1] = (0 << 0) |	//TM Take measurement, set ‘1’ will initiate measurement
								  (1 << 1) |	//Continuous Measurement Mode On
								  (3 << 2) |	//CM Freq0..1 How often the chip will take measurements in Continuous Measurement Mode: 0=1,5Hz, 1=13Hz, 2=25Hz, 3=50Hz
								  (0 << 4) |	//No Boost. Setting this bit high will disable the charge pump and cause the storage capacitor to be charged off VDD.
								  (0 << 5) |	//SET
								  (0 << 6) |	//RESET
								  (0 << 7);		//Refill Cap Writing “1” will recharge the capacitor at CAP pin, it is requested to be issued before SET/RESET command.
				chErr = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chDaneMagMMC, 2, 2);	//wyślij polecenie wykonania pomiaru
				if (!chErr)
				{
					uDaneCM4.dane.nZainicjowano |= INIT_MMC34160;
					chErr = StartujOdczytMMC3416x();
				}
			}
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Startuje cykliczny pomiar magnetometru MMC3416xPJ w trybie wyzwalanym ręcznie
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 420us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujPomiarMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
	{
		chDaneMagMMC[0] = PMMC3416_INT_CTRL0;
		chDaneMagMMC[1] = (1 << 0) |	//TM Take measurement, set ‘1’ will initiate measurement
						 (0 << 1) |	//Continuous Measurement Mode On
						 (0 << 2) |	//CM Freq0..1 How often the chip will take measurements in Continuous Measurement Mode: 0=1,5Hz, 1=13Hz, 2=25Hz, 3=50Hz
						 (0 << 4) |	//No Boost. Setting this bit high will disable the charge pump and cause the storage capacitor to be charged off VDD.
						 (0 << 5) |	//SET
						 (0 << 6) |	//RESET
						 (0 << 7);	//Refill Cap Writing “1” will recharge the capacitor at CAP pin, it is requested to be issued before SET/RESET command.
		chErr = HAL_I2C_Master_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chDaneMagMMC, 2);	//wyślij polecenie wykonania pomiaru
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje odczyt danych z magnetometru HMC5883
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 620us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujOdczytMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;
	extern uint8_t chOdczytywanyMagnetometr;	//zmienna wskazuje który magnetometr jest odczytywany: MAG_MMC lub MAG_IIS

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
	{
		chOdczytywanyMagnetometr = MAG_MMC;
		chDaneMagMMC[0] = PMMC3416_XOUT_L;
		chErr = HAL_I2C_Master_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chDaneMagMMC, 1);	//wyślij polecenie odczytu wszystkich pomiarów
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna odczyt danych pomiarowych
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 1,4ms przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 6);		//odczytaj dane
	else
		chErr = InicjujMMC3416x();	//wykonaj inicjalizację
	return chErr;
}


