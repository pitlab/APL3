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
uint8_t chSekwencjaPomiaruMMC;
extern uint8_t chOdczytywanyMagnetometr;	//zmienna wskazuje który magnetometr jest odczytywany: MAG_MMC lub MAG_IIS
uint8_t chLicznikOczekiwania;
int16_t sPomiarMMCH[3], sPomiarMMCL[3];	//wyniki pomiarów dla dodatniego i ujemnego namagnesowania czujnika

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
			if (chDaneMagMMC[0] == 0x06)	//czy zgadza się ID
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
				if (chErr)
					return chErr;

				chDaneMagMMC[0] = PMMC3416_INT_CTRL1;
				chDaneMagMMC[1] = (0 << 0) |	//BW0..1 Output resolution:	0=16 bits, 7.92 mS; 1=16 bits, 4.08 mS; 2=14 bits, 2.16 mS; 3= 12 bits, 1.20 mS
								  (0 << 2) |	//X-inhibit - Factory-use Register
								  (0 << 3) |	//Y-inhibit - Factory-use Register
								  (0 << 4) |	//Z-inhibit - Factory-use Register
								  (0 << 5) |	//ST_XYZ Selftest check, write “1” to this bit and execute a TM command, after TM is completed the result can be read as bit ST_XYZ_OK.
								  (0 << 6) |	//Temp_tst - Factory-use Register
								  (0 << 7);		//SW_RST Writing “1”will cause the part to reset, similar to power-up. It will clear all registers and also re-read OTP as part of its startup routine.
				chErr = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chDaneMagMMC, 2, 2);	//wyślij polecenie wykonania pomiaru
				if (!chErr)
					uDaneCM4.dane.nZainicjowano |= INIT_MMC34160;
			}
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj jeden element sekwencji potrzebnych do uzyskania pomiaru
// Najlepszy wynik wedlug dokumentacji uzyskuje się wykonując sekwencję: SET, MEASUREMENT, RESET, MEASUREMENT
// Cząstkowe wyniki oprocz natężenia pola zawierają offset. Następnie cząstkowe pomiary należy odjąć od siebie i podzielić przez 2
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania: pełna pętla zajmuje 170ms -> 5,88Hz
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaMMC3416x(void)
{
	uint8_t chErr = ERR_OK;

	switch (chSekwencjaPomiaruMMC)
	{
	case SPMMC3416_REFIL_SET:		//wyślij polecenie rozpoczęcia ładowania kondensatora do polecenia SET
		if ((uDaneCM4.dane.nZainicjowano & INIT_MMC34160) != INIT_MMC34160)
		{
			chErr = InicjujMMC3416x();
			chSekwencjaPomiaruMMC--;
		}
		//tutaj nie ma break, tylko jeżeli magnetometr jest zainicjowany to wykonaj dalsze polecenia
	case SPMMC3416_REFIL_RESET:		//wyślij polecenie rozpoczęcia ładowania kondensatora do polecenia RESET
		chErr = PolecenieMMC3416x(POL_REFILL);
		chLicznikOczekiwania = 10;
		break;

	case SPMMC3416_CZEKAJ_REFSET:	//czekaj 50ms na naładowanie
	case SPMMC3416_CZEKAJ_REFRES:	//czekaj 50ms na naładowanie
		chLicznikOczekiwania--;
		if (chLicznikOczekiwania)		//dopóki nie upłynie czas oczekiwania nie wychodź z tego etapu
			chSekwencjaPomiaruMMC--;
		break;

	case SPMMC3416_SET:				//wyślij polecenie SET
		chErr = PolecenieMMC3416x(POL_SET);
		break;

	case SPMMC3416_START_POM_HP:	//wyślij polecenie wykonania pomiaru H+
	case SPMMC3416_START_POM_HM:	//wyślij polecenie wykonania pomiaru H-
		chErr = PolecenieMMC3416x(POL_TM);
		break;

	case SPMMC3416_START_STAT_P:	//wyślij polecenie odczytania statusu
	case SPMMC3416_START_STAT_M:	//wyślij polecenie odczytania statusu
		StartujOdczytRejestruMMC3416x(PMMC3416_STATUS);
		break;

	case SPMMC3416_CZYT_STAT_P:		//odczytaj status i sprawdź gotowość pomiaru
	case SPMMC3416_CZYT_STAT_M:		//odczytaj status i sprawdź gotowość pomiaru
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 1);		//odczytaj dane
		chOdczytywanyMagnetometr = 0;	//nie interpretuj odczytanych danych jako wyniku pomiaru
		break;

	case SPMMC3416_START_CZYT_HP:	//wyślij polecenie odczytu pomiaru H+
	case SPMMC3416_START_CZYT_HM:	//wyślij polecenie odczytu pomiaru H-
		if (chDaneMagMMC[0] & 0x01)	//sprawdź odczytany status czy ustawiony jest bit "Meas Done"
			chErr = StartujOdczytRejestruMMC3416x(PMMC3416_XOUT_L);
		else
			chSekwencjaPomiaruMMC -= 3;	//jeżeli niegotowy to wróć do odczytu statusu
		break;

	case SPMMC3416_CZYTAJ_HP:		//odczytaj pomiar H+
	case SPMMC3416_CZYTAJ_HM:		//odczytaj pomiar H-
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 6);		//odczytaj dane
		chOdczytywanyMagnetometr = MAG_MMC;	//identyfikuje układ w callbacku odczytu danych
		break;

	case SPMMC3416_RESET:			//wyślij polecenie RESET
		PolecenieMMC3416x(POL_RESET);
		break;

	default:
	}
	chSekwencjaPomiaruMMC++;
	chSekwencjaPomiaruMMC &= 0x0F;

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje odczyt danych z magnetometru HMC5883
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 620us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujOdczytRejestruMMC3416x(uint8_t chRejestr)
{
	uint8_t chErr = ERR_BRAK_MMC34160;
	extern uint8_t chOdczytywanyMagnetometr;	//zmienna wskazuje który magnetometr jest odczytywany: MAG_MMC lub MAG_IIS

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
	{
		chOdczytywanyMagnetometr = MAG_MMC;
		chDaneMagMMC[0] = chRejestr;	//PMMC3416_XOUT_L;
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



////////////////////////////////////////////////////////////////////////////////
// Inicjuje odczyt statusu z magnetometru HMC5883
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 620us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajStatusMMC3416x(void)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
		chErr = HAL_I2C_Master_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 1);
	return chErr;
}




////////////////////////////////////////////////////////////////////////////////
// Wysyła polecenie REFIL napełnienia kondensatora dla wykonania polecen SET i RESET
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 420us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t PolecenieMMC3416x(uint8_t chPolecenie)
{
	uint8_t chErr = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
	{
		chDaneMagMMC[0] = PMMC3416_INT_CTRL0;
		chDaneMagMMC[1] = chPolecenie |	//bit zdefiniowany w pliku h
						 (0 << 1) |	//Continuous Measurement Mode On
						 (3 << 2);	//CM Freq0..1 How often the chip will take measurements in Continuous Measurement Mode: 0=1,5Hz, 1=13Hz, 2=25Hz, 3=50Hz
		chErr = HAL_I2C_Master_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chDaneMagMMC, 2);	//wyślij polecenie wykonania pomiaru
	}
	return chErr;
}
