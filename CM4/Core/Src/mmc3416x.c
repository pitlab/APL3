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
ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chStatusMagMMC);	//I2C4 współpracuje z BDMA a on ogarnia tylko SRAM4
ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM4")))	chPolWychMagMMC[3]);	//polecenia i dane wysyłane do czujnika w sobnej zmiennej aby nie kolidowały z danymi przychodzącymi


extern I2C_HandleTypeDef hi2c4;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern volatile unia_wymianyCM4_t uDaneCM4;
uint8_t chSekwencjaPomiaruMMC;
extern volatile uint8_t chCzujnikOdczytywanyNaI2CInt;	//identyfikator czujnika obsługiwanego na wewnętrznej magistrali I2C: MAG_MMC lub MAG_IIS
extern volatile uint8_t chCzujnikZapisywanyNaI2CInt;
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

	chPolWychMagMMC[0] = PMMC3416_PRODUCT_ID;
	chErr = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 1, TOUT_I2C4_2B);	//wyślij polecenie odczytu rejestru identyfikacyjnego
	if (!chErr)
	{
		chErr =  HAL_I2C_Master_Receive(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 1, TOUT_I2C4_2B);		//odczytaj dane
		if (!chErr)
		{
			if (chDaneMagMMC[0] == 0x06)	//czy zgadza się ID
			{
				chPolWychMagMMC[0] = PMMC3416_INT_CTRL0;
				chPolWychMagMMC[1] = (0 << 0) |	//TM Take measurement, set ‘1’ will initiate measurement
								  (1 << 1) |	//Continuous Measurement Mode On
								  (3 << 2) |	//CM Freq0..1 How often the chip will take measurements in Continuous Measurement Mode: 0=1,5Hz, 1=13Hz, 2=25Hz, 3=50Hz
								  (0 << 4) |	//No Boost. Setting this bit high will disable the charge pump and cause the storage capacitor to be charged off VDD.
								  (0 << 5) |	//SET
								  (0 << 6) |	//RESET
								  (0 << 7);		//Refill Cap Writing “1” will recharge the capacitor at CAP pin, it is requested to be issued before SET/RESET command.
				chErr = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 2, TOUT_I2C4_2B);	//wyślij polecenie wykonania pomiaru
				if (chErr)
					return chErr;

				chPolWychMagMMC[0] = PMMC3416_INT_CTRL1;
				chPolWychMagMMC[1] = (0 << 0) |	//BW0..1 Output resolution:	0=16 bits, 7.92 mS; 1=16 bits, 4.08 mS; 2=14 bits, 2.16 mS; 3= 12 bits, 1.20 mS
								  (0 << 2) |	//X-inhibit - Factory-use Register
								  (0 << 3) |	//Y-inhibit - Factory-use Register
								  (0 << 4) |	//Z-inhibit - Factory-use Register
								  (0 << 5) |	//ST_XYZ Selftest check, write “1” to this bit and execute a TM command, after TM is completed the result can be read as bit ST_XYZ_OK.
								  (0 << 6) |	//Temp_tst - Factory-use Register
								  (0 << 7);		//SW_RST Writing “1”will cause the part to reset, similar to power-up. It will clear all registers and also re-read OTP as part of its startup routine.
				chErr = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 2, TOUT_I2C4_2B);	//wyślij polecenie wykonania pomiaru
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

	if ((uDaneCM4.dane.nZainicjowano & INIT_MMC34160) != INIT_MMC34160)
	{
		chErr = InicjujMMC3416x();
		return chErr;
	}

	switch (chSekwencjaPomiaruMMC)
	{
	case SPMMC3416_REFIL_SET:		//wyślij polecenie rozpoczęcia ładowania kondensatora do polecenia SET
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
		chPolWychMagMMC[0] = PMMC3416_STATUS;
		chErr = HAL_I2C_Master_Seq_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 1, I2C_FIRST_FRAME);	//wyślij polecenie odczytu statusu nie kończąc transferu STOP-em
		chCzujnikZapisywanyNaI2CInt = MAG_MMC_STATUS;	//po zakończeniu uruchom drugą część transmisji dzielonej
		break;

	/*case SPMMC3416_CZYT_STAT_P:		//odczytaj status i sprawdź gotowość pomiaru
	case SPMMC3416_CZYT_STAT_M:		//odczytaj status i sprawdź gotowość pomiaru
		chErr = HAL_I2C_Master_Seq_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, &chStatusMagMMC, 1, I2C_LAST_FRAME);		//odczytaj dane i zakończ STOP
		chCzujnikOdczytywanyNaI2CInt = 0;	//nie interpretuj odczytanych danych jako wyniku pomiaru
		break;*/

	case SPMMC3416_START_CZYT_HP:	//wyślij polecenie odczytu pomiaru H+
	case SPMMC3416_START_CZYT_HM:	//wyślij polecenie odczytu pomiaru H-
		if (chStatusMagMMC & 0x01)	//sprawdź odczytany status czy ustawiony jest bit "Meas Done"
		{
			chPolWychMagMMC[0] = PMMC3416_XOUT_L;
			chErr = HAL_I2C_Master_Seq_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 1, I2C_FIRST_FRAME);	//wyślij polecenie odczytu danych nie kończąc transferu STOP-em
			chCzujnikZapisywanyNaI2CInt = MAG_MMC;	//po zakończeniu uruchom drugą część transmisji dzielonej
		}
		else
			chSekwencjaPomiaruMMC -= 3;	//jeżeli niegotowy to wróć do odczytu statusu
		break;

	/*case SPMMC3416_CZYTAJ_HP:		//odczytaj pomiar H+
	case SPMMC3416_CZYTAJ_HM:		//odczytaj pomiar H-
		chErr = HAL_I2C_Master_Seq_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 6, I2C_LAST_FRAME);		//odczytaj dane i zakończ STOP
		chCzujnikOdczytywanyNaI2CInt = MAG_MMC;		//w callbacku interpretuj odczytane dane jako pomiar magnetometru MMC
		break; */

	case SPMMC3416_RESET:			//wyślij polecenie RESET
		PolecenieMMC3416x(POL_RESET);
		break;

	default:
	}
	chSekwencjaPomiaruMMC++;
	//chSekwencjaPomiaruMMC &= 0x0F;
	if (chSekwencjaPomiaruMMC >= SPMMC3416_LICZ_OPERACJI)
		chSekwencjaPomiaruMMC = 0;

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
		chPolWychMagMMC[0] = PMMC3416_INT_CTRL0;
		chPolWychMagMMC[1] = chPolecenie |	//bit zdefiniowany w pliku h
						 (0 << 1) |	//Continuous Measurement Mode On
						 (3 << 2);	//CM Freq0..1 How often the chip will take measurements in Continuous Measurement Mode: 0=1,5Hz, 1=13Hz, 2=25Hz, 3=50Hz
		chErr = HAL_I2C_Master_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 2);	//wyślij polecenie wykonania pomiaru
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Polecenie będące drugą częścią sekwencji podzielonej operacji odczytu statusu.
// Będzie uruchomione w callbacku zakończenia operacji wysłania polecenia odczytu statusu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t MagMMC_CzytajStatus(void)
{
	chCzujnikOdczytywanyNaI2CInt = 0;	//nie interpretuj odczytanych danych jako wyniku pomiaru
	return HAL_I2C_Master_Seq_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, &chStatusMagMMC, 1, I2C_LAST_FRAME);		//odczytaj dane i zakończ STOP
}



////////////////////////////////////////////////////////////////////////////////
// Polecenie będące drugą częścią sekwencji podzielonej operacji odczytu dnych.
// Będzie uruchomione w callbacku zakończenia operacji wysłania polecenia odczytu danych
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t MagMMC_CzytajDane(void)
{
	chCzujnikOdczytywanyNaI2CInt = MAG_MMC;		//w callbacku interpretuj odczytane dane jako pomiar magnetometru MMC
	return HAL_I2C_Master_Seq_Receive_DMA(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 6, I2C_LAST_FRAME);		//odczytaj dane i zakończ STOP
}

