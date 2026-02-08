//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa magnetometru MMC3416xPJ na magistrali I2C4
//
// (c) Pit Lab 2025-26
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <MMC3416x.h>
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"
#include "fram.h"
#include "konfig_fram.h"



//Zmienne przesyłane przez I2C4 we współpracy z BDMA a on ma dostęp tylko SRAM4
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4")))	chDaneMagMMC[6];		//dane pomiarowe magnetometru MMC
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4")))	chStatusMagMMC;		//ststus magnetometru MMC
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM4")))	chPolWychMagMMC[3];	//polecenia i dane wysyłane do magnetometru MMC w osobnej zmiennej aby nie kolidowały z danymi przychodzącymi
extern I2C_HandleTypeDef hi2c4;
extern DMA_HandleTypeDef hdma_i2c4_rx;
extern DMA_HandleTypeDef hdma_i2c4_tx;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chSekwencjaPomiaruMMC = 0;	//bieżąca sekwencja wykonywania operacji na czujniku MMC. W trakcie inicjalizacji pełni rolę licznika prób inicjalziacji
uint8_t chRodzajPomiaruMMC;		//rodzaj pomiaru: H+ po poleceniu SET, lub H- po poleceniu RESET
extern volatile uint8_t chCzujnikOdczytywanyNaI2CInt;	//identyfikator czujnika obsługiwanego na wewnętrznej magistrali I2C: MAG_MMC lub MAG_IIS
extern volatile uint8_t chCzujnikZapisywanyNaI2CInt;
uint8_t chLicznikOczekiwania;
int16_t sPomiarMMCH[3], sPomiarMMCL[3];	//wyniki pomiarów dla dodatniego i ujemnego namagnesowania czujnika
float fPrzesMagn2[3], fSkaloMagn2[3];



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika MMC34160PJ
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMMC3416x(void)
{
	uint8_t chBlad;

	chPolWychMagMMC[0] = PMMC3416_PRODUCT_ID;
	chBlad = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 1, TOUT_I2C4_2B);	//wyślij polecenie odczytu rejestru identyfikacyjnego
	if (!chBlad)
	{
		chBlad =  HAL_I2C_Master_Receive(&hi2c4, MMC34160_I2C_ADR + READ, chDaneMagMMC, 1, TOUT_I2C4_2B);		//odczytaj dane
		if (!chBlad)
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
				chBlad = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 2, TOUT_I2C4_2B);	//wyślij polecenie wykonania pomiaru
				if (chBlad)
					return chBlad;

				chPolWychMagMMC[0] = PMMC3416_INT_CTRL1;
				chPolWychMagMMC[1] = (0 << 0) |	//BW0..1 Output resolution:	0=16 bits, 7.92 mS; 1=16 bits, 4.08 mS; 2=14 bits, 2.16 mS; 3= 12 bits, 1.20 mS
								  (0 << 2) |	//X-inhibit - Factory-use Register
								  (0 << 3) |	//Y-inhibit - Factory-use Register
								  (0 << 4) |	//Z-inhibit - Factory-use Register
								  (0 << 5) |	//ST_XYZ Selftest check, write “1” to this bit and execute a TM command, after TM is completed the result can be read as bit ST_XYZ_OK.
								  (0 << 6) |	//Temp_tst - Factory-use Register
								  (0 << 7);		//SW_RST Writing “1”will cause the part to reset, similar to power-up. It will clear all registers and also re-read OTP as part of its startup routine.
				chBlad = HAL_I2C_Master_Transmit(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 2, TOUT_I2C4_2B);	//wyślij polecenie wykonania pomiaru
				if (!chBlad)
					return chBlad;

				for (uint16_t n=0; n<3; n++)
				{
					chBlad |= CzytajFramZWalidacja(FAH_MAGN2_PRZESX + 4*n, &fPrzesMagn2[n], VMIN_PRZES_MAGN, VMAX_PRZES_MAGN, VDOM_PRZES_MAGN, ERR_ZLA_KONFIG);
					chBlad |= CzytajFramZWalidacja(FAH_MAGN2_SKALOX + 4*n, &fSkaloMagn2[n], VMIN_SKALO_MAGN, VMAX_SKALO_MAGN, VDOM_SKALO_MAGN, ERR_ZLA_KONFIG);
				}
			}
			else
				chBlad = ERR_BRAK_MMC34160;
		}
	}
	return chBlad;
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
	uint8_t chBlad = BLAD_OK;

	//po MAX_PROB_INICJALIZACJI ustawiany jest bit braku czujnika. Taki czujnik nie jest dłużej obsługiwany
	if (uDaneCM4.dane.nBrakCzujnika & INIT_MMC34160)
		return ERR_BRAK_MMC34160;

	if ((uDaneCM4.dane.nZainicjowano & INIT_MMC34160) != INIT_MMC34160)
	{
		if (chSekwencjaPomiaruMMC < MAX_PROB_INICJALIZACJI)		//W trakcie inicjalizacji chSekwencjaPomiaruMMC pełni rolę licznika prób inicjalizacji
		{
			chSekwencjaPomiaruMMC++;
			chBlad = InicjujMMC3416x();
			if (chBlad == BLAD_OK)
			{
				uDaneCM4.dane.nZainicjowano |= INIT_MMC34160;
				chSekwencjaPomiaruMMC = 0;
			}
		}
		else
		{
			uDaneCM4.dane.nBrakCzujnika |= INIT_MMC34160;
			chBlad = ERR_BRAK_MMC34160;
		}
		return chBlad;
	}

	switch (chSekwencjaPomiaruMMC)
	{
	case SPMMC3416_REFIL_SET:		//wyślij polecenie rozpoczęcia ładowania kondensatora do polecenia SET
	case SPMMC3416_REFIL_RESET:		//wyślij polecenie rozpoczęcia ładowania kondensatora do polecenia RESET
		chBlad = PolecenieMMC3416x(POL_REFILL);
		chLicznikOczekiwania = 10;
		break;

	case SPMMC3416_CZEKAJ_REFSET:	//czekaj 50ms na naładowanie
	case SPMMC3416_CZEKAJ_REFRES:	//czekaj 50ms na naładowanie
		chLicznikOczekiwania--;
		if (chLicznikOczekiwania)		//dopóki nie upłynie czas oczekiwania nie wychodź z tego etapu
			chSekwencjaPomiaruMMC--;
		break;

	case SPMMC3416_SET:				//wyślij polecenie SET
		chBlad = PolecenieMMC3416x(POL_SET);
		chLicznikOczekiwania = CYKLI_PRZEMAGNSOWANIA_MMC;
		break;

	case SPMMC3416_START_POM_HP:	//wyślij polecenie wykonania pomiaru H+
	case SPMMC3416_START_POM_HM:	//wyślij polecenie wykonania pomiaru H-
		chRodzajPomiaruMMC = chSekwencjaPomiaruMMC;
		chBlad = PolecenieMMC3416x(POL_TM);
		break;

	case SPMMC3416_START_STAT_P:	//wyślij polecenie odczytania statusu
	case SPMMC3416_START_STAT_M:	//wyślij polecenie odczytania statusu
		chPolWychMagMMC[0] = PMMC3416_STATUS;
		chCzujnikZapisywanyNaI2CInt = MAG_MMC_STATUS;	//po zakończeniu uruchom drugą część transmisji dzielonej
		chBlad = HAL_I2C_Master_Seq_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 1, I2C_FIRST_FRAME);	//wyślij polecenie odczytu statusu nie kończąc transferu STOP-em
		break;

	case SPMMC3416_START_CZYT_HP:	//wyślij polecenie odczytu pomiaru H+
	case SPMMC3416_START_CZYT_HM:	//wyślij polecenie odczytu pomiaru H-
		if (chStatusMagMMC & 0x01)	//sprawdź odczytany status czy ustawiony jest bit "Meas Done"
		{
			chPolWychMagMMC[0] = PMMC3416_XOUT_L;
			chCzujnikZapisywanyNaI2CInt = MAG_MMC;	//po zakończeniu uruchom drugą część transmisji dzielonej
			chBlad = HAL_I2C_Master_Seq_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 1, I2C_FIRST_FRAME);	//wyślij polecenie odczytu danych nie kończąc transferu STOP-em
			chLicznikOczekiwania--;
			if (chLicznikOczekiwania)		//wykonaj serię pomiarów między przemagnesowaniami
				chSekwencjaPomiaruMMC -= 3;
		}
		else
			chSekwencjaPomiaruMMC -= 3;	//jeżeli niegotowy to wróć do odczytu statusu
		break;

	case SPMMC3416_RESET:			//wyślij polecenie RESET
		PolecenieMMC3416x(POL_RESET);
		chLicznikOczekiwania = CYKLI_PRZEMAGNSOWANIA_MMC;
		break;

	default:
	}
	chSekwencjaPomiaruMMC++;
	if (chSekwencjaPomiaruMMC >= SPMMC3416_LICZ_OPERACJI)
		chSekwencjaPomiaruMMC = 0;

	return chBlad;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła polecenie REFIL napełnienia kondensatora dla wykonania polecen SET i RESET
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 420us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t PolecenieMMC3416x(uint8_t chPolecenie)
{
	uint8_t chBlad = ERR_BRAK_MMC34160;

	if (uDaneCM4.dane.nZainicjowano & INIT_MMC34160)
	{
		chPolWychMagMMC[0] = PMMC3416_INT_CTRL0;
		chPolWychMagMMC[1] = chPolecenie |	//bit zdefiniowany w pliku h
						 (0 << 1) |	//Continuous Measurement Mode On
						 (3 << 2);	//CM Freq0..1 How often the chip will take measurements in Continuous Measurement Mode: 0=1,5Hz, 1=13Hz, 2=25Hz, 3=50Hz
		chBlad = HAL_I2C_Master_Transmit_DMA(&hi2c4, MMC34160_I2C_ADR, chPolWychMagMMC, 2);	//wyślij polecenie wykonania pomiaru
	}
	return chBlad;
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

