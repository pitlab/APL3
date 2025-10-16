//////////////////////////////////////////////////////////////////////////////
//
// Obsługa rejestratora na karcie SD
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include "rejestrator.h"
#include "bsp_driver_sd.h"
#include "moduly_SPI.h"
#include "wymiana.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <string.h>
#include <stdio.h>
#include <jpeg.h>

extern SD_HandleTypeDef hsd1;
extern uint8_t retSD;    /* Return value for SD */
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */
uint8_t chNazwaPlikuObr[DLG_NAZWY_PLIKU_OBR];	//początek nazwy pliku z obrazem, po tym jest data i czas
extern volatile unia_wymianyCM4_t uDaneCM4;
uint8_t __attribute__ ((aligned (32))) aTxBuffer[_MAX_SS];
uint8_t __attribute__ ((aligned (32))) aRxBuffer[_MAX_SS];
__IO uint8_t RxCplt, TxCplt;
uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
uint32_t nKonfLogera[3] = {0xFFFFFF2B, 0xFFC00000, 0x000001F8};	//zestaw flag włączajacych dane do rejestracji
static char __attribute__ ((aligned (32))) chBufZapisuKarty[ROZMIAR_BUFORA_LOGU];	//bufor na jedną linijkę logu
char __attribute__ ((aligned (32))) chBufPodreczny[40];
UINT nDoZapisuNaKarte, nZapisanoNaKarte;
uint8_t chKodBleduFAT;
uint8_t chTimerSync;	//odlicza czas w jednostce zapisu na dysk do wykonania sync
uint16_t sDlugoscWierszaLogu, sMaxDlugoscWierszaLogu;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern double dSumaZyro1[3], dSumaZyro2[3];
extern uint8_t chStatusBufJpeg;	//przechowyje bity okreslające status procesu przepływu danych na kartę SD
extern volatile uint8_t chKolejkaBuf[ILOSC_BUF_JPEG];	//przechowuje kolejność zapisu buforów na kartę. Istotne gdy trzba zapisać więcej niż 1 bufor
extern uint8_t chWskNapKolejki, chWskOprKolejki;	//wskaźniki napełniania i opróżniania kolejki buforów do zapisu
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforJpeg[2][ROZMIAR_BUF_JPEG];
FIL SDJpegFile;       //struktura pliku z obrazem
//extern char chNapis[100];
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern JPEG_HandleTypeDef hjpeg;
extern const uint8_t chNaglJpegSOI[20];
extern const uint8_t chNaglJpegEOI[2];	//EOI (End Of Image)



////////////////////////////////////////////////////////////////////////////////
// Zwraca obecność karty w gnieździe. Wymaga wcześniejszego odczytania stanu expanderów I/O, ktore czytane są w każdym obiegu pętli StartDefaultTask()
// Parametry: brak
// Zwraca: obecność karty: SD_PRESENT == 1 lub SD_NOT_PRESENT == 0
////////////////////////////////////////////////////////////////////////////////
uint8_t BSP_SD_IsDetected(void)
{
	uint8_t status = SD_PRESENT;
	extern uint8_t chPort_exp_odbierany[3];

	if (chPort_exp_odbierany[0] & EXP04_LOG_CARD_DET)		//styk detekcji karty zwiera do masy gdy karta jest obecna a pulllup wystawia 1 gdy jest nieobecna w gnieździe
		status = SD_NOT_PRESENT;
	return status;
}



////////////////////////////////////////////////////////////////////////////////
// Włącza napiecie 1.8V dla karty
// Parametry: status:  SET - włącz 1,8V
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_SD_DriveTransceiver_1_8V_Callback(FlagStatus status)
{
	extern uint8_t chPort_exp_wysylany[];
	extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
	uint8_t chErr;

	//Może być wywoływany przez inicjalizacją Expanderów, więc sprawdź czy expandery są zainicjowane a jeżeli nie to najpierw je inicjalizuj
	if ((nZainicjowanoCM7 & INIT_EXPANDER_IO) == 0)
		InicjujSPIModZewn();

	if (status == SET)
		chPort_exp_wysylany[0] &= ~EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: L=1,8V
	else
		chPort_exp_wysylany[0] |=  EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: H=3,3V

	//wysyłaj aż dane do ekspandera do skutku
	do
	{
		chErr = WyslijDaneExpandera(SPI_EXTIO_0, chPort_exp_wysylany[0]);
		if (chErr == ERR_ZAJETY_SEMAFOR)	//czy SPi jest zajęte przez inny proces?
			osDelay(1);						//jeżeli tak to czekaj aż inny proces zakończy pracę
	}
	while (chErr != BLAD_OK);
}




////////////////////////////////////////////////////////////////////////////////
// Obsługa zapisu danych w rejestratorze. Funkcja jest wywoływana cyklicznie w dedykowanym wątku
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaPetliRejestratora(void)
{
	chBufZapisuKarty[0] = 0;	//ustaw pusty bufor

	if (chStatusRejestratora & STATREJ_OTWARTY_PLIK)
	{

		//czas
		if (nKonfLogera[0] & KLOG1_CZAS)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Czas [g:m:s.ss];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

				uint32_t nSetneSekundy;
				nSetneSekundy = 100 * sTime.SubSeconds / sTime.SecondFraction;
				sprintf(chBufPodreczny, "%02d:%02d:%02d.%02ld;", sTime.Hours,  sTime.Minutes,  sTime.Seconds, nSetneSekundy);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Ciśnienie czujnika 1
		if (nKonfLogera[0] & KLOG1_PRES1)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "CisnienieBzw1 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnieBzw[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Ciśnienie czujnika 2
		if (nKonfLogera[0] & KLOG1_PRES2)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "CisnienieBzw2 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnieBzw[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Wysokość czujnika ciśnienia 1
		if (nKonfLogera[0] & KLOG1_AMSL1)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Wysokosc MSL1 [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fWysokoMSL[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Wysokość czujnika ciśnienia 2
		if (nKonfLogera[0] & KLOG1_AMSL2)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Wysokosc MSL2 [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fWysokoMSL[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Ciśnienie różnicowe czujnika 1
		if (nKonfLogera[1] & KLOG2_CISROZ1)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "CisnRozn1 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fCisnRozn[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Ciśnienie różnicowe czujnika 2
		if (nKonfLogera[1] & KLOG2_CISROZ2)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "CisnRozn2 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fCisnRozn[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura czujnika ciśnienia różnicowego 1
		if (nKonfLogera[1] & KLOG2_TEMPCISR1)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "TempCisnRozn1 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_CISR1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura czujnika ciśnienia różnicowego 2
		if (nKonfLogera[1] & KLOG2_TEMPCISR2)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "TempCisnRozn2 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_CISR2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}



		//Prędkość wzgledem powietrza czujnika różnicowego 1
		if (nKonfLogera[0] & KLOG1_IAS1)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "IAS1 [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fPredkosc[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Prędkość wzgledem powietrza czujnika różnicowego 2
		if (nKonfLogera[0] & KLOG1_IAS2)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "IAS2 [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fPredkosc[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}
	//---------------------------------------------

		//prędkość obrotowa P żyroskopu 1
		if (nKonfLogera[0] & KLOG1_ZYRO1P)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "Zyro1 X [%c/s];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "ZyroKal1P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal1[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//prędkość obrotowa Q żyroskopu 1
		if (nKonfLogera[0] & KLOG1_ZYRO1Q)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "Zyro1 Y [%c/s];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "ZyroKal1Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal1[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//prędkość obrotowa R żyroskopu 1
		if (nKonfLogera[0] & KLOG1_ZYRO1R)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "Zyro1 Z [%c/s];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "ZyroKal1R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal1[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//prędkość obrotowa P żyroskopu 2
		if (nKonfLogera[0] & KLOG1_ZYRO2P)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "Zyro2 X [%c/s];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "ZyroKal2P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal2[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//prędkość obrotowa Q żyroskopu 2
		if (nKonfLogera[0] & KLOG1_ZYRO2Q)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "Zyro2 Y [%c/s];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "ZyroKal2Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal2[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//prędkość obrotowa R żyroskopu 2
		if (nKonfLogera[0] & KLOG1_ZYRO2R)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "Zyro2 Z [%c/s];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "ZyroKal2R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal2[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

	//---------------------------
		//przyspieszenie w osi X akcelerometru 1
		if (nKonfLogera[0] & KLOG1_AKCEL1X)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel1X [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel1[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Y akcelerometru 1
		if (nKonfLogera[0] & KLOG1_AKCEL1Y)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel1Y [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel1[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Z akcelerometru 1
		if (nKonfLogera[0] & KLOG1_AKCEL1Z)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel1Z [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel1[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi X akcelerometru 2
		if (nKonfLogera[0] & KLOG1_AKCEL2X)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel2X [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel2[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Y akcelerometru 2
		if (nKonfLogera[0] & KLOG1_AKCEL2Y)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel2Y [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel2[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Z akcelerometru 2
		if (nKonfLogera[0] & KLOG1_AKCEL2Z)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel2Z [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel2[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


		//temperatura czujnika ciśnienia 1
		if (nKonfLogera[0] & KLOG1_TEMPBARO1)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "TempBaro1 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_BARO1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


		//temperatura IMU1
		if (nKonfLogera[0] & KLOG1_TEMPIMU1)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "TempIMU1 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_IMU1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura IMU2
		if (nKonfLogera[0] & KLOG1_TEMPIMU2)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "TempIMU2 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_IMU2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

	//---------------------------

		//składowa magnetyczna w osi X magnetometru 1
		if (nKonfLogera[0] & KLOG1_MAG1X)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Magn1X [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne1[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Y magnetometru 1
		if (nKonfLogera[0] & KLOG1_MAG1Y)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Magn1Y [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne1[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Z magnetometru 1
		if (nKonfLogera[0] & KLOG1_MAG1Z)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Magn1Z [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne1[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi X magnetometru 2
		if (nKonfLogera[0] & KLOG1_MAG2X)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Magn2X [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne2[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Y magnetometru 2
		if (nKonfLogera[0] & KLOG1_MAG2Y)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Magn2Y [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne2[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Z magnetometru 2
		if (nKonfLogera[0] & KLOG1_MAG2Z)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Magn2- [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne2[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}




		//Magnetometr3 X
		if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			strncat(chBufZapisuKarty, "Magn3X [-];", MAX_ROZMIAR_WPISU_LOGU);
		else
		{
			sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne3[0]);
			strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//Magnetometr3 Y
		if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			strncat(chBufZapisuKarty, "Magn3Y [-];", MAX_ROZMIAR_WPISU_LOGU);
		else
		{
			sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne3[1]);
			strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//Magnetometr3 Z
		if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			strncat(chBufZapisuKarty, "Magn3Z [-];", MAX_ROZMIAR_WPISU_LOGU);
		else
		{
			sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne3[2]);
			strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}


	//---------------------------



	//----------------- GNSS --------------------------------
		//Szerokość geograficzna z GPS
		if (nKonfLogera[1] & KLOG2_GLONG)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "SzerokoscGeo [%c];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "SzerokoscGeo [°];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.6f;", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Długość geograficzna z GPS
		if (nKonfLogera[1] & KLOG2_GLATI)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "DlugoscGeo [%c];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty,  "DlugoscGeo [°];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.6f;", uDaneCM4.dane.stGnss1.dDlugoscGeo);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wysokość n.p.m. z GPS
		if (nKonfLogera[1] & KLOG2_GALTI)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "WysokoscGPS [mnpm];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.stGnss1.fWysokoscMSL);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//prędkość wzgledem ziemi z GPS
		if (nKonfLogera[1] & KLOG2_GSPED)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "PredWzgZiemi [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kurs względem ziemi z GPS
		if (nKonfLogera[1] & KLOG2_GCURS)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(chBufPodreczny, "Kurs [%c];", '°');
				//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(chBufZapisuKarty, "KursGeo [°];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fKurs);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//liczba widocznych satelitów / Fix
		if (nKonfLogera[1] & KLOG2_GSATS)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "LiczbaSat/Fix;", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%d/%d;", uDaneCM4.dane.stGnss1.chLiczbaSatelit, uDaneCM4.dane.stGnss1.chFix);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Vertical Dilution of Precision
		if (nKonfLogera[1] & KLOG2_GVDOP)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "VDOP [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fVdop);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Horizontal Dilution of Precision
		if (nKonfLogera[1] & KLOG2_GHDOP)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "HDOP [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fHdop);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//niefiltrowana prędkość z GPS w kierunku północnym
		if (nKonfLogera[1] & KLOG2_GSPD_N)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "PredGPS_N [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi * cosf(uDaneCM4.dane.stGnss1.fKurs * DEG2RAD));		//sprawdzić!
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//niefiltrowana prędkość z GPS w kierunku wschodnim
		if (nKonfLogera[1] & KLOG2_GSPD_E)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "PredGPS_E [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi * sinf(uDaneCM4.dane.stGnss1.fKurs * DEG2RAD));		//sprawdzić!
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa P żyroskopu 1
		if (nKonfLogera[0] & KLOG2_ZYROSUR1P)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ZyroSur1P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur1[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa Q żyroskopu 1
		if (nKonfLogera[0] & KLOG2_ZYROSUR1Q)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ZyroSur1Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur1[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa R żyroskopu 1
		if (nKonfLogera[0] & KLOG2_ZYROSUR1R)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ZyroSur1R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur1[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa P żyroskopu 2
		if (nKonfLogera[0] & KLOG2_ZYROSUR2P)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ZyroSur2P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur2[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa Q żyroskopu 2
		if (nKonfLogera[0] & KLOG2_ZYROSUR2Q)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ZyroSur2Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur2[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa R żyroskopu 2
		if (nKonfLogera[0] & KLOG2_ZYROSUR2R)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ZyroSur2R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur2[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


	//----------------- IMU --------------------------------
		//kąt phi wektora inercji
		if (nKonfLogera[2] & KLOG3_KATPHI)
		{

		}

		//kąt theta wektora inercji
		if (nKonfLogera[2] & KLOG3_KATTHE)
		{

		}

		//kąt psi wektora inercji
		if (nKonfLogera[2] & KLOG3_KATPSI)
		{

		}

		//kąt phi obliczony na podstawie danych z akcelerometru
		if (nKonfLogera[2] & KLOG3_KATPHIA)
		{

		}

		//kąt theta obliczony na podstawie danych z akcelerometru
		if (nKonfLogera[2] & KLOG3_KATTHEA)
		{

		}

		//kąt psi obliczony na podstawie danych z magnetometru
		if (nKonfLogera[2] & KLOG3_KATPSIM)
		{

		}

		//kąt phi obliczony jako całka prędkości P z żyroskopu
		if (nKonfLogera[2] & KLOG3_KATPHIZ)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "PhiZyro1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro1[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "PhiZyro2 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro2[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt theta obliczony jako całka prędkości Q z żyroskopu
		if (nKonfLogera[2] & KLOG3_KATTHEZ)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ThetaZyro1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro1[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "ThetaZyro2 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro2[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt psi obliczony jako całka prędkości R z żyroskopu
		if (nKonfLogera[2] & KLOG3_KATPSIZ)
		{
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "PsiZyro1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro1[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "PsiZyro2 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(chBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro2[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}



		//jeżeli był zapisywany nagłówek to przejdź do zapisu danych
		if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			chStatusRejestratora &= ~ STATREJ_ZAPISZ_NAGLOWEK;


		//sprawdź poziom zapełnienia bufora logu
		sDlugoscWierszaLogu = strlen(chBufZapisuKarty);
		if (sDlugoscWierszaLogu > sMaxDlugoscWierszaLogu)
			sMaxDlugoscWierszaLogu = sDlugoscWierszaLogu;
		if (sDlugoscWierszaLogu > ROZMIAR_BUFORA_LOGU)
			Error_Handler();
		strncat(chBufZapisuKarty, "\n", 2);	//znak końca wiersza

		f_puts(chBufZapisuKarty, &SDFile);	//zapis do pliku

		//co określoną liczbę zapisów zrób sync aby nie utracić danych w przypadku braku formalnego zakończenia logowania
		if (chTimerSync)
			chTimerSync--;
		else
		{
			chTimerSync = WPISOW_NA_SYNC;
			f_sync(&SDFile);				//Flush cached data of the writing file
		}
	}
	else	//jeżei plik nie jest otwarty to go otwórz
	{
		FRESULT fres;
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		sprintf(chBufPodreczny, "%04d%02d%02d_%02d%02d%02d_APL3.csv",sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
		fres = f_open(&SDFile, chBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
		if (fres == FR_OK)
			chStatusRejestratora |= STATREJ_OTWARTY_PLIK | STATREJ_ZAPISZ_NAGLOWEK;
		sMaxDlugoscWierszaLogu = 0;
		chTimerSync = WPISOW_NA_SYNC;
	}


	//sprawdź czy nie wyłączono rejestratoraw czasie pracy
	if ((chStatusRejestratora & STATREJ_WLACZONY) == 0)
	{
		if (chStatusRejestratora & STATREJ_OTWARTY_PLIK)
		{
			f_close(&SDFile);
			f_mount(NULL, "", 1);		//zdemontuj system plików
			chStatusRejestratora = 0;
		}
	}

	//wydano polecenie zamknięcia pliku
	if (chStatusRejestratora & STATREJ_ZAMKNIJ_PLIK)
	{
		f_close(&SDFile);
		chStatusRejestratora &= ~(STATREJ_ZAMKNIJ_PLIK | STATREJ_OTWARTY_PLIK | STATREJ_WLACZONY);
	}

	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
//  Verify that SD card is ready to use after the Erase
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t Wait_SDCARD_Ready(void)
{
	uint32_t loop = SD_TIMEOUT;

	while(loop > 0)
	{
		loop--;
		if(HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER)
			return HAL_OK;
	}
	return HAL_ERROR;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje okno z testem transferu karty SD
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestKartySD(void)
{
	uint32_t index = 0;
	__IO uint8_t step = 0;
	uint32_t start_time = 0;
	uint32_t stop_time = 0;
	char chNapis[60];
	//extern uint8_t chPort_exp_wysylany[];
	//float fNapiecie;

	/*if (chRysujRaz)
	{
		setColor(GRAY80);
		chRysujRaz = 0;
		BelkaTytulu("Test tranferu karty SD");

		if (chPort_exp_wysylany[0] & EXP02_LOG_VSELECT)	//LOG_SD1_VSEL: H=3,3V
			fNapiecie = 3.3;
		else
			fNapiecie = 1.8;
		setColor(YELLOW);
		sprintf(chNapis, "Karta pracuje z napi%cciem: %.1fV ", ę, fNapiecie);
		print(chNapis, 10, 30);

		setColor(GRAY40);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
	}*/


	if(HAL_SD_Erase(&hsd1, ADDRESS, ADDRESS + BUFFER_SIZE) != HAL_OK)
		Error_Handler();

	if(Wait_SDCARD_Ready() != HAL_OK)
		Error_Handler();

	while(1)
	{
		switch(step)
	    {
	    	case 0:	// Initialize Transmission buffer
	    		for (index = 0; index < BUFFER_SIZE; index++)
	    			aTxBuffer[index] = DATA_PATTERN + index;
	    		SCB_CleanDCache_by_Addr((uint32_t*)aTxBuffer, BUFFER_SIZE);
	    		index = 0;
	    		start_time = HAL_GetTick();
	    		step++;
	    		break;

	    	case 1:
	    		TxCplt = 0;
	    		if(Wait_SDCARD_Ready() != HAL_OK)
	    			Error_Handler();
	    		if(HAL_SD_WriteBlocks_DMA(&hsd1, aTxBuffer, ADDRESS, NB_BLOCK_BUFFER) != HAL_OK)
	    			Error_Handler();
	    		step++;
	    		break;

	    	case 2:
	    		if(TxCplt != 0)
	    		{
	    			index++;
	    			if(index < NB_BUFFER)
	    				step--;
	    			else
					{
						stop_time = HAL_GetTick();
						printf(chNapis, "Czas zapisu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
						//setColor(GRAY80);
						//sprintf(chNapis, "Czas zapisu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
						//print(chNapis, 10, 50);
						step++;
					}
				}
	    		break;

	    	case 3:	//Initialize Reception buffer
	    		for (index = 0; index < BUFFER_SIZE; index++)
	    			aRxBuffer[index] = 0;
	    		SCB_CleanDCache_by_Addr((uint32_t*)aRxBuffer, BUFFER_SIZE);
	    		start_time = HAL_GetTick();
	    		index = 0;
	    		step++;
	    		break;

	    	case 4:
	    		if(Wait_SDCARD_Ready() != HAL_OK)
	    			Error_Handler();

	    		RxCplt = 0;
	    		if(HAL_SD_ReadBlocks_DMA(&hsd1, aRxBuffer, ADDRESS, NB_BLOCK_BUFFER) != HAL_OK)
	    			Error_Handler();
	    		step++;
	    		break;

	    	case 5:
	    		if(RxCplt != 0)
	    		{
	    			index++;
	    			if(index<NB_BUFFER)
	    				step--;
	    			else
	    			{
	    				stop_time = HAL_GetTick();
	    				printf(chNapis, "Czas odczytu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
	    				//setColor(GRAY80);
	    				//sprintf(chNapis, "Czas odczytu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
	    				//print(chNapis, 10, 70);
	    				step++;
	    			}
	    		}
	    		break;

	    	case 6:	//Check Reception buffer
	    		index=0;
	    		while((index<BUFFER_SIZE) && (aRxBuffer[index] == aTxBuffer[index]))
	    			index++;

	    		if (index != BUFFER_SIZE)
	    		{
	    			//setColor(RED);
	    			//sprintf(chNapis, "B%c%cd weryfikacji!", ł, ą);
	    			//print(chNapis, 10, 90);
	    			Error_Handler();
	    		}

	    		//setColor(GREEN);
	    		//sprintf(chNapis, "Weryfikacja OK");
	    		//print(chNapis, 10, 90);
	    		step = 0;
	    		break;

	    	default :
	    		Error_Handler();
	    }

		//if(statusDotyku.chFlagi & DOTYK_DOTKNIETO)	//warunek wyjścia z testu
			//return;
	}
}



////////////////////////////////////////////////////////////////////////////////
// zapisuje strumień danych z enkodera jpeg na kartę przekazywany przez bufor: chBuforJpeg[ILOSC_BUF_JPEG][ROZMIAR_BUF_JPEG]
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaZapisuJpeg(void)
{
	FRESULT fres;
	UINT nZapisanoBajtow;

	extern volatile uint8_t chKolejkaBuf[ILOSC_BUF_JPEG];	//przechowuje kolejność zapisu buforów na kartę. Istotne gdy trzba zapisać więcej niż 1 bufor
	extern uint8_t chWskOprKolejki;	//wskaźnik opróżniania kolejki buforów do zapisu

	if (chStatusBufJpeg & STAT_JPG_OTWORZ)		//jest flaga otwarcia pliku
	{
		if (chStatusBufJpeg & STAT_JPG_OTWARTY)
		{
			fres = f_close(&SDJpegFile);
			if (fres == FR_OK)
			{
				chStatusBufJpeg &= ~STAT_JPG_OTWARTY;
				printf("Awar.zamkn.pliku\r\n");
			}
		}

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		sprintf(chBufPodreczny, "%s_%04d%02d%02d_%02d%02d%02d.jpg", chNazwaPlikuObr, sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

		fres = f_open(&SDJpegFile, chBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
		if (fres == FR_OK)
		{
			chStatusBufJpeg &= ~STAT_JPG_OTWORZ;	//skasuj flagę potwierdzając otwarcie pliku do zapisu
			chStatusBufJpeg |= STAT_JPG_OTWARTY;
		}
	}
	else
	if ((chStatusBufJpeg & (STAT_JPG_NAGLOWEK | STAT_JPG_PELEN_BUF0 | STAT_JPG_OTWARTY)) == (STAT_JPG_NAGLOWEK | STAT_JPG_PELEN_BUF0 | STAT_JPG_OTWARTY))	//jest pierwszy nie pełny bufor danych czekajacy na dodanie nagłówka
	{
		//na początku danych z jpeg jest znacznik SOI [2] ale brakuje nagłówka pliku. Wstaw kompletny nagłówek a za nim dane jpeg bez SOI
		//bufor z danymi zaczyna się od pozycji ROZMIAR_NAGL_JPEG
		f_write(&SDJpegFile, chNaglJpegSOI, ROZMIAR_NAGL_JPEG, &nZapisanoBajtow);
		//f_write(&SDJpegFile, &chBuforJpeg[0][ROZMIAR_NAGL_JPEG], ROZMIAR_BUF_JPEG - ROZMIAR_NAGL_JPEG, &bw);	//dane ze  znacznikiem SOI (FFD8) zawsze są w pierwszym buforze
		//powyższe niestety nie działa, więc pomimo konieczności wstawienia nagłówka zapisuję pełen bufor
		f_write(&SDJpegFile, &chBuforJpeg[0][ROZMIAR_ZNACZ_xOI], ROZMIAR_BUF_JPEG - ROZMIAR_ZNACZ_xOI, &nZapisanoBajtow);	//dane ze  znacznikiem SOI (FFD8) zawsze są w pierwszym buforze
		chStatusBufJpeg &= ~(STAT_JPG_NAGLOWEK | STAT_JPG_PELEN_BUF0);	//skasuj flagę
		chWskOprKolejki++;
		chWskOprKolejki &= MASKA_LICZBY_BUF;
	}
	else
	if ((chWskOprKolejki != chWskNapKolejki) | ((chStatusBufJpeg & (STAT_JPG_PELEN_BUF | STAT_JPG_OTWARTY)) == (STAT_JPG_PELEN_BUF | STAT_JPG_OTWARTY)))
	{
		uint8_t chZajetoscBufora;
		if (chWskNapKolejki > chWskOprKolejki)
			chZajetoscBufora = chWskNapKolejki - chWskOprKolejki;
		else
			chZajetoscBufora = ILOSC_BUF_JPEG + chWskNapKolejki - chWskOprKolejki;
		printf("Buf: %d\r\n", chZajetoscBufora);
		//jest bufor do zapisu. Wyciagnij go z kolejki aby zapisać w odpowiedzniej kolejności. Jest to istotne gdy np. są do zapisu pierwszy i ostatni, wtedy kolejka definiuje kolejność zapisu
		fres = f_write(&SDJpegFile, &chBuforJpeg[chKolejkaBuf[chWskOprKolejki]][0], ROZMIAR_BUF_JPEG, &nZapisanoBajtow);
		if (fres == FR_OK)
		{
			//chStatusBufJpeg &= ~(1 << chKolejkaBuf[chWskOprKolejki]);	//skasuj flagę opróżnionego bufora
			chStatusBufJpeg &= ~STAT_JPG_PELEN_BUF;
			chWskOprKolejki++;
			chWskOprKolejki &= MASKA_LICZBY_BUF;
		}
	}
	else
	if (chStatusBufJpeg & STAT_JPG_ZAMKNIJ)
	{
		fres = f_write(&SDJpegFile, chNaglJpegEOI, ROZMIAR_ZNACZ_xOI, &nZapisanoBajtow);
		fres = f_close(&SDJpegFile);
		if (fres == FR_OK)
		{
			chStatusBufJpeg &= ~(STAT_JPG_ZAMKNIJ | STAT_JPG_OTWARTY);	//skasuj flagi polecenia zamknięcia i stanu otwartosci pliku
			f_sync(&SDJpegFile);
			printf("Gotowe\r\n");
		}
	}

	//jeżeli nie ma nic do zapisu, to przełacz sie na inny wątek
	//if ((chStatusBufJpeg & (STAT_JPG_PELEN_BUF0 | STAT_JPG_PELEN_BUF1 | STAT_JPG_PELEN_BUF2 | STAT_JPG_PELEN_BUF3)) == 0)
	if ((chStatusBufJpeg & STAT_JPG_PELEN_BUF) == 0)
		osDelay(1);
}
