/*
 * rejestrator.c
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#include "rejestrator.h"
#include "bsp_driver_sd.h"
#include "moduly_SPI.h"
#include "wymiana.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <string.h>
#include <stdio.h>


extern SD_HandleTypeDef hsd1;
extern uint8_t retSD;    /* Return value for SD */
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */
extern uint8_t chPorty_exp_odbierane[LICZBA_EXP_SPI_ZEWN];
extern volatile unia_wymianyCM4_t uDaneCM4;
ALIGN_32BYTES(uint8_t aTxBuffer[_MAX_SS]);
ALIGN_32BYTES(uint8_t aRxBuffer[_MAX_SS]);
__IO uint8_t RxCplt, TxCplt;
uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
ALIGN_32BYTES(static char chBufZapisuKarty[ROZMIAR_BUFORA_KARTY]);	//bufor na jedną linijkę logu
ALIGN_32BYTES(static char chBufPodreczny[30]);
UINT nDoZapisuNaKarte, nZapisanoNaKarte;
uint8_t chKodBleduFAT;
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern double dSumaZyro1[3], dSumaZyro2[3];

////////////////////////////////////////////////////////////////////////////////
// Zwraca obecność karty w gnieździe. Wymaga wcześniejszego odczytania stanu expanderów I/O, ktore czytane są w każdym obiegu pętli StartDefaultTask()
// Parametry: brak
// Zwraca: obecność karty: SD_PRESENT == 1 lub SD_NOT_PRESENT == 0
////////////////////////////////////////////////////////////////////////////////
uint8_t BSP_SD_IsDetected(void)
{
	uint8_t status = SD_PRESENT;
	extern uint8_t chPorty_exp_odbierane[3];

	if (chPorty_exp_odbierane[0] & EXP04_LOG_CARD_DET)		//styk detekcji karty zwiera do masy gdy karta jest obecna a pulllup wystawia 1 gdy jest nieobecna w gnieździe
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
	extern uint8_t chPorty_exp_wysylane[];
	extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu
	uint8_t chErr;

	//Może być wywoływany przez inicjalizacją Expanderów, więc sprawdź czy expandery są zainicjowane a jeżeli nie to najpierw je inicjalizuj
	if ((nZainicjowano[0] & INIT0_EXPANDER_IO) == 0)
		InicjujSPIModZewn();

	if (status == SET)
		chPorty_exp_wysylane[0] &= ~EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: L=1,8V
	else
		chPorty_exp_wysylane[0] |=  EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: H=3,3V

	//wysyłaj aż dane do ekspandera do skutku
	do
		chErr = WyslijDaneExpandera(SPI_EXTIO_0, chPorty_exp_wysylane[0]);
	while (chErr != ERR_OK);
}




////////////////////////////////////////////////////////////////////////////////
// Obsługa zapisu danych w rejestratorze. Funkcja jest wywoływana cyklicznie w dedykowanym wątku
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaPetliRejestratora(void)
{
	chBufZapisuKarty[0] = 0;	//ustaw pusty bufor

	if ((chPorty_exp_odbierane[0] & EXP04_LOG_CARD_DET)	== 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty
	{

		if (chStatusRejestratora & STATREJ_FAT_GOTOWY)
		{
			if (chStatusRejestratora & STATREJ_OTWARTY_PLIK)
			{
				//czas
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Czas [g:m:s.ss];", MAX_ROZMIAR_WPISU);
				else
				{
					HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
					HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

					uint32_t nSetneSekundy;
					nSetneSekundy = 100 * sTime.SubSeconds / sTime.SecondFraction;
					sprintf(chBufPodreczny, "%02d:%02d:%02d.%02ld;", sTime.Hours,  sTime.Minutes,  sTime.Seconds, nSetneSekundy);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Ciśnienie czujnika 1
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Cisnienie1 [Pa];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnie[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Wysokość czujnika 1
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Wysokosc1 [m];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fWysoko[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//Ciśnienie różnicowe czujnika 1
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CisnRozn1 [Pa];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnRozn[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Ciśnienie różnicowe czujnika 2
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CisnRozn2 [Pa];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnRozn[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Prędkość wzgledem powietrza czujnika różnicowego 1
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "IAS1 [m/s];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fPredkosc[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Prędkość wzgledem powietrza czujnika różnicowego 2
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "IAS2 [m/s];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fPredkosc[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//akcelerometr1 X
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Akcel1 X [g];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fAkcel1[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//akcelerometr1 Y
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Akcel1 Y [g];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fAkcel1[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}
				//akcelerometr1 Z
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Akcel1 Z [g];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fAkcel1[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//akcelerometr2 X
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Akcel2 X [g];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fAkcel2[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//akcelerometr2 Y
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Akcel2 Y [g];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fAkcel2[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}
				//akcelerometr2 Z
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Akcel2 Z [g];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.3f;", uDaneCM4.dane.fAkcel2[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//żyroskop1 X
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "Zyro1 X [%c/s];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "Zyro1 X [°/s];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fZyroSur1[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//żyroskop1 Y
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "Zyro1 Y [%c/s];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "Zyro1 Y [°/s];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fZyroSur1[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//żyroskop1 Z
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "Zyro1 Z [%c/s];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "Zyro1 Z [°/s];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fZyroSur1[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//żyroskop2 X
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "Zyro2 X [%c/s];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "Zyro2 X [°/s];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fZyroSur2[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//żyroskop2 Y
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "Zyro2 Y [%c/s];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "Zyro2 Y [°/s];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fZyroSur2[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//żyroskop2 Z
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "Zyro2 Z [%c/s];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "Zyro2 Z [°/s];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fZyroSur2[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

//---------------------------

				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CalkaZyro1 X [°];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCalkaZyro1[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CalkaZyro1 Y [°];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCalkaZyro1[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CalkaZyro1 Z [°];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCalkaZyro1[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CalkaZyro2 X [°];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCalkaZyro2[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CalkaZyro2 Y [°];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCalkaZyro2[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "CalkaZyro2 Z [°];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.fCalkaZyro2[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//Magnetometr1 X
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn1 X [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne1[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Magnetometr1 Y
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn1 Y [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne1[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Magnetometr1 Z
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn1 Z [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne1[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//Magnetometr2 X
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn2 X [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne2[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Magnetometr1 Y
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn2 Y [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne2[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Magnetometr1 Z
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn2 Z [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne2[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//Magnetometr3 X
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn3 X [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne3[0]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Magnetometr3 Y
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn3 Y [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne3[1]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Magnetometr3 Z
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Magn3 Z [uT];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d;", uDaneCM4.dane.sMagne3[2]);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//----------------- GNSS --------------------------------
				//SzerokoscGeo
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "SzerokoscGeo [%c];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "SzerokoscGeo [°];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.6f;", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//DlugoscGeo
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "DlugoscGeo [%c];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty,  "DlugoscGeo [°];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.6f;", uDaneCM4.dane.stGnss1.dDlugoscGeo);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//fWysokoscMSL
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "WysokoscMSL [m];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.1f;", uDaneCM4.dane.stGnss1.fWysokoscMSL);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//PredkoscWzglZiemi
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Predk WzgZie [m/s];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Kurs GNSS
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					//sprintf(chBufPodreczny, "Kurs [%c];", '°');
					//strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
					strncat(chBufZapisuKarty, "Kurs [°];", MAX_ROZMIAR_WPISU);
				}
				else
				{
					sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fKurs);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}

				//Liczba Sat/Fix
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					strncat(chBufZapisuKarty, "Licz.Sat/Fix [-/-];", MAX_ROZMIAR_WPISU);
				else
				{
					sprintf(chBufPodreczny, "%d/%d;", uDaneCM4.dane.stGnss1.chLiczbaSatelit, uDaneCM4.dane.stGnss1.chFix);
					strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
				}


				//jeżeli był zapisywany nagłówek to przejdź do zapisu danych
				if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					chStatusRejestratora &= ~ STATREJ_ZAPISZ_NAGLOWEK;
				strncat(chBufZapisuKarty, "\n", 2);	//znak końca linii
				f_puts(chBufZapisuKarty, &SDFile);	//zapis do pliku
				chBufZapisuKarty[0] = 0;	//ustaw 0 na początku bufora
			}
			else	//jeżei plik nie jest otwarty to go otwórz
			{
				FRESULT fres;
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				sprintf(chBufPodreczny, "%04d%02d%02d_%02d%02d%02d_APL3.csv",sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);
				fres = f_open(&SDFile, chBufPodreczny, FA_OPEN_ALWAYS | FA_WRITE);
				if (fres == FR_OK)
					chStatusRejestratora |= STATREJ_OTWARTY_PLIK | STATREJ_ZAPISZ_NAGLOWEK;
			}
		}
		else	//jeżeli FAT nie jest gotowy to go zamontuj
		{
			DSTATUS status;
			FRESULT fres;

			hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
			status = SD_initialize(0);
			if (status == RES_OK)
			{
				fres = f_mount(&SDFatFS, SDPath, 1);
				if (fres == FR_OK)
				{
					chStatusRejestratora |= STATREJ_FAT_GOTOWY;
					//fres = f_open(&SDFile, "abc.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
					//if (fres == FR_OK)
					//{
						//f_gets(chBufZapisuKarty, ROZMIAR_BUFORA_KARTY, &SDFile);
						//f_close(&SDFile);
					//}
				}
				else
				{
					//jeżeli nie udało sie zamontować FAT to utwórz go ponownie
					DWORD au = _MAX_SS;
					fres = f_mkfs(SDPath, FM_FAT32, au, aTxBuffer, sizeof(aTxBuffer));	//sprawdzić czy tak może być
				}
				chKodBleduFAT = fres;
			}
		}
	}
	else	//jeżeli nie ma karty
	{
		if (chStatusRejestratora & STATREJ_FAT_GOTOWY)
		{
			if (chStatusRejestratora & STATREJ_OTWARTY_PLIK)
				f_close(&SDFile);
			f_mount(NULL, "", 1);		//zdemontuj system plików
			chStatusRejestratora = 0;
		}
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

	return ERR_OK;
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
	//extern uint8_t chPorty_exp_wysylane[];
	//float fNapiecie;

	/*if (chRysujRaz)
	{
		setColor(GRAY80);
		chRysujRaz = 0;
		BelkaTytulu("Test tranferu karty SD");

		if (chPorty_exp_wysylane[0] & EXP02_LOG_VSELECT)	//LOG_SD1_VSEL: H=3,3V
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


/*
 *
 * Program testowy pinów karty
uint8_t chExpander = 0;
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  chErr |= InicjujSPIModZewn();
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  	GPIO_InitStruct.Pin = GPIO_PIN_2;
  	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Pull = GPIO_NOPULL;
  	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    for (;;)
    {
    	chExpander ^= EXP02_LOG_VSELECT;
    	WyslijDaneExpandera(SPI_EXTIO_0, chExpander);
    	HAL_Delay(1);

    	for (uint8_t n=0; n<8; n++)
		{
		  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
		  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
		  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
		  //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
		}
    }
  }*/
