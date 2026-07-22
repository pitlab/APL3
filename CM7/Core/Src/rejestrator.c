//////////////////////////////////////////////////////////////////////////////
//
// Obsługa rejestratora na karcie SD
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Rejestrator.h>
#include <Czas.h>
#include <Exif.h>
#include <Jpeg.h>
#include <Bmp.h>
#include <Kamera.h>
#include <LCD/ILI9488.h>
#include "bsp_driver_sd.h"
#include "ModulySPI.h"
#include "wymiana.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <string.h>
#include <stdio.h>
#include "Ekran.h"
#include "LCD.h"


extern SD_HandleTypeDef hsd1;
extern uint8_t retSD;    /* Return value for SD */
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */
uint8_t cNazwaPlikuObrazu[DLG_NAZWY_PLIKU_OBR];	//początek nazwy pliku z obrazem, po tym jest data i czas
extern volatile unia_wymianyCM4_t uDaneCM4;
uint8_t __attribute__ ((aligned (32))) aTxBuffer[_MAX_SS];
uint8_t __attribute__ ((aligned (32))) aRxBuffer[_MAX_SS];
__IO uint8_t RxCplt, TxCplt;
volatile uint8_t cStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
uint32_t nKonfLogera[3] = {0xFFFFFFFF, 0xF8FFFFFF, 0xFFF};	//zestaw flag włączajacych dane do rejestracji
static char __attribute__ ((aligned (32))) cBufZapisuKarty[ROZMIAR_BUFORA_LOGU];	//bufor na jedną linijkę logu
char __attribute__ ((aligned (32))) cBufPodreczny[_MAX_LFN];
UINT nDoZapisuNaKarte, nZapisanoNaKarte;
uint8_t cKodBleduFAT;
uint8_t cTimerSync;	//odlicza czas w jednostce zapisu na dysk do wykonania sync
uint16_t sDlugoscWierszaLogu, sMaxDlugoscWierszaLogu;
extern RTC_TimeTypeDef stTime;
extern RTC_DateTypeDef stDate;
extern double dSumaZyro1[3], dSumaZyro2[3];
extern volatile uint8_t cStatusBufJpeg;	//przechowyje bity okreslające status procesu przepływu danych z bufora danych skompresowanych
extern volatile uint8_t cWskOprBufJpeg;	// wskazuje z którego bufora obecnie są odczytywane dane
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM"))) cBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG];
extern volatile uint16_t sZajetoscBuforaWyJpeg;		//liczba bajtów w buforze wyjściowym kompresora

FIL SDJpegFile;       //struktura pliku z obrazem
extern JPEG_HandleTypeDef hjpeg;
extern const uint8_t cNaglJpegSOI[ROZMIAR_NAGL_JPEG];
extern const uint8_t cNaglJpegEOI[ROZMIAR_ZNACZ_xOI];	//EOI (End Of Image)
extern const uint8_t cNaglJpegExif[ROZMIAR_EXIF];
extern stKonfKam_t stKonfKam;
extern JPEG_ConfTypeDef stKonfJpeg;	//struktura konfiguracyjna JPEGa
extern volatile uint8_t cCzasSwieceniaLED[LICZBA_LED];	//czas świecenia liczony w kwantach 0,1s jest zmniejszany w przerwaniu TIM17_IRQHandler



////////////////////////////////////////////////////////////////////////////////
// Wątek rejestratora na karcie SD
// Parametry: argument* ?
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WatekRejestratora(void *argument)
{
	extern volatile uint8_t cStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
	extern uint8_t cPort_exp_odbierany[LICZBA_EXP_SPI_ZEWN];
	extern uint8_t cKodBleduFAT;

	for(;;)
	{
		if ((cPort_exp_odbierany[0] & EXP04_LOG_CARD_DET)	== 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty, aktywny niski
		{
			if (cStatusRejestratora & STATREJ_FAT_GOTOWY)
			{
				if (cStatusRejestratora & STATREJ_WLACZONY)
				{
					ObslugaPetliRejestratora();
					osDelay(50);	//okres logowania 20 Hz
					//osDelay(20);	//okres logowania 50 Hz
				}
				else
				if (cStatusRejestratora & STATREJ_ZAPISZ_JPG)
				{
					ObslugaZapisuJpeg();
				}
				else
				if (cStatusRejestratora & STATREJ_ZAPISZ_BMP)
				{
					ObslugaZapisuBmp();
				}
				else
					osDelay(5);	//jeżeli nie ma nic do zapisu to wstrzymaj wątek na tyle czasu
			}
			else	//jeżeli FAT nie jest gotowy to go zamontuj
			{
				DSTATUS status;
				FRESULT fres;

				hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
				//hsd1.ErrorCode = 0;							//zacznij pracę bez kodu błędu
				status = disk_initialize(0);
				if (status == RES_OK)
				{
					fres = BSP_SD_Init();
					if (fres == FR_OK)
					{
						fres = f_mount(&SDFatFS, SDPath, 1);		//1=montuj teraz, 0=przy próbie zapisu
						if (fres == FR_OK)
						{
							cStatusRejestratora |= STATREJ_FAT_GOTOWY;
						}
						else
						{
							//jeżeli nie udało sie zamontować FAT to utwórz go ponownie
							DWORD au = _MAX_SS;
							fres = f_mkfs(SDPath, FM_FAT32, au, NULL, _MAX_SS);	//sprawdzić czy tak może być
						}
						cKodBleduFAT = fres;
					}
					else
						cCzasSwieceniaLED[LED_CZER] = 3;	//x0,1s - sygnalizacja błędu montowania woluminu karty
				}
				if (fres != FR_OK)
					osDelay(1000);	//ponawiaj próbę inicjalizacji co tyle czasu
			}
		}
		else	//jeżeli nie ma karty
		{
			if (cStatusRejestratora & STATREJ_FAT_GOTOWY)
			{
				if (cStatusRejestratora & STATREJ_OTWARTY_PLIK)
					f_close(&SDFile);
				f_mount(NULL, "", 1);		//zdemontuj system plików
				cStatusRejestratora = 0;
			}
			else
				cCzasSwieceniaLED[LED_CZER] = 1;	//x0,1s - sygnalizacja braku karty
			osDelay(2000);	//sprawdź czy jest karta co tyle czasu

		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Zwraca obecność karty w gnieździe. Wymaga wcześniejszego odczytania stanu expanderów I/O, ktore czytane są w każdym obiegu pętli StartDefaultTask()
// Parametry: brak
// Zwraca: obecność karty: SD_PRESENT == 1 lub SD_NOT_PRESENT == 0
////////////////////////////////////////////////////////////////////////////////
uint8_t BSP_SD_IsDetected(void)
{
	uint8_t cStatus = SD_PRESENT;
	extern uint8_t cPort_exp_odbierany[3];

	if (cPort_exp_odbierany[0] & EXP04_LOG_CARD_DET)		//styk detekcji karty zwiera do masy gdy karta jest obecna a pulllup wystawia 1 gdy jest nieobecna w gnieździe
		cStatus = SD_NOT_PRESENT;
	return cStatus;
}



////////////////////////////////////////////////////////////////////////////////
// Włącza napiecie 1.8V dla karty
// Parametry: status:  SET - włącz 1,8V
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_SD_DriveTransceiver_1_8V_Callback(FlagStatus status)
{
	extern uint8_t cPort_exp_wysylany[];
	extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
	uint8_t cBłąd;

	//Może być wywoływany przez inicjalizacją Expanderów, więc sprawdź czy expandery są zainicjowane a jeżeli nie to najpierw je inicjalizuj
	if ((nZainicjowanoCM7 & INIT_EXPANDER_IO) == 0)
		InicjujSPIModZewn();

	if (status == SET)
		cPort_exp_wysylany[0] &= ~EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: L=1,8V
	else
		cPort_exp_wysylany[0] |=  EXP02_LOG_VSELECT;	//LOG_SD1_VSEL: H=3,3V

	//wysyłaj aż dane do ekspandera do skutku
	do
	{
		cBłąd = WyslijDaneExpandera(SPI_EXTIO_0, cPort_exp_wysylany[0]);
		if (cBłąd == BLAD_SEMAFOR_ZAJETY)	//czy SPi jest zajęte przez inny proces?
			osDelay(1);						//jeżeli tak to czekaj aż inny proces zakończy pracę
	}
	while (cBłąd != BLAD_OK);
}




////////////////////////////////////////////////////////////////////////////////
// Obsługa zapisu danych w rejestratorze. Funkcja jest wywoływana cyklicznie w dedykowanym wątku
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaPetliRejestratora(void)
{
	cBufZapisuKarty[0] = 0;	//ustaw pusty bufor
	char *cZnak;

	if (cStatusRejestratora & STATREJ_OTWARTY_PLIK)
	{
	//--- pierwsze słowo konfiguracji logera --------------------------
		//czas
		if (nKonfLogera[0] & KLOG1_CZAS)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Czas [g:m:s.ss];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				PobierzDateCzas(&stDate, &stTime);
				uint32_t nSetneSekundy;
				nSetneSekundy = 99 - (99 * stTime.SubSeconds / stTime.SecondFraction);
				sprintf(cBufPodreczny, "%02d:%02d:%02d.%02ld;", stTime.Hours,  stTime.Minutes,  stTime.Seconds, nSetneSekundy);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//ciśnienie atmosferyczne z czujnika ciśnienia 1
		if (nKonfLogera[0] & KLOG1_PRES1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "CisnienieBzw1 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnieBzw[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//ciśnienie atmosferyczne z czujnika ciśnienia 2
		if (nKonfLogera[0] & KLOG1_PRES2)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "CisnienieBzw2 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnieBzw[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wysokość barometryczna z czujnika ciśnienia 1
		if (nKonfLogera[0] & KLOG1_AMSL1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Wysokosc MSL1 [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fWysokoMSL[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wysokość barometryczna z czujnika ciśnienia 2
		if (nKonfLogera[0] & KLOG1_AMSL2)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Wysokosc MSL2 [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fWysokoMSL[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wskazania wariometru 1
		if (nKonfLogera[0] & KLOG1_VARIO1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Wariometr 1 [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.fWariometr[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wskazania wariometru 2
		if (nKonfLogera[0] & KLOG1_VARIO2)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Wariometr 2 [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.fWariometr[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Ciśnienie czujnika różnicowego 1
		if (nKonfLogera[0] & KLOG1_CISROZ1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "CisnRozn1 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fCisnRozn[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Ciśnienie czujnika różnicowego 2
		if (nKonfLogera[0] & KLOG1_CISROZ2)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "CisnRozn2 [Pa];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fCisnRozn[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Prędkość wzgledem powietrza z czujnika różnicowego 1
		if (nKonfLogera[0] & KLOG1_IAS1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "IAS1 [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fPredkosc[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Prędkość wzgledem powietrza z czujnika różnicowego 2
		if (nKonfLogera[0] & KLOG1_IAS2)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "IAS2 [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fPredkosc[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura czujnika ciśnienia 1
		if (nKonfLogera[0] & KLOG1_TEMPBARO1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "TempBaro1 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_BARO1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura czujnika ciśnienia różnicowego 1
		if (nKonfLogera[0] & KLOG1_TEMPCISR1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "TempCisnRozn1 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_CISR1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura czujnika ciśnienia różnicowego 2
		if (nKonfLogera[0] & KLOG1_TEMPCISR2)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "TempCisnRozn2 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_CISR2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


	//--- drugie słowo konfiguracji logera --------------------------
		//surowa prędkość obrotowa P żyroskopu 1
		if (nKonfLogera[1] & KLOG2_ZYROSUR1P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ZyroSur1P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur1[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa Q żyroskopu 1
		if (nKonfLogera[1] & KLOG2_ZYROSUR1Q)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ZyroSur1Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur1[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa R żyroskopu 1
		if (nKonfLogera[1] & KLOG2_ZYROSUR1R)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ZyroSur1R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur1[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa P żyroskopu 2
		if (nKonfLogera[1] & KLOG2_ZYROSUR2P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ZyroSur2P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur2[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa Q żyroskopu 2
		if (nKonfLogera[1] & KLOG2_ZYROSUR2Q)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ZyroSur2Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur2[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa R żyroskopu 2
		if (nKonfLogera[1] & KLOG2_ZYROSUR2R)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ZyroSur2R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur2[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa P żyroskopu 1
		if (nKonfLogera[1] & KLOG2_ZYRO1P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "Zyro1 X [%c/s];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "ZyroKal1P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal1[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa Q żyroskopu 1
		if (nKonfLogera[1] & KLOG2_ZYRO1Q)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "Zyro1 Y [%c/s];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "ZyroKal1Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal1[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa R żyroskopu 1
		if (nKonfLogera[1] & KLOG2_ZYRO1R)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "Zyro1 Z [%c/s];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "ZyroKal1R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal1[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa P żyroskopu 2
		if (nKonfLogera[1] & KLOG2_ZYRO2P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "Zyro2 X [%c/s];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "ZyroKal2P [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal2[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa Q żyroskopu 2
		if (nKonfLogera[1] & KLOG2_ZYRO2Q)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "Zyro2 Y [%c/s];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "ZyroKal2Q [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal2[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa R żyroskopu 2
		if (nKonfLogera[1] & KLOG2_ZYRO2R)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "Zyro2 Z [%c/s];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "ZyroKal2R [rad/s];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal2[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

	//---------------------------
		//przyspieszenie w osi X akcelerometru 1
		if (nKonfLogera[1] & KLOG2_AKCEL1X)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Akcel1X [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel1[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Y akcelerometru 1
		if (nKonfLogera[1] & KLOG2_AKCEL1Y)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Akcel1Y [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel1[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Z akcelerometru 1
		if (nKonfLogera[1] & KLOG2_AKCEL1Z)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Akcel1Z [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel1[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi X akcelerometru 2
		if (nKonfLogera[1] & KLOG2_AKCEL2X)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Akcel2X [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel2[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Y akcelerometru 2
		if (nKonfLogera[1] & KLOG2_AKCEL2Y)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Akcel2Y [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel2[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi Z akcelerometru 2
		if (nKonfLogera[1] & KLOG2_AKCEL2Z)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Akcel2Z [m/s^2];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel2[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi X magnetometru 1
		if (nKonfLogera[1] & KLOG2_MAG1X)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn1X [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne1[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Y magnetometru 1
		if (nKonfLogera[1] & KLOG2_MAG1Y)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn1Y [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne1[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Z magnetometru 1
		if (nKonfLogera[1] & KLOG2_MAG1Z)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn1Z [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne1[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi X magnetometru 2
		if (nKonfLogera[1] & KLOG2_MAG2X)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn2X [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne2[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Y magnetometru 2
		if (nKonfLogera[1] & KLOG2_MAG2Y)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn2Y [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne2[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Z magnetometru 2
		if (nKonfLogera[1] & KLOG2_MAG2Z)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn2- [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne2[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi X magnetometru 3
		if (nKonfLogera[1] & KLOG2_MAG3X)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn3X [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne3[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Y magnetometru 3
		if (nKonfLogera[1] & KLOG2_MAG3Y)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn3Y [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne3[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Y magnetometru 3
		if (nKonfLogera[1] & KLOG2_MAG3Z)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Magn3Z [-];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fMagne3[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura IMU1
		if (nKonfLogera[1] & KLOG2_TEMPIMU1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "TempIMU1 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_IMU1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura IMU2
		if (nKonfLogera[1] & KLOG2_TEMPIMU2)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "TempIMU2 [K];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_IMU2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


	//--- trzecie słowo konfiguracji logera --------------------------

		//kąt phi wektora inercji
		if (nKonfLogera[2] & KLOG3_KATPHI)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Phi [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stBSP.fKatIMU[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt theta wektora inercji
		if (nKonfLogera[2] & KLOG3_KATTHE)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Theta [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stBSP.fKatIMU[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt psi wektora inercji
		if (nKonfLogera[2] & KLOG3_KATPSI)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "Psi [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stBSP.fKatIMU[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt phi obliczony na podstawie danych z akcelerometru
		if (nKonfLogera[2] & KLOG3_KATPHIA)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PhiAkcel1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatAkcel1[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt theta obliczony na podstawie danych z akcelerometru
		if (nKonfLogera[2] & KLOG3_KATTHEA)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ThetaAkcel1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatAkcel1[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt psi obliczony na podstawie danych z magnetometru
		if (nKonfLogera[2] & KLOG3_KATPSIM)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PsiAkcel1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatAkcel1[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt phi obliczony jako całka prędkości P z żyroskopu
		if (nKonfLogera[2] & KLOG3_KATPHIZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PhiZyro1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro1[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PhiZyro2 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro2[0]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt theta obliczony jako całka prędkości Q z żyroskopu
		if (nKonfLogera[2] & KLOG3_KATTHEZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ThetaZyro1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro1[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "ThetaZyro2 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro2[1]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąt psi obliczony jako całka prędkości R z żyroskopu
		if (nKonfLogera[2] & KLOG3_KATPSIZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PsiZyro1 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro1[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PsiZyro2 [rad];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fKatZyro2[2]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


	//----------------- GNSS --------------------------------
		//Szerokość geograficzna z GPS
		if (nKonfLogera[2] & KLOG3_GLONG)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "SzerokoscGeo [%c];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "SzerokoscGeo [°];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.6f;", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Długość geograficzna z GPS
		if (nKonfLogera[2] & KLOG3_GLATI)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "DlugoscGeo [%c];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty,  "DlugoscGeo [°];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.6f;", uDaneCM4.dane.stGnss1.dDlugoscGeo);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wysokość n.p.m. z GPS
		if (nKonfLogera[2] & KLOG3_GALTI)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "WysokoscGPS [mnpm];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.stGnss1.fWysokoscMSL);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//prędkość wzgledem ziemi z GPS
		if (nKonfLogera[2] & KLOG3_GSPED)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PredWzgZiemi [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kurs względem ziemi z GPS
		if (nKonfLogera[2] & KLOG3_GCURS)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				//sprintf(cBufPodreczny, "Kurs [%c];", '°');
				//strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
				strncat(cBufZapisuKarty, "KursGeo [°];", MAX_ROZMIAR_WPISU_LOGU);
			}
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fKurs);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//liczba widocznych satelitów / Fix
		if (nKonfLogera[2] & KLOG3_GSATS)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "LiczbaSat/Fix;", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%d/%d;", uDaneCM4.dane.stGnss1.cLiczbaSatelit, uDaneCM4.dane.stGnss1.cFix);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Vertical Dilution of Precision
		if (nKonfLogera[2] & KLOG3_GVDOP)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "VDOP [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fVdop);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Horizontal Dilution of Precision
		if (nKonfLogera[2] & KLOG3_GHDOP)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "HDOP [m];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fHdop);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//niefiltrowana prędkość z GPS w kierunku północnym
		if (nKonfLogera[2] & KLOG3_GSPD_N)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PredGPS_N [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi * cosf(uDaneCM4.dane.stGnss1.fKurs * DEG2RAD));		//sprawdzić!
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//niefiltrowana prędkość z GPS w kierunku wschodnim
		if (nKonfLogera[2] & KLOG3_GSPD_E)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(cBufZapisuKarty, "PredGPS_E [m/s];", MAX_ROZMIAR_WPISU_LOGU);
			else
			{
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi * sinf(uDaneCM4.dane.stGnss1.fKurs * DEG2RAD));		//sprawdzić!
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


		//jeżeli był zapisywany nagłówek to przejdź do zapisu danych
		if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			cStatusRejestratora &= ~ STATREJ_ZAPISZ_NAGLOWEK;
		else
		{
			//znajdź kropki i zamień na przecinki
			do
			{
				cZnak = strchr(cBufZapisuKarty, '.');
				if (cZnak)
					*cZnak = ',';
			}
			while (cZnak);
		}

		//sprawdź poziom zapełnienia bufora logu
		sDlugoscWierszaLogu = strlen(cBufZapisuKarty);
		if (sDlugoscWierszaLogu > sMaxDlugoscWierszaLogu)
			sMaxDlugoscWierszaLogu = sDlugoscWierszaLogu;
		if (sDlugoscWierszaLogu > ROZMIAR_BUFORA_LOGU)
			Error_Handler();
		strncat(cBufZapisuKarty, "\n", 2);	//znak końca wiersza

		f_puts(cBufZapisuKarty, &SDFile);	//zapis do pliku

		//co określoną liczbę zapisów zrób sync aby nie utracić danych w przypadku braku formalnego zakończenia logowania
		if (cTimerSync)
			cTimerSync--;
		else
		{
			cTimerSync = WPISOW_NA_SYNC;
			f_sync(&SDFile);				//Flush cached data of the writing file
		}
	}
	else	//jeżei plik nie jest otwarty to go otwórz
	{
		FRESULT fres;
		PobierzDateCzas(&stDate, &stTime);
		sprintf(cBufPodreczny, "%04d%02d%02d_%02d%02d%02d_APL3.csv",stDate.Year+2000, stDate.Month, stDate.Date, stTime.Hours, stTime.Minutes, stTime.Seconds);
		fres = f_open(&SDFile, cBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
		if (fres == FR_OK)
			cStatusRejestratora |= STATREJ_OTWARTY_PLIK | STATREJ_ZAPISZ_NAGLOWEK;
		sMaxDlugoscWierszaLogu = 0;
		cTimerSync = WPISOW_NA_SYNC;
	}

	//sprawdź czy nie wyłączono rejestratoraw czasie pracy
	if ((cStatusRejestratora & STATREJ_WLACZONY) == 0)
	{
		if (cStatusRejestratora & STATREJ_OTWARTY_PLIK)
		{
			f_close(&SDFile);
			f_mount(NULL, "", 1);		//zdemontuj system plików
			cStatusRejestratora = 0;
		}
	}

	//wydano polecenie zamknięcia pliku
	if (cStatusRejestratora & STATREJ_ZAMKNIJ_PLIK)
	{
		f_close(&SDFile);
		cStatusRejestratora &= ~(STATREJ_ZAMKNIJ_PLIK | STATREJ_OTWARTY_PLIK | STATREJ_WLACZONY);
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
	uint32_t nIndex = 0;
	__IO uint8_t step = 0;
	uint32_t start_time = 0;
	uint32_t stop_time = 0;
	char cNapis[60];
	//extern uint8_t cPort_exp_wysylany[];
	//float fNapiecie;

	/*if (chRysujRaz)
	{
		setColor(GRAY80);
		chRysujRaz = 0;
		BelkaTytulu("Test tranferu karty SD");

		if (cPort_exp_wysylany[0] & EXP02_LOG_VSELECT)	//LOG_SD1_VSEL: H=3,3V
			fNapiecie = 3.3;
		else
			fNapiecie = 1.8;
		setColor(YELLOW);
		sprintf(cNapis, "Karta pracuje z napi%cciem: %.1fV ", ę, fNapiecie);
		print(cNapis, 10, 30);

		setColor(GRAY40);
		sprintf(cNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(cNapis, CENTER, 300);
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
	    		for (nIndex = 0; nIndex < BUFFER_SIZE; nIndex++)
	    			aTxBuffer[nIndex] = DATA_PATTERN + nIndex;
	    		SCB_CleanDCache_by_Addr((uint32_t*)aTxBuffer, BUFFER_SIZE);
	    		nIndex = 0;
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
	    			nIndex++;
	    			if(nIndex < NB_BUFFER)
	    				step--;
	    			else
					{
						stop_time = HAL_GetTick();
						printf(cNapis, "Czas zapisu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
						//setColor(GRAY80);
						//sprintf(cNapis, "Czas zapisu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
						//print(cNapis, 10, 50);
						step++;
					}
				}
	    		break;

	    	case 3:	//Initialize Reception buffer
	    		for (nIndex = 0; nIndex < BUFFER_SIZE; nIndex++)
	    			aRxBuffer[nIndex] = 0;
	    		SCB_CleanDCache_by_Addr((uint32_t*)aRxBuffer, BUFFER_SIZE);
	    		start_time = HAL_GetTick();
	    		nIndex = 0;
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
	    			nIndex++;
	    			if(nIndex < NB_BUFFER)
	    				step--;
	    			else
	    			{
	    				stop_time = HAL_GetTick();
	    				printf(cNapis, "Czas odczytu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
	    				//setColor(GRAY80);
	    				//sprintf(cNapis, "Czas odczytu: %lums, transfer %02.2f MB/s  ", stop_time - start_time, (float)((float)(DATA_SIZE>>10)/(float)(stop_time - start_time)));
	    				//print(cNapis, 10, 70);
	    				step++;
	    			}
	    		}
	    		break;

	    	case 6:	//Check Reception buffer
	    		nIndex = 0;
	    		while((nIndex < BUFFER_SIZE) && (aRxBuffer[nIndex] == aTxBuffer[nIndex]))
	    			nIndex++;

	    		if (nIndex != BUFFER_SIZE)
	    		{
	    			//setColor(RED);
	    			//sprintf(cNapis, "B%c%cd weryfikacji!", ł, ą);
	    			//print(cNapis, 10, 90);
	    			Error_Handler();
	    		}

	    		//setColor(GREEN);
	    		//sprintf(cNapis, "Weryfikacja OK");
	    		//print(cNapis, 10, 90);
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
// zapisuje strumień danych z enkodera jpeg na kartę przekazywany przez bufor: cBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG]
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaZapisuJpeg(void)
{
	FRESULT fres;
	UINT nZapisanoBajtow = 0;

	if (cStatusBufJpeg & STAT_JPG_OTWORZ)		//jest flaga otwarcia pliku
	{
		if (cStatusBufJpeg & STAT_JPG_OTWARTY)
		{
			fres = f_close(&SDJpegFile);
			if (fres == FR_OK)
			{
				cStatusBufJpeg &= ~STAT_JPG_OTWARTY;
				printf("Awar.zamkn.pliku\r\n");
			}
			else
				printf("Blad zamkn.pliku\r\n");
		}

		PobierzDateCzas(&stDate, &stTime);
		sprintf(cBufPodreczny, "%s_%04d%02d%02d_%02d%02d%02d.jpg", cNazwaPlikuObrazu, stDate.Year+2000, stDate.Month, stDate.Date, stTime.Hours, stTime.Minutes, stTime.Seconds);
		fres = f_open(&SDJpegFile, cBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
		if (fres == FR_OK)
		{
			cStatusBufJpeg &= ~STAT_JPG_OTWORZ;	//skasuj flagę potwierdzając otwarcie pliku do zapisu
			cStatusBufJpeg |= STAT_JPG_OTWARTY;

			uint32_t nRozmiarExif = PrzygotujExif(&stKonfJpeg, &stKonfKam, &uDaneCM4.dane, &stDate, &stTime);
			nRozmiarExif = (nRozmiarExif + 3) & 0xFFFFFFFC;										//wyrównanie do 4 bajtów aby DMA się nie zacinało
			fres |= f_write(&SDJpegFile, cNaglJpegExif, nRozmiarExif, &nZapisanoBajtow);		//exif
		}
		else
		{
			printf("Blad otw.pliku\r\n");
		}
	}
	else
	if (sZajetoscBuforaWyJpeg && (cStatusBufJpeg & STAT_JPG_OTWARTY))
	{
		if (sZajetoscBuforaWyJpeg > ROZM_BUF_WY_JPEG)
			fres = f_write(&SDJpegFile, &cBuforJpeg[cWskOprBufJpeg][0], ROZM_BUF_WY_JPEG, &nZapisanoBajtow);
		else
			fres = f_write(&SDJpegFile, &cBuforJpeg[cWskOprBufJpeg][0], sZajetoscBuforaWyJpeg, &nZapisanoBajtow);

		if (fres == FR_OK)
		{
			cWskOprBufJpeg++;
			cWskOprBufJpeg &= MASKA_LICZBY_BUF;
			sZajetoscBuforaWyJpeg -= nZapisanoBajtow;
			if (cStatusBufJpeg & STAT_JPG_ZATRZYMANE_WY)
			{
				cStatusBufJpeg &= ~STAT_JPG_ZATRZYMANE_WY;
				HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_OUTPUT);		//wzów kompresję po opróżnieniu bufora wyjściowego
				printf("WznWy, ");
			}
		}
		else
			printf("Blad zap.pliku\r\n");
	}
	else
	if ((cStatusBufJpeg & STAT_JPG_ZAMKNIJ) && (cStatusBufJpeg & STAT_JPG_OTWARTY) && (sZajetoscBuforaWyJpeg == 0))
	{
		fres  = f_write(&SDJpegFile, cNaglJpegEOI, ROZMIAR_ZNACZ_xOI, &nZapisanoBajtow);
		fres |= f_close(&SDJpegFile);
		if (fres == FR_OK)
		{
			printf("Zapisane\r\n");
			cStatusBufJpeg &= ~(STAT_JPG_ZAMKNIJ + STAT_JPG_OTWARTY);	//skasuj flagi
			cStatusRejestratora &= ~STATREJ_ZAPISZ_JPG;	//wyłącz flagę obsługi pliku JPEG
		}
		else
			printf("Blad nr %d zamkniecia pliku\r\n", fres);
	}

	if (sZajetoscBuforaWyJpeg)	//jeżeli nadal jest coś do zapisu
	{
		if ((cStatusBufJpeg & STAT_JPG_OTWARTY) == 0)	//jeżeli są dane a plik nie jest otwarty to ustaw flagę otwarcia
		{
			cStatusBufJpeg |= STAT_JPG_OTWORZ;
			cStatusRejestratora |= STATREJ_ZAPISZ_JPG;	//wyłącz flagę obsługi pliku JPEG
			printf("Awar.otw.pliku\r\n");
		}
	}
	else
		osDelay(5);		//jeżeli nie ma nic do zapisu, to przełącz się na inny wątek
}
