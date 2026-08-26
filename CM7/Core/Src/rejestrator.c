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
#include <Napisy.h>
#include "ProtokolKomunikacyjny.h"

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
uint32_t nKonfLogera[LICZBA_SLOW_REJESTRATORA] = {0x3F0710FF, 0x0003FFFF, 0x0000FFFF, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000FFF};	//zestaw flag włączajacych dane do rejestracji
static char __attribute__ ((aligned (32))) cBufZapisuKarty[ROZMIAR_BUFORA_LOGU];	//bufor na jedną linijkę logu
char __attribute__ ((aligned (32))) cBufPodreczny[_MAX_LFN];
UINT nDoZapisuNaKarte, nZapisanoNaKarte;
uint8_t cKodBleduFAT;
uint8_t cTimerSync;	//odlicza czas w jednostce zapisu na dysk do wykonania sync
uint16_t sDlugoscWierszaLogu, sMaxDlugoscWierszaLogu;
uint16_t sZapisanoLogu;
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
extern const char *cNazwyPozycjiRejestratora[LICZBA_NAZW_POZYCJI_REJESTRATORA];
extern stBSP_ID_t stBSP_ID;	//struktura zawierajaca adres i nazwę BSP


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
	uint32_t nCzas, nCzasRejestracji;
	//uint32_t nOkresRejestracji = 50000;	//10ms -> 100Hz, 50ms -> 20Hz
	uint32_t nOkresRejestracji = 100000;	//10ms -> 100Hz, 100ms -> 10Hz

	nCzasRejestracji = PobierzCzasT6();
	for(;;)
	{
		if ((cPort_exp_odbierany[0] & EXP04_LOG_CARD_DET) == 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty, aktywny niski
		{
			if (cStatusRejestratora & STATREJ_FAT_GOTOWY)
			{
				if (cStatusRejestratora & STATREJ_WLACZONY)
				{
					nCzas = MinalCzas(nCzasRejestracji);
					if (nCzas >= nOkresRejestracji)
					{
						nCzasRejestracji = PobierzCzasT6() - (nCzas - nOkresRejestracji);	//jeżeli czas nie trafił w swoje miejsce to skompensuj różnicę w następnym obiegu
						ObslugaPetliRejestratora();
						osDelay(5);
					}
					osDelay(2);
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
				FRESULT fres = FR_OK;

				//hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
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
// W pierezszym wierszu zapisuwany jest nagłówek logu. Nazwy pobierane są ze zmiennej: cNazwyPozycjiRejestratora[]
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaPetliRejestratora(void)
{
	uint8_t cBłąd = BLAD_OK;
	cBufZapisuKarty[0] = 0;	//ustaw pusty bufor
	char *cZnak;

	if (cStatusRejestratora & STATREJ_OTWARTY_PLIK)
	{
	//--- pierwsze słowo konfiguracji logera --------------------------
		//czas
		if (nKonfLogera[0] & KLOG1_CZAS)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_CZAS_GGMMSSSS]);
			else
			{
				PobierzDateCzas(&stDate, &stTime);
				uint32_t nSetneSekundy;
				nSetneSekundy = 99 - (99 * stTime.SubSeconds / stTime.SecondFraction);
				sprintf(cBufPodreczny, "%02d:%02d:%02d.%02ld;", stTime.Hours,  stTime.Minutes,  stTime.Seconds, nSetneSekundy);
			}
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//ciśnienie atmosferyczne z czujnika ciśnienia 1 i 2
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[0] & KLOG1_PRES1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_CISNIENIE_BZWZGL_XD_PA]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie

				}
				else
					sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fCisnieBzw[n]);

				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wysokość barometryczna bezwzględna z czujnika ciśnienia 1 i 2
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[0] & KLOG1_AMSL1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_WYSOKOSC_MSL_XD_M]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fWysokoMSL[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wysokość barometryczna względna z czujnika ciśnienia 1 i 2
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[0] & KLOG1_AGL1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_WYSOKOSC_AGL_XD_M]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fWysokoAGL[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//wskazania wariometru 1 i 2
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[0] & KLOG1_WARIO1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_WARIOMETR_XD_MS]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.fWariometr[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Ciśnienie czujnika różnicowego 1 i 2
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[0] & KLOG1_CISROZ1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_CISN_ROZNICOWE_XD_PA]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fCisnRozn[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//Prędkość wzgledem powietrza z czujnika różnicowego 1 i 2
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[0] & KLOG1_IAS1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_PREDK_IAS_XD_MS]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fPredkosc[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


		//temperatura czujnika ciśnienia 1
		if (nKonfLogera[0] & KLOG1_TEMPBARO1)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
			{
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_TEMP_BARO_XD_K]);
				sprintf(cBufPodreczny, cBufPodreczny, 1);	//wypełnij parametr XD=%d zakodowany w nazwie
			}
			else
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_BARO1]);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//temperatura czujnika ciśnienia różnicowego 1 i 2
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[0] & KLOG1_TEMPCISR1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_TEMP_ROZN_XD_K]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemper[TEMP_CISR1 + n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


		for (uint8_t n=0; n<2; n++)	//pętla dla dwóch kanałów zasilania
		{
			//napięcie baterii
			if (nKonfLogera[n] & (KLOG1_BAT1_NAP << 4*n))
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_BAT_XD_NAPIECIE_V]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fNapiecieAku[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			//prąd baterii
			if (nKonfLogera[0] & (KLOG1_BAT1_PRAD << 4*n))
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_BAT_XD_PRAD_A]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fPradAku[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			//energia pobrana z baterii
			if (nKonfLogera[0] & (KLOG1_BAT1_ENER << 4*n))
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_BAT_XD_ENER_POBR_MAH]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fEnergiaPobr[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}

			//napięcie wejściowe zasilania
			if (nKonfLogera[0] & (KLOG1_ZAS1_NAP << 4*n))
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_ZASIL_XD_NAPIECIE_V]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fNapiecieWej[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		for (uint8_t n=0; n<4; n++)	//pętla dla 4 czujników zewnętrznych
		{
			//wejście analogowe
			if (nKonfLogera[n] & (KLOG1_ADC1_1 << n))
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_CZUJ_ZEWN_XD_V]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowany w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.fNapCzujZewn[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura CPU
		if (nKonfLogera[0] & KLOG1_TEMP_CPU)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_TEMP_CPU_K]);
			else
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.fTemperCPU);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//napięcie magistrali serw
		if (nKonfLogera[0] & KLOG1_NAP_SERW)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_SERWA_NAPIECIE_V]);
			else
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.fNapiecieSerw);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}


//--- drugie słowo konfiguracji logera --------------------------
		//surowa prędkość obrotowa P żyroskopu 1
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi żyroskopu
		{
			if (nKonfLogera[1] & KLOG2_ZYROSUR1P << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_ZYRO_SUR_XD_XC_RADS]);
					sprintf(cBufPodreczny, cBufPodreczny, 1, 'P' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur1[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//surowa prędkość obrotowa P żyroskopu 2
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi żyroskopu
		{
			if (nKonfLogera[1] & KLOG2_ZYROSUR2P << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_ZYRO_SUR_XD_XC_RADS]);
					sprintf(cBufPodreczny, cBufPodreczny, 2, 'P' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroSur2[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa P żyroskopu 1
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi żyroskopu
		{
			if (nKonfLogera[1] & KLOG2_ZYRO1P << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_ZYRO_KAL_XD_XC_RADS]);
					sprintf(cBufPodreczny, cBufPodreczny, 1, 'P' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal1[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//skalibrowana prędkość obrotowa P żyroskopu 2
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi żyroskopu
		{
			if (nKonfLogera[1] & KLOG2_ZYRO2P << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_ZYRO_KAL_XD_XC_RADS]);
					sprintf(cBufPodreczny, cBufPodreczny, 2, 'P' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fZyroKal2[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

	//---------------------------
		//przyspieszenie w osi X akcelerometru 1
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi akcelerometru
		{
			if (nKonfLogera[1] & KLOG2_AKCEL1X << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_AKCEL_XD_XC_MS2]);
					sprintf(cBufPodreczny, cBufPodreczny, 1, 'X' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel1[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//przyspieszenie w osi X akcelerometru 2
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi akcelerometru
		{
			if (nKonfLogera[1] & KLOG2_AKCEL2X << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_AKCEL_XD_XC_MS2]);
					sprintf(cBufPodreczny, cBufPodreczny, 2, 'X' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fAkcel2[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi X magnetometru 1
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi magnetometru
		{
			if (nKonfLogera[1] & KLOG2_MAG1X << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_MAGNETO_XD_XC_GAUSS]);
					sprintf(cBufPodreczny, cBufPodreczny, 1, 'X' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie

				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fMagne1[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi X magnetometru 2
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi magnetometru
		{
			if (nKonfLogera[1] & KLOG2_MAG2X << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_MAGNETO_XD_XC_GAUSS]);
					sprintf(cBufPodreczny, cBufPodreczny, 2, 'X' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fMagne2[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//składowa magnetyczna w osi Y magnetometru 3
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 osi magnetometru
		{
			if (nKonfLogera[1] & KLOG2_MAG3X << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_MAGNETO_XD_XC_GAUSS]);
					sprintf(cBufPodreczny, cBufPodreczny, 3, 'X' + n);	//wypełnij parametry XD=%d i XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fMagne3[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//temperatura IMU1
		for (uint8_t n=0; n<2; n++)	//pętla dla czujników
		{
			if (nKonfLogera[1] & KLOG2_TEMPIMU1 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_TEMP_IMU_XD_K]);
					sprintf(cBufPodreczny, cBufPodreczny, n+1);	//wypełnij parametr XD=%d zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.fTemper[TEMP_IMU1 + n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


//--- trzecie słowo konfiguracji logera --------------------------

		//kąty wektora inercji dla BSP uzyskane z filtra Kalmana
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 katów IMU
		{
			if (nKonfLogera[2] & KLOG3_BSP_IMUX << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_KAT_KALM_IMU_XC_RAD]);
					sprintf(cBufPodreczny, cBufPodreczny,  'X' + n);	//wypełnij parametr XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.6f;", uDaneCM4.dane.stBSP.fKatIMU[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąty wektora inercji uzyskane z filtra komplementarnego (całka z żyroskopów + trygonometria z akcelerometrów)
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 katów IMU
		{
			if (nKonfLogera[2] & KLOG3_KOMP_IMUX << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_KAT_KOMP_IMU_XC_RAD]);
					sprintf(cBufPodreczny, cBufPodreczny,  'X' + n);	//wypełnij parametr XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.6f;", uDaneCM4.dane.fKatIMU1[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąty wektora inercji uzyskane w wyniku obliczeń na kwaternionach
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 katów IMU
		{
			if (nKonfLogera[2] & KLOG3_KWAT_IMUX << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_KAT_KWAT_IMU_XC_RAD]);
					sprintf(cBufPodreczny, cBufPodreczny,  'X' + n);	//wypełnij parametr XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.6f;", uDaneCM4.dane.fKatIMU2[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąty wektora inercji uzyskane w wyniku obliczeń trygonometrycznych na wektorach akcelerometrów
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 katów IMU
		{
			if (nKonfLogera[2] & KLOG3_KWAT_IMUX << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_KAT_AKCE_IMU_XC_RAD]);
					sprintf(cBufPodreczny, cBufPodreczny,  'X' + n);	//wypełnij parametr XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.6f;", uDaneCM4.dane.fKatAkcel1[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kąty wektora inercji uzyskane w wyniku całkowania prędkosci katowych z żyroskopów
		for (uint8_t n=0; n<3; n++)	//pętla dla 3 katów IMU
		{
			if (nKonfLogera[2] & KLOG3_KWAT_IMUX << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				{
					sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_KAT_ZYRO_IMU_XC_RAD]);
					sprintf(cBufPodreczny, cBufPodreczny,  'X' + n);	//wypełnij parametr XC=%c zakodowane w nazwie
				}
				else
					sprintf(cBufPodreczny, "%.6f;", uDaneCM4.dane.fKatZyro1[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}



	//----------------- GNSS --------------------------------
		//Szerokość geograficzna z GPS
		if (nKonfLogera[2] & KLOG3_GLONG)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_SZEROKOSC_GEO_RAD]);
			else
				sprintf(cBufPodreczny, "%.8f;", uDaneCM4.dane.stGnss1.dSzerokoscGeo);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//Długość geograficzna z GPS
		if (nKonfLogera[2] & KLOG3_GLATI)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_DLUGOSC_GEO_RAD]);
			else
				sprintf(cBufPodreczny, "%.8f;", uDaneCM4.dane.stGnss1.dDlugoscGeo);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wysokość n.p.m. z GPS
		if (nKonfLogera[2] & KLOG3_GALTI)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_WYSOKOSC_GNSS_M]);
			else
				sprintf(cBufPodreczny, "%.1f;", uDaneCM4.dane.stGnss1.fWysokoscMSL);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//prędkość wzgledem ziemi z GPS
		if (nKonfLogera[2] & KLOG3_GSPED)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_PREDKOSC_WZGL_ZIEMI_MS]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//kurs względem ziemi z GPS
		if (nKonfLogera[2] & KLOG3_GCURS)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_KURS_GNSS_RAD]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stGnss1.fKurs);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//liczba widocznych satelitów
		if (nKonfLogera[2] & KLOG3_GSATS)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_LICZBA_SATELITOW]);
			else
				sprintf(cBufPodreczny, "%d;", uDaneCM4.dane.stGnss1.cLiczbaSatelit);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//Vertical Dilution of Precision
		if (nKonfLogera[2] & KLOG3_GVDOP)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_VDOP_M]);
			else
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fVdop);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//Horizontal Dilution of Precision
		if (nKonfLogera[2] & KLOG3_GHDOP)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_HDOP_M]);
			else
				sprintf(cBufPodreczny, "%.2f;", uDaneCM4.dane.stGnss1.fHdop);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//niefiltrowana prędkość z GPS w kierunku północnym
		if (nKonfLogera[2] & KLOG3_GSPD_N)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_PREDK_GNSS_N_MS]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi * cosf(uDaneCM4.dane.stGnss1.fKurs * DEG2RAD));		//sprawdzić!
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//niefiltrowana prędkość z GPS w kierunku wschodnim
		if (nKonfLogera[2] & KLOG3_GSPD_E)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s;", cNazwyPozycjiRejestratora[NREJ_PREDK_GNSS_E_MS]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi * sinf(uDaneCM4.dane.stGnss1.fKurs * DEG2RAD));		//sprawdzić!
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

//--- czwarte słowo konfiguracji logera --------------------------
		//kanały 1..KANALY_ODB_RC odbiornika RC zajmują pierwsze 16 bitów słowa konfiguracji

		for (uint8_t n=0; n<KANALY_ODB_RC; n++)
		{
			if (nKonfLogera[3] & (KLOG4_ODBRC_K1 << n))
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					sprintf(cBufPodreczny, "%s%d;", cNazwyPozycjiRejestratora[NREJ_ODBIORNIKRC_KAN], n+1);
				else
					sprintf(cBufPodreczny, "%d;", uDaneCM4.dane.sKanalRC[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		//kanały 1..KANALY_WYJSC_RC wyjść RC zajmują ostatnie 16 bitów słowa konfiguracji
		for (uint8_t n=0; n<KANALY_WYJSC_RC; n++)
		{
			if (nKonfLogera[3] & (KLOG4_WYJRC_K1 << n))
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					sprintf(cBufPodreczny, "%s%d;", cNazwyPozycjiRejestratora[NREJ_WYJSCIERC_KAN], n+1);
				else
					sprintf(cBufPodreczny, "%d;", uDaneCM4.dane.sWyjscieRC[n]);
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}


//--- piąte słowo konfiguracji logera --------------------------

		//wartość zadana regulatora sterowania przechyleniem
		if (nKonfLogera[4] & KLOG5_PID_PRZE_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość wejściowa dla wszystkich członów
		if (nKonfLogera[4] & KLOG5_PID_PRZE_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[4] & KLOG5_PID_PRZE_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[4] & KLOG5_PID_PRZE_WY_P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu I
		if (nKonfLogera[4] & KLOG5_PID_PRZE_WY_I)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJ_I]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fWyjscieI);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[4] & KLOG5_PID_PRZE_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[4] & KLOG5_PID_PRZE_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania przechyleniem
		if (nKonfLogera[4] & KLOG5_PID_PRZE_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_PRZE].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wartość zadana regulatora sterowania prędkością kątową przechylenia
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_FZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_FILTR_WZAD]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fFiltrWZad);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..15) wartość wejściowa dla wszystkich członów
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_WY_P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania prędkością kątową przechylenia
		if (nKonfLogera[4] & KLOG5_PID_PK_PRZE_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_PRZE], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_PRZE].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wartość zadana regulatora sterowania pochyleniem
		if (nKonfLogera[4] & KLOG5_PID_POCH_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość wejściowa dla wszystkich członów
		if (nKonfLogera[4] & KLOG5_PID_POCH_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[4] & KLOG5_PID_POCH_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[4] & KLOG5_PID_POCH_WY_P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu I
		if (nKonfLogera[4] & KLOG5_PID_POCH_WY_I)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_WYJ_I]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWyjscieI);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[4] & KLOG5_PID_POCH_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[4] & KLOG5_PID_POCH_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania pochyleniem
		if (nKonfLogera[4] & KLOG5_PID_POCH_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_POCH], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_POCH].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wartość zadana regulatora sterowania prędkością kątową pochylenia
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_FZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_FILTR_WZAD]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fFiltrWZad);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość wejściowa dla wszystkich członów
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania prędkością kątową pochylenia
		if (nKonfLogera[4] & KLOG5_PID_PK_POCH_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_POCH], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_POCH].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}


//--- szóste słowo konfiguracji logera --------------------------

		//wartość zadana regulatora sterowania odchyleniem
		if (nKonfLogera[5] & KLOG6_PID_ODCH_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość wejściowa dla wszystkich członów
		if (nKonfLogera[5] & KLOG6_PID_ODCH_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[5] & KLOG6_PID_ODCH_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[5] & KLOG6_PID_ODCH_WY_P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu I
		if (nKonfLogera[5] & KLOG6_PID_ODCH_WY_I)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJ_I]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fWyjscieI);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[5] & KLOG6_PID_ODCH_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[5] & KLOG6_PID_ODCH_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania odchyleniem
		if (nKonfLogera[5] & KLOG6_PID_ODCH_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_KATA_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_KĄTA_ODCH].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wartość zadana regulatora sterowania prędkością kątową odchylenia
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość zadana do liczenia wartosci wyprzedzającej
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_FZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_FILTR_WZAD]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fFiltrWZad);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość wejściowa dla wszystkich członów
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_WY_P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania prędkością kątową odchylenia
		if (nKonfLogera[5] & KLOG6_PID_PK_ODCH_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PRED_ODCH], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ODCH].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wartość zadana regulatora sterowania wysokością
		if (nKonfLogera[5] & KLOG6_PID_WYSO_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość wejściowa dla wszystkich członów
		if (nKonfLogera[5] & KLOG6_PID_WYSO_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[5] & KLOG6_PID_WYSO_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[5] & KLOG6_PID_WYSO_WY_P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu I
		if (nKonfLogera[5] & KLOG6_PID_WYSO_WY_I)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_WYJ_I]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWyjscieI);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[5] & KLOG6_PID_WYSO_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[5] & KLOG6_PID_WYSO_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania odchyleniem
		if (nKonfLogera[5] & KLOG6_PID_WYSO_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_WYSOKOSCI], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_WYSOKOSCI].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wartość zadana regulatora prędkości zmiany wysokości
		if (nKonfLogera[5] & KLOG6_PID_PR_WYSO_WZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_WART_ZADANA]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fZadana);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość zadana dla członu wyprzedzenia
		if (nKonfLogera[5] & KLOG6_PID_PR_WYSO_FZAD)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_FILTR_WZAD]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fFiltrWZad);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana wartość wejściowa dla wszystkich członów
		if (nKonfLogera[5] & KLOG6_PID_WYSO_FWEJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_FILTR_WWEJ]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fFiltrWWej);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//przefiltrowana (0..255) wartość wejściowa dla członu różniczkującego
		if (nKonfLogera[5] & KLOG6_PID_PK_WYSO_FROZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_FILTR_ROZN]);
			else
				sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fFiltrRóżn);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu P
		if (nKonfLogera[5] & KLOG6_PID_PR_WYSO_WY_P)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_WYJ_P]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fWyjscieP);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu D
		if (nKonfLogera[5] & KLOG6_PID_PR_WYSO_WY_D)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_WYJ_D]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fWyjscieD);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście członu wyprzedzającego
		if (nKonfLogera[5] & KLOG6_PID_PR_WYSO_WYPRZ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_WYJ_WYPRZ]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fWyjscieWyprz);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//wyjście regulatora sterowania prędkością zmiany wysokości
		if (nKonfLogera[5] & KLOG6_PID_PR_WYSO_WYJ)
		{
			if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				sprintf(cBufPodreczny, "%s.%s;", cNazwyPozycjiRejestratora[NREJ_REG_PR_ZM_WYS], cNazwyPozycjiRejestratora[NREJ_WYJSCIE]);
			else
				sprintf(cBufPodreczny, "%.3f;", uDaneCM4.dane.stPID[PID_PRED_ZWYS].fWyjsciePID);
			strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
		}

		//miejsce na rejestrację regulatorów nawigacyjnych


		//ósme słowo konfiguracji logera - filtry Kalmana
		for (uint8_t n=0; n<4; n++)
		{
			if (nKonfLogera[7] & KLOG8_KALWYS_X0 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					sprintf(cBufPodreczny, "%sX[%d];", cNazwyPozycjiRejestratora[NREJ_KALMAN_WYS], n);
				else
					sprintf(cBufPodreczny, "%.4f;", uDaneCM4.dane.stKalmanDebug.fX[n]);	//wektor stanu
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		for (uint8_t n=0; n<4; n++)
		{
			if (nKonfLogera[7] & KLOG8_KALWYS_K0 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					sprintf(cBufPodreczny, "%sK[%d];", cNazwyPozycjiRejestratora[NREJ_KALMAN_WYS], n);
				else
					sprintf(cBufPodreczny, "%E;", uDaneCM4.dane.stKalmanDebug.fK[n]);	//główne elementy wzmocnienia
				strncat(cBufZapisuKarty, cBufPodreczny, MAX_ROZMIAR_WPISU_LOGU);
			}
		}

		for (uint8_t n=0; n<4; n++)
		{
			if (nKonfLogera[7] & KLOG8_KALWYS_P0 << n)
			{
				if (cStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
					sprintf(cBufPodreczny, "%sP[%d];", cNazwyPozycjiRejestratora[NREJ_KALMAN_WYS], n);
				else
					sprintf(cBufPodreczny, "%E;", uDaneCM4.dane.stKalmanDebug.fP[n]);	//główna przekątna wariancji procesu
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
		{
			cBufZapisuKarty[ROZMIAR_BUFORA_LOGU - 3] = 0;
			cBłąd = BLAD_BUF_OVERRUN;
		}
		strncat(cBufZapisuKarty, "\n", 2);	//znak końca wiersza

		sZapisanoLogu = f_puts(cBufZapisuKarty, &SDFile);	//zapis do pliku

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
		sprintf(cBufPodreczny, "%04d%02d%02d_%02d%02d%02d_%s.csv",stDate.Year+2000, stDate.Month, stDate.Date, stTime.Hours, stTime.Minutes, stTime.Seconds, stBSP_ID.cNazwa);
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

	return cBłąd;
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
