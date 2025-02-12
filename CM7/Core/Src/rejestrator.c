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
ALIGN_32BYTES(static char chBufPodreczny[25]);
UINT nDoZapisuNaKarte, nZapisanoNaKarte;

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

	if ((chPorty_exp_odbierane[0] & EXP04_LOG_CARD_DET)	== 0)	//LOG_SD1_CDETECT - wejście detekcji obecności karty
	{

		if (chStatusRejestratora & STATREJ_FAT_GOTOWY)
		{

			//czas
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Czas [g:m:s.ss];", MAX_ROZMIAR_WPISU);
			else
			{
				sprintf(chBufPodreczny, "%02d:%02d:%02d;", uDaneCM4.dane.stGnss1.chGodz,  uDaneCM4.dane.stGnss1.chMin,  uDaneCM4.dane.stGnss1.chSek);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
			}

			//Wysokość czujnika 1
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Wysokość1 [m];", MAX_ROZMIAR_WPISU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fWysoko[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
			}

			//akcelerometr1 X
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel1 X [g];", MAX_ROZMIAR_WPISU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fAkcel1[0]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
			}

			//akcelerometr1 Y
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel1 Y [g];", MAX_ROZMIAR_WPISU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fAkcel1[1]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
			}
			//akcelerometr1 XZ
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				strncat(chBufZapisuKarty, "Akcel1 Z [g];", MAX_ROZMIAR_WPISU);
			else
			{
				sprintf(chBufPodreczny, "%.2f;", uDaneCM4.dane.fAkcel1[2]);
				strncat(chBufZapisuKarty, chBufPodreczny, MAX_ROZMIAR_WPISU);
			}

			//jeżeli był zapisywany nagłówek to przejdź do zapisu danych
			if (chStatusRejestratora & STATREJ_ZAPISZ_NAGLOWEK)
				chStatusRejestratora &= ~ STATREJ_ZAPISZ_NAGLOWEK;
			strncat(chBufZapisuKarty, "\n\r", 3);	//znak końca linii
			f_puts(chBufZapisuKarty, &SDFile);	//zapis do pliku
			chBufZapisuKarty[0] = 0;	//ustaw 0 na początku bufora
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
					sprintf(chBufPodreczny, "%04d-%02d-%02d_APL3.csv", uDaneCM4.dane.stGnss1.chRok+2000, uDaneCM4.dane.stGnss1.chMies, uDaneCM4.dane.stGnss1.chDzien);
					fres = f_open(&SDFile, chBufPodreczny, FA_OPEN_ALWAYS | FA_WRITE);
					if (fres == FR_OK)
						chStatusRejestratora |= STATREJ_FAT_GOTOWY | STATREJ_ZAPISZ_NAGLOWEK;
				}
				else	//jeżeli nie udało sie zamontować FAT to utwórz go ponownie
				{
					DWORD au = _MAX_SS;
					fres = f_mkfs(SDPath, FM_FAT32, au, aTxBuffer, sizeof(aTxBuffer));	//sprawdzić czy tak może być
				}
			}
		}
	}
	else	//jeżeli nie ma karty
	{
		if (chStatusRejestratora & STATREJ_FAT_GOTOWY)
		{
			f_close(&SDFile);
			f_mount(NULL, "", 1);		//zdemontuj system plików
			chStatusRejestratora = 0;
		}
	}

	//sprawdź czy nie wyłączono rejestratoraw czasie pracy
	if ((chStatusRejestratora & STATREJ_WLACZONY) == 0)
	{
		if (chStatusRejestratora & STATREJ_OTWARTY_PLIK)
			f_close(&SDFile);
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


