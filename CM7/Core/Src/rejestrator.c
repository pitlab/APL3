/*
 * rejestrator.c
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#include "rejestrator.h"
#include "bsp_driver_sd.h"
#include "moduly_SPI.h"

#include "ff_gen_drv.h"
//#include "sdram_diskio.h"
#include "sd_diskio.h"
#include <stdio.h>
//#include "bsp_driver_sdram.h"

extern SD_HandleTypeDef hsd1;
//extern SDRAM_HandleTypeDef hsdram1;

//extern uint8_t retSDRAMDISK;    /* Return value for SDRAMDISK */
//extern char SDRAMDISKPath[4];   /* SDRAMDISK logical drive path */
//extern FATFS SDRAMDISKFatFS;    /* File system object for SDRAMDISK logical drive */
//extern FIL SDRAMDISKFile;       /* File object for SDRAMDISK */
extern uint8_t retSD;    /* Return value for SD */
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */
extern FIL SDFile;       /* File object for SD */
//ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM2")))	chBufforFAT_SD1[_MAX_SS]);
//ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM2")))	chBufforFAT_DRAM[_MAX_SS]);
ALIGN_32BYTES(uint8_t aTxBuffer[_MAX_SS]);
ALIGN_32BYTES(uint8_t aRxBuffer[_MAX_SS]);
__IO uint8_t RxCplt, TxCplt;


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
	uint8_t chErr;

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
// Montuje FAT dla definiowanych napędów
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t MontujFAT(void)
{
	DSTATUS status;
	FRESULT fres;
	char workBuffer[30];
	//DWORD au = _MAX_SS;			/* Size of allocation unit (cluster) [byte] */

	if (BSP_SD_IsDetected())		//czy karta jest w gnieździe?
	{
		hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
		status = SD_initialize(1);
		if (status == RES_OK)
		{
			//fres = f_mkfs(SDPath, FM_FAT32, au, aTxBuffer, sizeof(aTxBuffer));
			//if (fres == FR_OK)
			{
				//http://stm32f4-discovery.net/2015/08/hal-library-20-fatfs-for-stm32fxxx/
				fres = f_mount(&SDFatFS, SDPath, 1);
				if (fres == FR_OK)
				{
					fres = f_open(&SDFile, "first.txt", FA_OPEN_ALWAYS | FA_WRITE);
					if (fres == FR_OK)
					{
						sprintf(workBuffer, "Total card size: lu kBytes\n");
						f_puts(workBuffer, &SDFile);
						f_close(&SDFile);
					}
					f_mount(NULL, "", 1);		//Unmount SDCARD
				}
			}
		}
	}

	//SDRAM
/*	fres = f_mkfs(SDRAMDISKPath, FM_FAT32, au, aRxBuffer, sizeof(aRxBuffer));
	if (fres == FR_OK)
	{
		status = SDRAMDISK_initialize(0);
		if (status == RES_OK)
		{
			fres = f_mount(&SDRAMDISKFatFS, SDRAMDISKPath, 1);
			if (fres == FR_OK)
			{
				fres = f_open(&SDRAMDISKFile, "first_file.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
				if (fres == FR_OK)
				{
					sprintf(workBuffer, "Total card size: lu kBytes\n");
					f_puts(workBuffer, &SDRAMDISKFile);
					f_close(&SDRAMDISKFile);
				}
				f_mount(NULL, "", 1);
			}
		}
	}*/
	return fres;
}



////////////////////////////////////////////////////////////////////////////////
// Nadpisana funkcja weak do odczytu SDRAM. Zamieniono dostęp 32-bitowy na 16-bitowy
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
/*uint8_t BSP_SDRAM_ReadData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  uint8_t sdramstatus = SDRAM_OK;

  if(HAL_SDRAM_Read_16b(&hsdram1, (uint32_t *)uwStartAddress, pData, uwDataSize*2) != HAL_OK)
  {
    sdramstatus = SDRAM_ERROR;
  }

  return sdramstatus;
}*/



////////////////////////////////////////////////////////////////////////////////
// Nadpisana funkcja weak do zapisu SDRAM. Zamieniono dostęp 32-bitowy na 16-bitowy
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
/*uint8_t BSP_SDRAM_WriteData(uint32_t uwStartAddress, uint32_t *pData, uint32_t uwDataSize)
{
  uint8_t sdramstatus = SDRAM_OK;

  if(HAL_SDRAM_Write_16b(&hsdram1, (uint32_t *)uwStartAddress, pData, uwDataSize*2) != HAL_OK)
  {
    sdramstatus = SDRAM_ERROR;
  }

  return sdramstatus;
}*/


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


