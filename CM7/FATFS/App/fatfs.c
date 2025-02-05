/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <string.h>
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */
FRESULT fres; 		//Result after operations
BYTE readBuf[30];
//uint8_t workBuffer[_MAX_SS];
ALIGN_32BYTES(uint8_t __attribute__((section(".SekcjaSRAM2")))	workBuffer[_MAX_SS]);
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  if (BSP_SD_IsDetected())		//czy karta jest w gnieździe?
  {
	  //fres = f_mkfs(SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));	////utwórz FAT


	 // fres = f_mount(&SDFatFS, "", 1); //1=mount now
		/*if (fres == FR_OK)
		{

			fres = f_mkfs(SDPath, FM_ANY, 0, workBuffer, sizeof(workBuffer));	////utwórz FAT
			if (fres == FR_OK)
			{
				fres = f_open(&SDFile, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
				if (fres == FR_OK)
				{
					strncpy((char*)readBuf, "a new file is made!", 20);
					UINT bytesWrote;
					fres = f_write(&SDFile, readBuf, 19, &bytesWrote);
					f_close(&SDFile);
				}
			}
		}*/
	}
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  return 0;
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
