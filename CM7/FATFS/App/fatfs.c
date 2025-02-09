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
#include <stdio.h>
#include "ff_gen_drv.h"
#include "sdram_diskio.h"
#include "sd_diskio.h"

/* USER CODE END Header */
#include "fatfs.h"

uint8_t retSDRAMDISK;    /* Return value for SDRAMDISK */
char SDRAMDISKPath[4];   /* SDRAMDISK logical drive path */
FATFS SDRAMDISKFatFS;    /* File system object for SDRAMDISK logical drive */
FIL SDRAMDISKFile;       /* File object for SDRAMDISK */
uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */
/* USER CODE BEGIN Variables */


/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the SDRAMDISK driver ###########################*/
  retSDRAMDISK = FATFS_LinkDriver(&SDRAMDISK_Driver, SDRAMDISKPath);
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */



  /* USER CODE END Init */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
