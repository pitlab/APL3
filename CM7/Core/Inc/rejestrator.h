/*
 * rejestrator.h
 *
 *  Created on: Feb 2, 2025
 *      Author: PitLab
 */

#ifndef INC_REJESTRATOR_H_
#define INC_REJESTRATOR_H_

#include "sys_def_CM7.h"

#define DATA_SIZE              ((uint32_t)0x0640000U)
//#define BUFFER_SIZE            ((uint32_t)0x0004000U)
#define BUFFER_SIZE            ((uint32_t)_MAX_SS)
#define NB_BUFFER              DATA_SIZE / BUFFER_SIZE
#define NB_BLOCK_BUFFER        BUFFER_SIZE / BLOCKSIZE /* Number of Block (512o) by Buffer */
#define SD_TIMEOUT             ((uint32_t)0x00100000U)
#define ADDRESS                ((uint32_t)0x00004000U) /* SD Address to write/read data */
#define DATA_PATTERN           ((uint32_t)0xB5F3A5F3U) /* Data pattern to write */


//definicja znaczenia bitów rejestrtora
#define STATREJ_FAT_GOTOWY		0x01
#define STATREJ_ZAPISZ_NAGLOWEK	0x02
#define STATREJ_WLACZONY		0x04
#define STATREJ_OTWARTY_PLIK	0x08
#define STATREJ_ZAMKNIJ_PLIK	0x10
#define STATREJ_BYL_OTWARTY		0x20

#define ROZMIAR_BUFORA_KARTY	512
#define MAX_ROZMIAR_WPISU		20

uint8_t BSP_SD_IsDetected(void);
void HAL_SD_DriveTransceiver_1_8V_Callback(FlagStatus status);
uint8_t ObslugaPetliRejestratora(void);
uint8_t Wait_SDCARD_Ready(void);

#endif /* INC_REJESTRATOR_H_ */
