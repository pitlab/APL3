/*
 * flash_nor.h
 *
 *  Created on: Nov 20, 2024
 *      Author: PitLab
 */

#ifndef SRC_FLASH_NOR_H_
#define SRC_FLASH_NOR_H_

#include "sys_def_CM7.h"

#define ADRES_NOR			0x68000000
#define LICZBA_SEKTOROW		256
#define ROZMIAR8_SEKTORA	(128*1024)
#define ROZMIAR16_SEKTORA	(64*1024)
#define ROZMIAR8_BUFORA		512
#define ROZMIAR16_BUFORA	256
#define ROZMIAR8_STRONY		32
#define ROZMIAR16_STRONY	16
#define ROZMIAR8_EXT_SRAM	(4096*1024)
#define ROZMIAR16_EXT_SRAM	(2048*1024)
#define ROZMIAR16_BUF_SEKT	256

//znaczenie bitów statusu
#define FNOR_STATUS_DRB		0x80	// 7 - DRB: Device Ready Bit
#define FNOR_STATUS_ESSB	0x40	// 6 - ESSB: Erase Suspend Ststus Bit
#define FNOR_STATUS_ESB		0x20	// 5 - ESB: Erase Status Bit
#define FNOR_STATUS_PSB		0x10	// 4 - PSB: Program Ststus Bit
#define FNOR_STATUS_WBASB	0x08	// 3 - WBASB: Write Buffer Abort Status Bit
#define FNOR_STATUS_PSSB	0x04	// 2 - PSSB: Program Suspend Status Bit
#define FNOR_STATUS_SLSB	0x02	// 1 - SLSB: Sector Lock Status Bit


/* Constants to define address to set to write a command */
#define NOR_CMD_ADDRESS_FIRST_BYTE            (uint16_t)0x0AAA
#define NOR_CMD_ADDRESS_FIRST_CFI_BYTE        (uint16_t)0x00AA
#define NOR_CMD_ADDRESS_SECOND_BYTE           (uint16_t)0x0555
#define NOR_CMD_ADDRESS_THIRD_BYTE            (uint16_t)0x0AAA

#define NOR_CMD_ADDRESS_FIRST                 (uint16_t)0x0555
#define NOR_CMD_ADDRESS_FIRST_CFI             (uint16_t)0x0055
#define NOR_CMD_ADDRESS_SECOND                (uint16_t)0x02AA
#define NOR_CMD_ADDRESS_THIRD                 (uint16_t)0x0555
#define NOR_CMD_ADDRESS_FOURTH                (uint16_t)0x0555
#define NOR_CMD_ADDRESS_FIFTH                 (uint16_t)0x02AA
#define NOR_CMD_ADDRESS_SIXTH                 (uint16_t)0x0555

/* Constants to define data to program a command */
#define NOR_CMD_DATA_READ_RESET               (uint16_t)0x00F0
#define NOR_CMD_DATA_FIRST                    (uint16_t)0x00AA
#define NOR_CMD_DATA_SECOND                   (uint16_t)0x0055
#define NOR_CMD_DATA_AUTO_SELECT              (uint16_t)0x0090
#define NOR_CMD_DATA_PROGRAM                  (uint16_t)0x00A0
#define NOR_CMD_DATA_CHIP_BLOCK_ERASE_THIRD   (uint16_t)0x0080
#define NOR_CMD_DATA_CHIP_BLOCK_ERASE_FOURTH  (uint16_t)0x00AA
#define NOR_CMD_DATA_CHIP_BLOCK_ERASE_FIFTH   (uint16_t)0x0055
#define NOR_CMD_DATA_CHIP_ERASE               (uint16_t)0x0010
#define NOR_CMD_DATA_CFI                      (uint16_t)0x0098

#define NOR_CMD_DATA_BUFFER_AND_PROG          (uint8_t)0x25
#define NOR_CMD_DATA_BUFFER_AND_PROG_CONFIRM  (uint8_t)0x29
#define NOR_CMD_DATA_BLOCK_ERASE              (uint8_t)0x30

#define NOR_CMD_READ_ARRAY                    (uint16_t)0x00FF
#define NOR_CMD_WORD_PROGRAM                  (uint16_t)0x0040
#define NOR_CMD_BUFFERED_PROGRAM              (uint16_t)0x00E8
#define NOR_CMD_CONFIRM                       (uint16_t)0x00D0
#define NOR_CMD_BLOCK_ERASE                   (uint16_t)0x0020
#define NOR_CMD_BLOCK_UNLOCK                  (uint16_t)0x0060
#define NOR_CMD_READ_STATUS_REG               (uint16_t)0x0070
#define NOR_CMD_CLEAR_STATUS_REG              (uint16_t)0x0050

/* Mask on NOR STATUS REGISTER */
#define NOR_MASK_STATUS_DQ4                   (uint16_t)0x0010
#define NOR_MASK_STATUS_DQ5                   (uint16_t)0x0020
#define NOR_MASK_STATUS_DQ6                   (uint16_t)0x0040
#define NOR_MASK_STATUS_DQ7                   (uint16_t)0x0080

/* Address of the primary command set */
#define NOR_ADDRESS_COMMAND_SET               (uint16_t)0x0013


uint8_t InicjujFlashNOR(void);
uint8_t SprawdzObecnoscFlashNOR(void);
uint8_t KasujSektorFlashNOR(uint32_t nAdres);
uint8_t KasujFlashNOR(void);
uint8_t ZapiszDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc);
uint8_t Test_Flash(void);
uint8_t CzytajDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc);
uint8_t CzytajStatusNOR(uint32_t nAdres);
uint8_t CzytajIdNOR(uint16_t *sId);
uint8_t CzyPustySektorFNOR(uint16_t sSektor);
void HAL_NOR_MspWait(NOR_HandleTypeDef *hnor, uint32_t Timeout);

void TestPredkosciOdczytuNOR(void);
void TestPredkosciOdczytuRAM(void);


#endif /* SRC_FLASH_NOR_H_ */
