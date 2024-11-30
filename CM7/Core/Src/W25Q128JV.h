/*
 * W25Q128JV.h
 *
 *  Created on: Nov 28, 2024
 *      Author: PitLab
 */

#ifndef SRC_W25Q128JV_H_
#define SRC_W25Q128JV_H_


#include "sys_def_CM7.h"



//kody instrukcji 						Byte 1	Byte 2 	Byte 3 	Byte 4 	Byte 5 	Byte 6 	Byte 7
#define CMD_W25Q_Write_Enable 			0x06
#define CMD_W25Q_Volatile_SR_WR_EN 		0x50
#define CMD_W25Q_Write_Disable 			0x04
#define CMD_W25Q_Release_Power_down_ID 	0xAB 	//Dummy 	Dummy 	Dummy 	(ID7-0)
#define CMD_W25Q_Manufacturer_Device_ID 0x90 	//Dummy 	Dummy 	00h 	(MF7-0) (ID7-0)
#define CMD_W25Q_JEDEC_ID 				0x9F 	//(MF7-0) (ID15-8) (ID7-0)
#define CMD_W25Q_Read_Unique_ID 		0x4B 	//Dummy Dummy Dummy Dummy (UID63-0)
#define CMD_W25Q_Read_Data 				0x03 	//A23-A16 A15-A8 A7-A0 (D7-D0)
#define CMD_W25Q_Fast_Read 				0x0B 	//A23-A16 A15-A8 A7-A0 Dummy (D7-D0)
#define CMD_W25Q_Page_Program 			0x02 	//A23-A16 A15-A8 A7-A0 D7-D0 D7-D0(3)
#define CMD_W25Q_Sector_Erase_4KB 		0x20 	//A23-A16 A15-A8 A7-A0
#define CMD_W25Q_Block_Erase_32KB 		0x52 	//A23-A16 A15-A8 A7-A0
#define CMD_W25Q_Block_Erase_64KB 		0xD8 	//A23-A16 A15-A8 A7-A0
#define CMD_W25Q_Chip_Erase1 			0xC7
#define CMD_W25Q_Chip_Erase2 			0x60
#define CMD_W25Q_RD_Status_Register1 	0x05 	//(S7-S0)(2)
#define CMD_W25Q_WR_Status_Register1	0x01 	//(S7-S0)(4)
#define CMD_W25Q_RD_Status_Register2 	0x35	//(S15-S8)(2)
#define CMD_W25Q_WR_Status_Register2 	0x31 	//(S15-S8)
#define CMD_W25Q_RD_Status_Register3 	0x15 	//(S23-S16)(2)
#define CMD_W25Q_WR_Status_Register3 	0x11 	//(S23-S16)
#define CMD_W25Q_RD_SFDP_Register 		0x5A	 	//00 00 A7-A0 Dummy (D7-D0)
#define CMD_W25Q_Erase_Security_Reg 	0x44 	//A23-A16 A15-A8 A7-A0
#define CMD_W25Q_Program_Security_Reg	0x42 	//A23-A16 A15-A8 A7-A0 D7-D0 D7-D0(3)
#define CMD_W25Q_Read_Security_Register 0x48 	//A23-A16 A15-A8 A7-A0 Dummy (D7-D0)
#define CMD_W25Q_Global_Block_Lock 		0x7E
#define CMD_W25Q_Global_Block_Unlock 	0x98
#define CMD_W25Q_Read_Block_Lock 		0x3D 	//A23-A16 A15-A8 A7-A0 (L7-L0)
#define CMD_W25Q_Individ_Block_Lock 	0x36 	//A23-A16 A15-A8 A7-A0
#define CMD_W25Q_Individ_Block_Unlock 	0x39 	//A23-A16 A15-A8 A7-A0
#define CMD_W25Q_Erase_Program_Suspend 	0x75
#define CMD_W25Q_Erase_Program_Resume 	0x7A
#define CMD_W25Q_Power_down 			0xB9
#define CMD_W25Q_Enable_Reset 			0x66
#define CMD_W25Q_Reset_Device 			0x99

//Number of Clock 8 8 8 8 2 2 2 2 2
#define CMD_W25Q_Quad_In_Page_Program 	0x32 	//A23-A16 A15-A8 A7-A0 (D7-D0)(9) (D7-D0)(3) …
#define CMD_W25Q_Fast_Read_Quad_Out 	0x6B 	//A23-A16 A15-A8 A7-A0 Dummy Dummy Dummy Dummy (D7-D0)(10)

//Number of Clock 8 2 2 2 2 2 2 2 2
#define CMD_W25Q_Mftr_Device_ID_Quad	0x94 	//A23-A16 A15-A8 00 Dummy(11) Dummy Dummy (MF7-MF0) (ID7-ID0)
#define CMD_W25Q_Fast_Read_Quad			0xEB 	//A23-A16 A15-A8 A7-A0 Dummy(11) Dummy Dummy (D7-D0)
#define CMD_W25Q_Set_Burst_with_Wrap 	0x77 	//Dummy Dummy Dummy W8-W0


//czasy timeoutu programowania
#define TOUT_PAGE_PROGRAM				3	//timeout programowania strony 3ms
#define TOUT_STATUS_REG_PROGRAM			15	//timeout programowania rejestru statusu 15ms
#define TOUT_SECTOR4K_ERASE				400	//timeout kasowania sektora 4kB 400ms

//bity rejestru statusu 1
#define STATUS1_BUSY					0x01
#define STATUS1_WEL						0x02
#define STATUS1_BLOCK_PROTECT			0x1C
#define STATUS1_TOP_BOT_PROTECT			0x20
#define STATUS1_SECTOR_PROTECT			0x40
#define STATUS1_STATUS_REGISTER_PROTECT	0x80



uint8_t InicjujFlashQSPI(void);
uint8_t W25_TestTransferu(void);
uint8_t W25_SprawdzObecnoscFlashQSPI(void);
uint8_t W25_CzytajStatus(uint8_t chTypStatusu, uint8_t* chStatus);
uint8_t W25_CzytajDane1A1D(uint32_t nAdres, uint8_t* dane, uint16_t ilosc);
uint8_t W25_CzytajDane4A4D(uint32_t nAdres, uint8_t* dane, uint16_t ilosc);
uint8_t W25_UstawWriteEnable(void);
uint8_t W25_ProgramujStrone256B(uint32_t nAdres, uint8_t* dane, uint16_t ilosc);
uint8_t W25_KasujSektor4kB(uint32_t nAdres);


#endif /* SRC_W25Q128JV_H_ */
