//////////////////////////////////////////////////////////////////////////////
//
// Sterownik pamięci Flash W25Q128JV na magistrali QSPI 133MHz
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "W25Q128JV.h"
#include "errcode.h"
#include "sys_def_CM7.h"
#include <stdio.h>
#include "LCD.h"
#include "RPi35B_480x320.h"


extern QSPI_HandleTypeDef hqspi;


/*typedef struct
{
  uint32_t Instruction;         //Specifies the Instruction to be sent
                                //This parameter can be a value (8-bit) between 0x00 and 0xFF
  uint32_t Address;             //Specifies the Address to be sent (Size from 1 to 4 bytes according AddressSize)
                                //This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF
  uint32_t AlternateBytes;      //Specifies the Alternate Bytes to be sent (Size from 1 to 4 bytes according AlternateBytesSize)
                                //This parameter can be a value (32-bits) between 0x0 and 0xFFFFFFFF
  uint32_t AddressSize;         //Specifies the Address Size
                                //This parameter can be a value of @ref QSPI_AddressSize
  uint32_t AlternateBytesSize;  //Specifies the Alternate Bytes Size
                                //This parameter can be a value of @ref QSPI_AlternateBytesSize
  uint32_t DummyCycles;         //Specifies the Number of Dummy Cycles.
                                //This parameter can be a number between 0 and 31
  uint32_t InstructionMode;     //Specifies the Instruction Mode
                                //This parameter can be a value of @ref QSPI_InstructionMode
  uint32_t AddressMode;         //Specifies the Address Mode
                                //This parameter can be a value of @ref QSPI_AddressMode
  uint32_t AlternateByteMode;   //Specifies the Alternate Bytes Mode
                                //This parameter can be a value of @ref QSPI_AlternateBytesMode
  uint32_t DataMode;            //Specifies the Data Mode (used for dummy cycles and data phases)
                                //This parameter can be a value of @ref QSPI_DataMode
  uint32_t NbData;              //Specifies the number of data to transfer. (This is the number of bytes)
                                //This parameter can be any value between 0 and 0xFFFFFFFF (0 means undefined length until end of memory)
  uint32_t DdrMode;             //Specifies the double data rate mode for address, alternate byte and data phase
                                //This parameter can be a value of @ref QSPI_DdrMode
  uint32_t DdrHoldHalfCycle;    //Specifies if the DDR hold is enabled. When enabled it delays the data
                                //output by one half of system clock in DDR mode.
                                //This parameter can be a value of @ref QSPI_DdrHoldHalfCycle
  uint32_t SIOOMode;            //Specifies the send instruction only once mode
                                //This parameter can be a value of @ref QSPI_SIOOMode
}QSPI_CommandTypeDef;*/



////////////////////////////////////////////////////////////////////////////////
// Funkcja sprawdza czy została wykryta pamięć Flash W25Q128JV
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SprawdzObecnoscFlashQSPI(void)
{
	HAL_StatusTypeDef chErr;
	QSPI_CommandTypeDef cmd;
	uint8_t chBufor[2];

	cmd.Instruction = CMD_W25Q_Manufacturer_Device_ID;
	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
	cmd.Address = 0x000000;
	cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;
	cmd.AlternateBytes = QSPI_ALTERNATE_BYTES_NONE;
	cmd.NbData = 2;
	cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	if (chErr == HAL_OK)
	{
		//Teraz trzeba odczytać 2 bajty:	(MF7-0) (ID7-0)
		chErr = HAL_QSPI_Receive(&hqspi, chBufor, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
		if (chErr == HAL_OK)
		{
			if ((chBufor[0] != 0x90) || (chBufor[1] != 0x17))
				chErr = ERR_BRAK_FLASH_NOR;
		}
	}

	HAL_Delay(20);
	return chErr;
}
