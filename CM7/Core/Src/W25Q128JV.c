//////////////////////////////////////////////////////////////////////////////
//
// Sterownik pamięci Flash W25Q128JV na magistrali QSPI 133MHz
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include "W25Q128JV.h"
#include "LCD.h"
#include "RPi35B_480x320.h"
#include "flash_nor.h"


extern QSPI_HandleTypeDef hqspi;
extern char chNapis[];
extern struct _statusDotyku statusDotyku;


/* Each command can include five phases: instruction, address, alternate byte, dummy, data. Any of these phases
can be configured to be skipped, but at least one of the instruction, address, alternate byte, or data phase must be present.
 *typedef struct
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




/*	  uint32_t Match;              Specifies the value to be compared with the masked status register to get a match.
	                               This parameter can be any value between 0 and 0xFFFFFFFF
	  uint32_t Mask;               Specifies the mask to be applied to the status bytes received.
	                               This parameter can be any value between 0 and 0xFFFFFFFF
	  uint32_t Interval;           Specifies the number of clock cycles between two read during automatic polling phases.
	                               This parameter can be any value between 0 and 0xFFFF
	  uint32_t StatusBytesSize;    Specifies the size of the status bytes received.
	                               This parameter can be any value between 1 and 4
	  uint32_t MatchMode;          Specifies the method used for determining a match.
	                               This parameter can be a value of @ref QSPI_MatchMode
	  uint32_t AutomaticStop;      Specifies if automatic polling is stopped after a match.
	                               This parameter can be a value of @ref QSPI_AutomaticStop */

uint8_t InicjujFlashQSPI(void)
{
	HAL_StatusTypeDef chErr = 0;
	extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu

	if (W25_SprawdzObecnoscFlashQSPI() == ERR_OK)
		nZainicjowano[0] |= INIT0_FLASH_QSPI;
	return chErr;

}

////////////////////////////////////////////////////////////////////////////////
// Funkcja sprawdza czy została wykryta pamięć Flash W25Q128JV
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_SprawdzObecnoscFlashQSPI(void)
{
	HAL_StatusTypeDef chErr;
	QSPI_CommandTypeDef cmd;
	uint8_t chBufor[2];

	//faza instrukcji - obecna
	cmd.Instruction = CMD_W25Q_Manufacturer_Device_ID;
	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;

	//faza adresu - obecna 3B
	cmd.Address = 0x000000;
	cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;

	//faza alternate byte - nieobecna
	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	cmd.AlternateBytes = 0;

	//faza dummy - nieobecna
	cmd.DummyCycles = 0;

	//faza data - obecna 2B
	cmd.DataMode = QSPI_DATA_1_LINE;	//Data on a single line
	cmd.NbData = 2;

	//inne niezbędne parametry
	cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	cmd.DdrHoldHalfCycle = 0;

	chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	if (chErr == HAL_OK)
	{
		//Teraz trzeba odczytać 2 bajty:	(MF7-0) (ID7-0)
		chErr = HAL_QSPI_Receive(&hqspi, chBufor, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
		if (chErr == HAL_OK)
		{
			if ((chBufor[0] != 0xEF) || (chBufor[1] != 0x17))		//pdf str 22
				chErr = ERR_BRAK_FLASH_NOR;
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Czyta jeden z 3 rejestrów statusu
// Parametry:
//  chTypStatusu - identyfikator statusu
//  Status* - wskaźnik na odczytaną zawartosć rejestru
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_CzytajStatus(uint8_t chTypStatusu, uint8_t* chStatus)
{
	HAL_StatusTypeDef chErr;
		QSPI_CommandTypeDef cmd;

		//faza instrukcji - obecna
		cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
		switch (chTypStatusu)
		{
		case 1:	cmd.Instruction = CMD_W25Q_RD_Status_Register1;	break;
		case 2: cmd.Instruction = CMD_W25Q_RD_Status_Register2;	break;
		case 3:	cmd.Instruction = CMD_W25Q_RD_Status_Register3;	break;
		default: return ERR_ZLE_POLECENIE;
		}

		//faza adresu - brak
		cmd.AddressMode = QSPI_ADDRESS_NONE;

		//faza alternate byte - nieobecna
		cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;

		//faza dummy - nieobecna
		cmd.DummyCycles = 0;

		//faza data - obecna 1bajt
		cmd.DataMode = QSPI_DATA_1_LINE;	//Data on a single line
		cmd.NbData = 1;

		//inne parametry
		cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
		//cmd.SIOOMode = QSPI_SIOO_INST_ONLY_FIRST_CMD;
		cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
		cmd.DdrHoldHalfCycle = 0;

		chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
		if (chErr == HAL_OK)
		{
			//Teraz trzeba odczytać 1 bajt
			chErr = HAL_QSPI_Receive(&hqspi, chStatus, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
		}

		//HAL_Delay(20);
		return chErr;
}




////////////////////////////////////////////////////////////////////////////////
// Czyta zawartość pamięci od podanego adresu magistralą 1 przewodową
// Parametry: nAdres - adres początku danych do doczytu
// dane* - wskaźnik na odczytane dane
// ilosc - liczba danych do odczytu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_CzytajDane1A1D(uint32_t nAdres, uint8_t* dane, uint16_t ilosc)
{
	HAL_StatusTypeDef chErr;
	QSPI_CommandTypeDef cmd;

	//faza instrukcji - obecna
	cmd.Instruction = CMD_W25Q_Read_Data;
	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;

	//faza adresu - obecna 3B
	cmd.Address = nAdres;
	cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;

	//faza alternate byte - nieobecna
	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	cmd.AlternateBytes = 0;

	//faza dummy - nieobecna
	cmd.DummyCycles = 0;

	//faza data - obecna 2B
	cmd.DataMode = QSPI_DATA_1_LINE;	//Data on a single line
	cmd.NbData = ilosc;

	//inne niezbędne parametry
	cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	cmd.DdrHoldHalfCycle = 0;

	chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	if (chErr == HAL_OK)
		chErr = HAL_QSPI_Receive(&hqspi, dane, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Czyta zawartość pamięci od podanego adresu magistralą 4 przewodową
// Parametry: nAdres - adres początku danych do doczytu
// dane* - wskaźnik na odczytane dane
// ilosc - liczba danych do odczytu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_CzytajDane4A4D(uint32_t nAdres, uint8_t* dane, uint16_t ilosc)
{
	HAL_StatusTypeDef chErr;
	QSPI_CommandTypeDef cmd;

	//faza instrukcji - obecna
	cmd.Instruction = CMD_W25Q_Fast_Read_Quad;
	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;

	//faza adresu - obecna 3B
	cmd.Address = nAdres;
	cmd.AddressMode = QSPI_ADDRESS_4_LINES;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;

	//faza alternate byte - nieobecna
	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	cmd.AlternateBytes = 0;

	//faza dummy 4 cykle zegara po podaniu adresu
	cmd.DummyCycles = 4;

	//faza data - odczytaj "ilosc" danych po 4 liniach
	cmd.DataMode = QSPI_DATA_4_LINES;
	cmd.NbData = ilosc;

	//inne niezbędne parametry
	cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	cmd.DdrHoldHalfCycle = 0;

	chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	if (chErr == HAL_OK)
		chErr = HAL_QSPI_Receive(&hqspi, dane, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	return chErr;
}




////////////////////////////////////////////////////////////////////////////////
// Ustawia bit Write Enable Latch WEL w rejestrze ststusu
// wymaga wcześniejszego ustawienia Write Enable
// Parametry: nAdres - adres początku pamięci do zapisu
// dane* - wskaźnik na dane do zapisu
// ilosc - liczba danych do zapisu max 256 bajtów
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_UstawWriteEnable(void)
{
	HAL_StatusTypeDef chErr;
	QSPI_CommandTypeDef cmd;

	//faza instrukcji - obecna
	cmd.Instruction = CMD_W25Q_Write_Enable;
	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;

	//faza adresu - nieobecna
	cmd.Address = 0;
	cmd.AddressMode = QSPI_ADDRESS_NONE;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;

	//faza alternate byte - nieobecna
	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	cmd.AlternateBytes = 0;

	//faza dummy - nieobecna
	cmd.DummyCycles = 0;

	//faza data - nieobecna
	cmd.DataMode = QSPI_DATA_NONE;
	cmd.NbData = 0;

	//inne niezbędne parametry
	cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	cmd.DdrHoldHalfCycle = 0;

	chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Zapisuje zawartość strony pamięci od 1 d0 256 bajtów od podanego adresu magistralą 1 przewodową
// wymaga wcześniejszego ustawienia Write Enable
// Parametry: nAdres - adres początku pamięci do zapisu
// dane* - wskaźnik na dane do zapisu
// ilosc - liczba danych do zapisu max 256 bajtów
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_ProgramujStrone256B(uint32_t nAdres, uint8_t* dane, uint16_t ilosc)
{
	HAL_StatusTypeDef chErr;
	QSPI_CommandTypeDef cmd;
	uint8_t chStatus;
	uint32_t nCzas;

	//włacz pozwolenie na zapis. Po zaprogramowaniu strony pozwolenie samo sie wyłączy
	W25_UstawWriteEnable();
	W25_CzytajStatus(1, &chStatus);
	if (!(chStatus & STATUS1_WEL))
		return ERR_BRAK_POZW_ZAPISU;

	//faza instrukcji - obecna
	cmd.Instruction = CMD_W25Q_Page_Program;
	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;

	//faza adresu - obecna 3B
	cmd.Address = nAdres;
	cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;

	//faza alternate byte - nieobecna
	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	cmd.AlternateBytes = 0;

	//faza dummy - nieobecna
	cmd.DummyCycles = 0;

	//faza data - obecna 2B
	cmd.DataMode = QSPI_DATA_1_LINE;	//Data on a single line
	cmd.NbData = ilosc;

	//inne niezbędne parametry
	cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	cmd.DdrHoldHalfCycle = 0;

	chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	if (chErr == HAL_OK)
	{
		chErr = HAL_QSPI_Transmit(&hqspi, dane, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
		if (chErr != HAL_OK)
			return chErr;
	}

	//po zapisaniu czytaj status dopuki jest ustawiony bit Busy lub wystąpi timeout max 3ms
	nCzas = PobierzCzasT6();
	do
	{
		W25_CzytajStatus(1, &chStatus);
		if (MinalCzas(nCzas) > TOUT_PAGE_PROGRAM)
			chErr = ERR_TIMEOUT;
	}
	while ((chStatus & STATUS1_BUSY) && (!chErr));

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kasuje sektor 4kB pamięci
// wymaga wcześniejszego ustawienia Write Enable
// Parametry: nAdres - adres początku pamięci do skasowania
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_KasujSektor4kB(uint32_t nAdres)
{
	HAL_StatusTypeDef chErr;
	QSPI_CommandTypeDef cmd;
	uint8_t chStatus;
	uint32_t nCzas;

	//włącz pozwolenie na zapis. Po skasowaniu sektora pozwolenie samo sie wyłączy
	W25_UstawWriteEnable();
	W25_CzytajStatus(1, &chStatus);
	if (!(chStatus & STATUS1_WEL))
		return ERR_BRAK_POZW_ZAPISU;

	//faza instrukcji - obecna
	cmd.Instruction = CMD_W25Q_Sector_Erase_4KB;
	cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;

	//faza adresu - obecna 3B
	cmd.Address = nAdres;
	cmd.AddressMode = QSPI_ADDRESS_1_LINE;
	cmd.AddressSize = QSPI_ADDRESS_24_BITS;

	//faza alternate byte - nieobecna
	cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	cmd.AlternateBytes = 0;

	//faza dummy - nieobecna
	cmd.DummyCycles = 0;

	//faza data - nieobecna
	cmd.DataMode = QSPI_DATA_NONE;
	cmd.NbData = 0;

	//inne niezbędne parametry
	cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
	cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
	cmd.DdrHoldHalfCycle = 0;

	chErr = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	//if (chErr == HAL_OK)
	//{
		//chErr = HAL_QSPI_Transmit(&hqspi, dane, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
		//if (chErr != HAL_OK)
			//return chErr;
	//}

	//po zapisaniu czytaj status dopuki jest ustawiony bit Busy lub wystąpi timeout max 400ms
	nCzas = PobierzCzasT6();
	do
	{
		W25_CzytajStatus(1, &chStatus);
		if (MinalCzas(nCzas) > TOUT_SECTOR4K_ERASE)
			chErr = ERR_TIMEOUT;
	}
	while ((chStatus & STATUS1_BUSY) && (!chErr));

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja bezparametrowa do wygodnego wywoływania poleceń z zewnatrz
// Parametry:  brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t W25_TestTransferu(void)
{
	uint8_t chErr;
	uint16_t y;
	extern uint16_t sBufor[ROZMIAR16_BUFORA];
	uint32_t nCzas;
	extern uint8_t chRysujRaz;


	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Pomiar odczytu z Flash QSPI");
		setColor(GRAY60);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
		setColor(WHITE);
	}

	//zapisz stronę 256 Bajtów
	for (y=0; y<ROZMIAR16_BUFORA/2; y++)
		sBufor[y] = y;

	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		chErr = W25_ProgramujStrone256B(0x5000 + y*0x100, (uint8_t*)sBufor, ROZMIAR16_BUFORA/2);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd programowania", ł, ą);
			print(chNapis, 10, 80);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
		sprintf(chNapis, "ZapiszStrone256B() t = %ld us => %.2f MB/s ", nCzas, (float)(16 * ROZMIAR16_BUFORA) / (nCzas * 1.048576f));
	print(chNapis, 10, 80);


	//odczyt danych przez SPI po 1 linii
	nCzas = PobierzCzasT6();
	for (y=0; y<512; y++)
	{
		chErr = W25_CzytajDane1A1D(0x5000, (uint8_t*)sBufor, ROZMIAR16_BUFORA/2);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd odczytu", ł, ą);
			print(chNapis, 10, 100);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
		sprintf(chNapis, "CzytajDane1A1D() t = %ld us => %.2f MB/s ", nCzas, (float)(512 * ROZMIAR16_BUFORA) / (nCzas * 1.048576f));
	print(chNapis, 10, 100);


	//odczyt danych przez SPI po 4 liniach
	nCzas = PobierzCzasT6();
	for (y=0; y<512; y++)
	{
		chErr = W25_CzytajDane4A4D(0x5000, (uint8_t*)sBufor, ROZMIAR16_BUFORA/2);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd odczytu", ł, ą);
			print(chNapis, 10, 120);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
		sprintf(chNapis, "CzytajDane4A4D() t = %ld us => %.2f MB/s ", nCzas, (float)(512 * ROZMIAR16_BUFORA) / (nCzas * 1.048576f));
	print(chNapis, 10, 120);


	//kasuj sektor 4kB
	nCzas = PobierzCzasT6();
	for (y=0; y<1; y++)
	{
		chErr = W25_KasujSektor4kB(0x5000);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd kasowania", ł, ą);
			print(chNapis, 10, 140);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
		sprintf(chNapis, "KasujSektor4kB() t = %ld us => %.2f MB/s ", nCzas, (float)(4096) /  (float)(nCzas * 1.048576f));
	print(chNapis, 10, 140);

	return chErr;
}
