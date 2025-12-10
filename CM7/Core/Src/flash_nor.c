//////////////////////////////////////////////////////////////////////////////
//
// Sterownik pamięci Flash S29GL256S90 NOR, 16-bit, 90ns, 512Mb = 32MB
// Sprzętowo podłaczone są adresy A0..23, A25:24 = 00
// FMC ma 4 linie CS co odpowiada adresom A27:26. Używamy NE3, co odpowiada bitom 10 więc trzeci bajt adresu to 1000b = 0x80
// Finalnie mamy do dyspozycji adresy 0x6800 0000..0x68FF FFFF w banku 1
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "flash_nor.h"
//#include <stdio.h>
//#include "LCD.h"
//#include "RPi35B_480x320.h"
//#include "czas.h"
#include "pamiec.h"

/* Dodać polecnie pooling status bit
 * Sprawdzić jak działa przełaczanie między zapisem (HAL_NOR_WriteOperation_Enable) i odczytem (HAL_NOR_ReturnToReadMode)
 * */



extern SRAM_HandleTypeDef hsram1;
extern NOR_HandleTypeDef hnor3;
extern SDRAM_HandleTypeDef hsdram1;

////////////////////////////////////////////////////////////////////////////////
// Inicjuj Flash NOR
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFlashNOR(void)
{
	uint8_t chErr;
	extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu

	//ręcznie włącz Bit 13 WAITEN: Wait enable bit oraz Bit 11 WAITCFG: Wait timing configuration,ponieważ sterownik HAL go nie włącza
	//indeksy struktury BTCR są wspólne dla rejestrów BCRx i RTRx, więc BCR3 ma indeks 4
	hnor3.Instance->BTCR[4] |= FMC_WAIT_SIGNAL_ENABLE | FMC_WAIT_TIMING_DURING_WS;

	chErr = SprawdzObecnoscFlashNOR();
	if (chErr == BLAD_OK)
		nZainicjowanoCM7 |= INIT_FLASH_NOR;

	return chErr;
}






////////////////////////////////////////////////////////////////////////////////
// Funkcja sprawdza czy została wykryta pamięć NOR Flash serii S29GL
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SprawdzObecnoscFlashNOR(void)
{
	NOR_IDTypeDef norID;
	HAL_StatusTypeDef chErr;

	chErr = HAL_NOR_Read_ID(&hnor3, &norID);
	HAL_NOR_ReturnToReadMode(&hnor3);		//ustaw pamięć w tryb odczytu
	if (chErr == HAL_OK)
	{
		if ((norID.Device_Code1 == 0x227E) &&  (norID.Device_Code3 == 0x2201) && (norID.Manufacturer_Code == 0x0001))
		{
			//jeżeli będzie trzeba to można rozróżnić wielkości pamięci
			if ((norID.Device_Code2 != 0x2228) &&	//1Gb
				(norID.Device_Code2 != 0x2223) &&	//512Mb
				(norID.Device_Code2 != 0x2222) &&	//256Mb		S29GL256S90
				(norID.Device_Code2 != 0x2221))		//128Mb
			{
				chErr = ERR_BRAK_FLASH_NOR;
			}
		}
		else
			chErr = ERR_BRAK_FLASH_NOR;
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kasuj sektor flash o rozmiarze 128kB (0xFFFF słów)
// Parametry: nAdres sektora do skasowania
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KasujSektorFlashNOR(uint32_t nAdres)
{
	uint8_t chErr;

	chErr = HAL_NOR_Erase_Block(&hnor3, ADRES_NOR, nAdres & 0x00FFFFFF);	//potrzebny jest adres względny
	if (chErr == BLAD_OK)
		chErr = HAL_NOR_GetStatus(&hnor3, nAdres, 1100);	//Sector Erase time 128 kbyte (typ/max) = 275/1100ms (pdf str.46)
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kasuj całą pamięć flash
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KasujFlashNOR(void)
{
	return HAL_NOR_Erase_Chip(&hnor3, 0);
}



////////////////////////////////////////////////////////////////////////////////
// Czytaj bufor daych z flash NOR
// Parametry: nAdres do odczytu
// * sDane - wskaźnik na 16-bitowe dane do odczytu
//  nIlosc - ilość słów do odczytu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc)
{
	return HAL_NOR_ReadBuffer(&hnor3, nAdres, sDane, nIlosc);
}



////////////////////////////////////////////////////////////////////////////////
// Zapisz dane do flash. Optymalnie jest zapisywać cały bufor 256 słów
// Parametry: nAdres do zapisu
// * sDane - wskaźnik na 16-bitowe dane do zapisu
//  nIlosc - ilość słów (nie bajtów) do zapisu max 256 słów
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc)
{
	uint8_t chErr, chStan;

	chStan = HAL_NOR_GetState(&hnor3);
	if (chStan == HAL_NOR_STATE_PROTECTED)
	{
		chErr = HAL_NOR_WriteOperation_Enable(&hnor3);
		if (chErr)
			return chErr;
	}

	chErr = HAL_NOR_ProgramBuffer(&hnor3, nAdres & 0x00FFFFFF, sDane, nIlosc);	//potrzebny jest adres względny
	if (chErr == BLAD_OK)
		chErr = HAL_NOR_GetStatus(&hnor3, nAdres, 2);	//Buffer Programming time (Typ/max) = 340/750us  (pdf str.46)

	HAL_NOR_ReturnToReadMode(&hnor3);
	return chErr;
}



/*Blank Check
The Blank Check command will confirm if the selected main flash array sector is erased. The Blank Check
command does not allow for reads to the array during the Blank Check. Reads to the array while this command
is executing will return unknown data.
To initiate a Blank Check on a sector, write 33h to address 555h in the sector, while the EAC is in the standby state
The Blank Check command may not be written while the device is actively programming or erasing or suspended.
Use the status register read to confirm if the device is still busy and when complete if the sector is blank or not.
Bit 7 of the status register will show if the device is performing a Blank Check (similar to an erase operation).
Bit 5 of the status register will be cleared to ‘0’ if the sector is erased and set to ‘1’ if not erased.
As soon as any bit is found to not be erased, the device will halt the operation and report the results.
Once the Blank Check is completed, the EAC will return to the Standby State.*/



////////////////////////////////////////////////////////////////////////////////
// Sprawdza czy sektor jest skasowany pdf.str29
// Nie działa
// Parametry: sSektor - numer sektora (0..1023)
// Zwraca: kod błędu: 0 - skasowany, 1 - zajęty
////////////////////////////////////////////////////////////////////////////////
uint8_t CzyPustySektorFNOR(uint16_t sSektor)
{
	uint32_t nAdres = ADRES_NOR + sSektor * 0x10000 + 2*0x555;
	uint8_t chStatus;

	*(volatile uint16_t*)nAdres = 0x33;	//polecenie sprawdzenia zajętości sektora

	do
		chStatus = CzytajStatusNOR(ADRES_NOR + sSektor * 0x10000);
	while (chStatus & FNOR_STATUS_DRB);
	return chStatus & FNOR_STATUS_ESB;
}


////////////////////////////////////////////////////////////////////////////////
// Odczytuje rejestr statusu. Patrz pdf. str.48
// nie działa
// Znaczenie bitów statusu:
// 7 - DRB: Device Ready Bit
// 6 = ESSB: Erase Suspend Status Bit
// 5 - ESB: Erase Status Bit
// 4 - PSB: Program Ststus Bit
// 3 - WBASB: Write Buffer Abort Status Bit
// 2 - PSSB: Program Suspend Status Bit
// 1 - SLSB: Sector Lock Status Bit
// Parametry: nAdres - adres z jakiego chcemy uzyskać status (ostatnie słowo zapisu, adres sektora)
// Zwraca: młodsze 8 bitów statusu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajStatusNOR(uint32_t nAdres)
{
	uint16_t sStatus;

	*(volatile uint16_t*)((nAdres + 0x555) | ADRES_NOR) = 0x70;
	sStatus = *(__IO uint16_t *)nAdres;
	return (uint8_t)(sStatus & 0xFF);
}


////////////////////////////////////////////////////////////////////////////////
// Czytaj ID - nie działa
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajIdNOR(uint16_t *sId)
{


	*(volatile uint16_t*)(ADRES_NOR + NOR_CMD_ADDRESS_FIRST) = NOR_CMD_DATA_FIRST;
	*(volatile uint16_t*)(ADRES_NOR + NOR_CMD_ADDRESS_SECOND) = NOR_CMD_DATA_SECOND;
	*(volatile uint16_t*)(ADRES_NOR + NOR_CMD_ADDRESS_THIRD) = NOR_CMD_DATA_AUTO_SELECT;

	for (uint8_t n=0; n<16; n++)
		sId[n] = *(__IO uint16_t *)(ADRES_NOR + n);
	return BLAD_OK;
}


////////////////////////////////////////////////////////////////////////////////
// Nadpisuje pustą funkcję _weak majacą sprawdzać zajętość pamięci przez spawdzenie stanu linii PC6/FMC_NWAIT (RY/BY we flash NOR)
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void HAL_NOR_MspWait(NOR_HandleTypeDef *hnor, uint32_t Timeout)
{
  // Prevent unused argument(s) compilation warning
  UNUSED(hnor);
  //UNUSED(Timeout);
  HAL_Delay(Timeout);
}



