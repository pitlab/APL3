//////////////////////////////////////////////////////////////////////////////
//
// Sterownik pamięci Flash S29GL256S90 NOR, 16-bit, 90ns, 512Mb = 32MB
// Podłączony jest do banku 1: 0x6000 0000..0x6FFF FFFF,
// Sprzętowo podłaczone są adresy A0..23, A25:24 = 00
// FMC ma 4 linie CS co odpowiada adresom A27:26. Używamy NE3, co odpowiada bitom 10 więc trzeci bajt pamięci to 1000b = 0x80
// Finalnie mamy do dyspozycji adresy 0x6800 0000..0x68FF FFFF
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "flash_nor.h"
#include "errcode.h"
#include "sys_def_CM7.h"
#include <stdio.h>
#include "LCD.h"
#include "RPi35B_480x320.h"

/*
  Timing.AddressSetupTime = 0;					0ns
  Timing.AddressHoldTime = 9;					45ns
  Timing.DataSetupTime = 18;
  Timing.BusTurnAroundDuration = 4;				20ns
  Timing.CLKDivision = 2;		400/2=200MHz -> 5ns
  Timing.DataLatency = 2;
  Timing.AccessMode = FMC_ACCESS_MODE_A;*/


uint16_t sFlashMem[100] __attribute__((section(".FlashNorSection")));
extern NOR_HandleTypeDef hnor3;
extern char chNapis[50];



////////////////////////////////////////////////////////////////////////////////
// Funkcja sprawdza czy została wykryta pamięć NOR Flash S29GL256S90
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SprawdzObecnoscFlashNOR(void)
{
	NOR_IDTypeDef norID;
	HAL_StatusTypeDef Err = ERR_OK;

	Err = HAL_NOR_Read_ID(&hnor3, &norID);
	if (Err == HAL_OK)
	{
		if ((norID.Device_Code1 != 0x227E) || (norID.Device_Code2 != 0x2222) || (norID.Device_Code3 != 0x2201) || (norID.Manufacturer_Code != 0x0001))
			Err = ERR_BRAK_FLASH_NOR;
	}
	return Err;
}


uint8_t Test_Flash(void)
{
	HAL_StatusTypeDef Err;
	HAL_NOR_StateTypeDef Stan;
	uint16_t x, sBufor[ROZMIAR16_BUFORA];
	//uint16_t sStrona[ROZMIAR16_STRONY];
	uint32_t y, nAdres;
	uint32_t nCzas;

	//*( (uint16_t *)ADR_NOR + 0x55 ) = 0x0098; 	// write CFI entry command
	Err = HAL_NOR_ReturnToReadMode(&hnor3);
	setColor(WHITE);

	Stan = HAL_NOR_GetState(&hnor3);
	if (Stan == HAL_NOR_STATE_PROTECTED)
	{
		Err = HAL_NOR_WriteOperation_Enable(&hnor3);
		if (Err != ERR_OK)
		{
			sprintf(chNapis, "Blad właczenia zapisu");
			print(chNapis, 10, 20);
			return Err;
		}
	}

	/*nAdres = 0;
	Err = HAL_NOR_Erase_Chip(&hnor3, nAdres);
	if (Err != ERR_OK)
	{
		sprintf(chNapis, "Blad kasowania");
		print(chNapis, 10, 40);
		return Err;
	}
	Err = HAL_NOR_ReturnToReadMode(&hnor3);*/

	//zmierz czas kasowania sektora
	/*nCzas = HAL_GetTick();
	nAdres = 2 * ROZMIAR8_SEKTORA;	//sektor 2
	Err = HAL_NOR_Erase_Block(&hnor3, 0, nAdres);
	if (Err != ERR_OK)
	{
		sprintf(chNapis, "Blad kasowania sektora");
		print(chNapis, 10, 40);
		return Err;
	}

	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Czas kasowania sektora =%ldms", nCzas);
	print(chNapis, 10, 20);*/


	//zmierz czas programowania sektora po jednej stronie
	nCzas = HAL_GetTick();
	nAdres = 26 * ROZMIAR16_SEKTORA;	//sektor
	//for (y=0; y<(ROZMIAR16_SEKTORA/ROZMIAR16_STRONY); y++)
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = x + (y<<8);

		Err = HAL_NOR_ProgramBuffer(&hnor3, nAdres, sBufor, ROZMIAR16_BUFORA);
		if (Err != ERR_OK)
		{
			sprintf(chNapis, "Blad na stronie =%ld", y);
			print(chNapis, 10, 20);
			return Err;
		}
		nAdres += ROZMIAR8_BUFORA;
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Tp bufora = %ldms, transfer = %.2f kB/s", nCzas, (float)(16 * 1000 * ROZMIAR8_BUFORA / (nCzas * 1024)));
	print(chNapis, 10, 60);
	Err = HAL_NOR_ReturnToReadMode(&hnor3);


	for(;;)
	{
		nAdres = 0;
		nCzas = HAL_GetTick();
		for (y=0; y<1000; y++)
		{
			Err = HAL_NOR_ReadBuffer(&hnor3, nAdres, sBufor, ROZMIAR16_BUFORA);
			if (Err != ERR_OK)
			{
				sprintf(chNapis, "Blad odczytu");
				print(chNapis, 10, 20);
				return Err;
			}
		}
		nCzas = MinalCzas(nCzas);
		if (nCzas)
			sprintf(chNapis, "Tr bufora = %ldms, transfer = %.2f MB/s", nCzas, (float)(1000 * 1000 * ROZMIAR8_BUFORA / (nCzas * 1024 * 1024)));
		else
			sprintf(chNapis, "za szybko! ");
		print(chNapis, 10, 80);
	}

	/*nAdres = 20;
	Err = HAL_NOR_Read(&hnor3, &nAdres, bufor);

	nAdres = 20;
	bufor[0] = 0x55;
	Err = HAL_NOR_Program(&hnor3, &nAdres, bufor);
	Err = HAL_NOR_ReturnToReadMode(&hnor3);

	nAdres = 20;
	Err = HAL_NOR_Read(&hnor3, &nAdres, bufor);

	for (x=0; x<100; x++)
		bufor[x] = sFlashMem[x];

	nAdres = 0;
	Err = HAL_NOR_ReadBuffer(&hnor3, nAdres, bufor, 256);

	Err = HAL_NOR_Erase_Block(&hnor3, 0, 0);
	Err = HAL_NOR_ReturnToReadMode(&hnor3);

	nAdres = 0;
	Err = HAL_NOR_ReadBuffer(&hnor3, nAdres, bufor, 256);

	for (x=0; x<100; x++)
		bufor[x] = x;

	nAdres = 200;
	Err = HAL_NOR_ProgramBuffer(&hnor3, nAdres, bufor, 25);

	nAdres = 200;
	Err = HAL_NOR_ReadBuffer(&hnor3, nAdres, bufor, 256);

	HAL_NOR_Erase_Block(&hnor3, 0, 0);
	Err = HAL_NOR_ReturnToReadMode(&hnor3);


	for (x=0; x<100; x++)
		bufor[x] = sFlashMem[x];

	bufor[99] = 0;*/
	return Err;
}



