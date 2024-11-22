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


uint16_t sFlashMem[100] __attribute__((section(".FlashNorSection")));
extern NOR_HandleTypeDef hnor3;



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


void Test_Flash(void)
{
	HAL_StatusTypeDef Err;
	uint16_t x, bufor[256];
	uint32_t nAdres;

	//*( (uint16_t *)ADR_NOR + 0x55 ) = 0x0098; 	// write CFI entry command
	Err = HAL_NOR_ReturnToReadMode(&hnor3);
	nAdres = 0;
	//Err = HAL_NOR_Erase_Chip(&hnor3, nAdres);	//nie działa

	for (x=0; x<100; x++)
		bufor[x] = sFlashMem[x];

	nAdres = 20;
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

	//*( (uint16_t *)ADR_NOR + 0x000 ) = 0x00F0; 	// write cfi exit command

	bufor[99] = 0;
}



