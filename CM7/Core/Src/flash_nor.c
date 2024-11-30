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
#include <stdio.h>
#include "LCD.h"
#include "RPi35B_480x320.h"

/* Pamięć jest taktowana zegarem 200 MHz (max 240MHz), daje to okres 5ns (4,1ns)
 * Włączenie FIFO, właczenie wszystkich uprawnień na MPU Cortexa nic nie daje
 *
 * Timing.AddressSetupTime = 0;					0ns
  Timing.AddressHoldTime = 9;					45ns	45/8,3 => 6
  Timing.DataSetupTime = 18;
  Timing.BusTurnAroundDuration = 4;				20ns	20/8,3 => 3
  Timing.CLKDivision = 2;		240/2=120MHz -> 8,3ns
  Timing.DataLatency = 2;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  Dla tych parametrów zapis bufor 512B ma przepustowość 258kB/s, odczyt danych 44MB/s   */




uint16_t sMPUFlash[ROZMIAR16_BUFORA] __attribute__((section(".text")));
uint16_t sFlashMem[ROZMIAR16_BUFORA] __attribute__((section(".FlashNorSection")));
extern NOR_HandleTypeDef hnor3;
extern DMA_HandleTypeDef hdma_memtomem_dma1_stream1;
extern MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream1_tc_0;
extern char chNapis[];
HAL_StatusTypeDef hsErr;



////////////////////////////////////////////////////////////////////////////////
// Inicjuj Flash NOR
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFlashNOR(void)
{
	uint8_t chErr;

	extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu

	chErr = SprawdzObecnoscFlashNOR();
	if (chErr == ERR_OK)
		nZainicjowano[0] |= INIT0_FLASH_NOR;
	HAL_NOR_ReturnToReadMode(&hnor3);		//ustaw pamięć w trub odczytu
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
	if (chErr == HAL_OK)
	{
		if ((norID.Device_Code1 == 0x227E) &&  (norID.Device_Code3 == 0x2201) && (norID.Manufacturer_Code == 0x0001))
		{
			//jeżeli będzie trzeba to można rozróżnić wielkości pamięci
			if ((norID.Device_Code2 != 0x2228) &&	//1Gb
				(norID.Device_Code2 != 0x2223) &&	//512Mb
				(norID.Device_Code2 != 0x2222) &&	//256Mb		S29GL256S90
				(norID.Device_Code2 != 0x2222))		//128Mb
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
// Kasuj sektor flash
// Parametry: nAdres sektora do skasowania
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KasujSektorFlashNOR(uint32_t nAdres)
{
	uint8_t chErr;

	nAdres &= 0x00FFFFFF;		//potrzebny jest adres względny
	chErr = HAL_NOR_Erase_Block(&hnor3, 0, nAdres);
	HAL_NOR_ReturnToReadMode(&hnor3);
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
// Zapisz dane do flash
// Parametry: nAdres do zapisu
// * sDane - wskaźnik na 16-bitoer dane do zapisu
//  nIlosc - ilość słów do zapisu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc)
{
	uint8_t chErr;

	nAdres &= 0x00FFFFFF;		//potrzebny jest adres względny
	chErr = HAL_NOR_ProgramBuffer(&hnor3, nAdres, sDane, nIlosc);
	HAL_NOR_ReturnToReadMode(&hnor3);
	return chErr;
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

	//sprawdzenie zajętości sektorów
			/*for (x=0; x<6; x++)
			{
				nAdres = ADRES_NOR + x * ROZMIAR16_SEKTORA;
				*( (uint16_t *)nAdres + 0x555 ) = 0x0033;	//polecenie: Blank check


				nAdres = ADRES_NOR;
				*( (uint16_t *)nAdres + 0x555 ) = 0x70;	//polecenie: status read enter
				sStatus = *( (uint16_t *)nAdres);

				*( (uint16_t *)nAdres + 0x555 ) = 0x71;	//polecenie: status register clear


				sprintf(chNapis, "Sektor %ld: %x ", x, sStatus);
				print(chNapis, 10, 120 + x*20);
			}*/



	return Err;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje serię transferów z pamięci w celu określenia przepustowości
// Wyniki są wyświetlane na ekranie
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestPredkosciOdczytu(void)
{
	HAL_StatusTypeDef chErr;
	uint16_t sBufor[ROZMIAR16_BUFORA];
	uint32_t x, y, nAdres;
	uint32_t nCzas;
	extern uint8_t chRysujRaz;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Pomiar odczytu z Flash NOR");
		setColor(GRAY60);
		sprintf(chNapis, "Wdus ekran aby zakonczyc pomiar");
		print(chNapis, CENTER, 40);
		chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	}
	setColor(WHITE);

	//odczyt z NOR metodą odczytu bufora
	nAdres = 0;
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		chErr = HAL_NOR_ReadBuffer(&hnor3, nAdres, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "Blad odczytu ");
			print(chNapis, 10, 80);
			return;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "HAL_NOR_ReadBuffer() t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 80);
	}


	//odczyt z NOR metodą widzianego jako zmienna
	nAdres = 0;
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sFlashMem[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "NOR petla for()      t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 100);
	}


	//odczyt z Flash kontrolera
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sMPUFlash[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "Flash kontrolera     t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 120);
	}

	//Odczyt z RAM do RAM
	uint16_t sBufor2[ROZMIAR16_BUFORA];
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sBufor2[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "RAM kontrolera       t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 140);
	}


	//odczyt z NOR przez DMA
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sFlashMem, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: NOR->RAM        t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 180);
	}


	//odczyt z Flash przez DMA
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sMPUFlash, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "Blad odczytu przez DMA");
			print(chNapis, 10, 200);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: Flash->RAM      t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 200);
	}


	//odczyt z RAM przez DMA
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBufor2, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "Blad odczytu przez DMA");
			print(chNapis, 10, 220);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: RAM->RAM        t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 220);
	}


	//odczyt z NOR przez MDMA
	nCzas = HAL_GetTick();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sFlashMem, (uint32_t)sBufor, ROZMIAR16_BUFORA, 4096);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "Blad odczytu przez DMA");
		print(chNapis, 10, 260);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: NOR->RAM      t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 260);
	}


	//odczyt z Flash przez MDMA
	nCzas = HAL_GetTick();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sMPUFlash, (uint32_t)sBufor, ROZMIAR16_BUFORA, 4096);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "Blad odczytu przez DMA");
		print(chNapis, 10, 280);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: Flash->RAM     t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 280);
	}


	//odczyt z RAM przez MDMA
	nCzas = HAL_GetTick();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBufor2, (uint32_t)sBufor, ROZMIAR16_BUFORA, 4096);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "Blad odczytu przez DMA");
		print(chNapis, 10, 300);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: RAM->RAM       t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 300);
	}
}
