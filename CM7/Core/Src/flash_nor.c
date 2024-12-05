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






uint16_t sMPUFlash[ROZMIAR16_BUFORA] __attribute__((section(".text")));
uint16_t sFlashMem[ROZMIAR16_BUFORA] __attribute__((section(".FlashNorSection")));
uint16_t sBuforD1[ROZMIAR16_BUFORA]  __attribute__((section(".Bufory_SRAM1")));
uint16_t sExtSramBuf[ROZMIAR16_EXT_SRAM] __attribute__((section(".ExtSramSection")));
extern SRAM_HandleTypeDef hsram1;
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
	FMC_NORSRAM_TimingTypeDef Timing = {0};

	extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu
	extern void Error_Handler(void);

	/* Pamięć jest taktowana zegarem 200 MHz (max 240MHz), daje to okres 5ns (4,1ns)
	 * Włączenie FIFO, właczenie wszystkich uprawnień na MPU Cortexa nic nie daje
	  Dla tych parametrów zapis bufor 512B ma przepustowość 258kB/s, odczyt danych 45,5MB/s   */


	//ponieważ timing generowany przez Cube nie ustawia właściwych parametrów więc nadpisuję je tutaj
	Timing.AddressSetupTime = 0;		//0ns
	Timing.AddressHoldTime = 6;			//45ns	45/8,3 => 6
	Timing.DataSetupTime = 18;			//18*5 = 90ns
	Timing.BusTurnAroundDuration = 0;	//tyko dla multipleksowanego NOR
	Timing.CLKDivision = 2;				//240/2=120MHz -> 8,3ns
	Timing.DataLatency = 0;
	Timing.AccessMode = FMC_ACCESS_MODE_A;
	if (HAL_NOR_Init(&hnor3, &Timing, NULL) != HAL_OK)
		Error_Handler( );

	chErr = SprawdzObecnoscFlashNOR();
	if (chErr == ERR_OK)
		nZainicjowano[0] |= INIT0_FLASH_NOR;
	HAL_NOR_ReturnToReadMode(&hnor3);		//ustaw pamięć w tryb odczytu

	//Analogicznie zrób tak dla SRAM
	Timing.AddressSetupTime = 0;		//Address Setup Time to Write End = 8ns
	Timing.AddressHoldTime = 1;			//Address Hold from Write End = 0
	Timing.DataSetupTime = 2;			//Read/Write Cycle Time = 10ns
	Timing.BusTurnAroundDuration = 0;	//tyko dla multipleksowanego NOR
	Timing.CLKDivision = 2;
	Timing.DataLatency = 0;				//Data Hold from Write End = 0
	Timing.AccessMode = FMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
	  Error_Handler( );

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
	chErr = HAL_NOR_Erase_Block(&hnor3, ADRES_NOR, nAdres);
	HAL_NOR_MspWait(&hnor3, 100);	//czekaj z timeoutem na niektywny sygnał RY/BY, czyli na niezajętą pamięć
	//HAL_NOR_ReturnToReadMode(&hnor3);
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
	uint8_t chErr;

	HAL_NOR_MspWait(&hnor3, 100);	//czekaj z timeoutem na niektywny sygnał RY/BY, czyli na niezajątą pamięć
	//nAdres &= 0x00FFFFFF;		//potrzebny jest adres względny
	chErr = HAL_NOR_ReadBuffer(&hnor3, nAdres, sDane, nIlosc);
	return chErr;
}




////////////////////////////////////////////////////////////////////////////////
// Zapisz dane do flash
// Parametry: nAdres do zapisu
// * sDane - wskaźnik na 16-bitoer dane do zapisu
//  nIlosc - ilość słów (nie bajtów) do zapisu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszDaneFlashNOR(uint32_t nAdres, uint16_t* sDane, uint32_t nIlosc)
{
	uint8_t chErr;

	nAdres &= 0x00FFFFFF;		//potrzebny jest adres względny
	chErr = HAL_NOR_ProgramBuffer(&hnor3, nAdres, sDane, nIlosc);
	HAL_NOR_MspWait(&hnor3, 100);	//czekaj z timeoutem na niektywny sygnał RY/BY, czyli na niezajętą pamięć
	return chErr;
}




////////////////////////////////////////////////////////////////////////////////
// Funkcja testowa do wywołania z zewnątrz. Wykonuje pomiary kasowania programowania i odczytu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t Test_Flash(void)
{
	HAL_StatusTypeDef chErr;
	HAL_NOR_StateTypeDef Stan;
	uint16_t x, sBufor[ROZMIAR16_BUFORA];
	uint32_t y, nAdres;
	uint32_t nCzas;

	chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	setColor(WHITE);

	Stan = HAL_NOR_GetState(&hnor3);
	if (Stan == HAL_NOR_STATE_PROTECTED)
	{
		chErr = HAL_NOR_WriteOperation_Enable(&hnor3);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "Blad właczenia zapisu");
			print(chNapis, 10, 20);
			return chErr;
		}
	}

	//zmierz czas kasowania sektora
	nCzas = HAL_GetTick();
	for (y=0; y<4; y++)
	{
		nAdres = ADRES_NOR + ((26 + y) * ROZMIAR16_SEKTORA);
		chErr = KasujSektorFlashNOR(nAdres);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "Blad kasowania sektora");
			print(chNapis, 10, 40);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Kasowanie sektora = %ldms, transfer = %.2f MB/s", nCzas, (float)(4000 * ROZMIAR8_SEKTORA) / (nCzas * 1024 * 1024));
	print(chNapis, 10, 40);


	//odczyt jako test skasowania
	chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	for (y=0; y<64; y++)
	{
		chErr = CzytajDaneFlashNOR(nAdres + y * ROZMIAR16_BUFORA, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "Blad odczytu");
			print(chNapis, 10, 80);
			return chErr;
		}
	}

	//włacz zapis
	Stan = HAL_NOR_GetState(&hnor3);
	if (Stan == HAL_NOR_STATE_PROTECTED)
	{
		chErr = HAL_NOR_WriteOperation_Enable(&hnor3);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "Blad właczenia zapisu");
			print(chNapis, 10, 20);
			return chErr;
		}
	}


	//zmierz czas programowania połowy sektora
	nCzas = HAL_GetTick();
	nAdres = ADRES_NOR + 26 * ROZMIAR16_SEKTORA;	//sektor
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = x + (y<<8);

		chErr = ZapiszDaneFlashNOR(nAdres, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "Blad na stronie =%ld", y);
			print(chNapis, 10, 60);
			return chErr;
		}
		nAdres += ROZMIAR8_BUFORA;	//adres musi wyrażać bajty a nie słowa bo w funkcji programowania jest mnożony x2
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Zapis 16 buforow  = %ldms, transfer = %.2f kB/s", nCzas, (float)(16 * 1000 * ROZMIAR8_BUFORA) / (nCzas * 1024));
	print(chNapis, 10, 60);


	//zmierz czas odczytu
	chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	nAdres = ADRES_NOR + 26 * ROZMIAR16_SEKTORA;
	nCzas = HAL_GetTick();
	for (y=0; y<1000; y++)
	{
		chErr = CzytajDaneFlashNOR(nAdres, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "Blad odczytu");
			print(chNapis, 10, 80);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
		sprintf(chNapis, "Odczyt 1k buforow = %ldms, transfer = %.2f MB/s", nCzas, (float)(1000 * 1000 * ROZMIAR8_BUFORA) / (nCzas * 1024 * 1024));
	else
		sprintf(chNapis, "za szybko! ");
	print(chNapis, 10, 80);


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

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje serię transferów z pamięci w celu określenia przepustowości
// Wyniki są wyświetlane na ekranie
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestPredkosciOdczytuNOR(void)
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
	nAdres = ADRES_NOR;
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
	nAdres = ADRES_NOR;
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sFlashMem[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): NOR->ASRAM    t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
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
		sprintf(chNapis, "for(): Flash->ASRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
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
		sprintf(chNapis, "for(): ASRAM->ASRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
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
		sprintf(chNapis, "DMA: NOR->ASRAM      t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
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
		sprintf(chNapis, "DMA: Flash->ASRAM    t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
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
		sprintf(chNapis, "DMA: ASRAM->ASRAM    t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 220);
	}



	//odczyt z RAM D2 przez DMA
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBuforD1, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "Blad odczytu przez DMA");
			print(chNapis, 10, 240);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: SRAM1->ASRAM    t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 240);
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
		sprintf(chNapis, "MDMA: NOR->ASRAM     t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
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
		sprintf(chNapis, "MDMA: Flash->ASRAM   t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 280);
	}


	//odczyt z RAM przez MDMA
	nCzas = HAL_GetTick();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBuforD1, (uint32_t)sBufor, ROZMIAR16_BUFORA, 4096);
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
		sprintf(chNapis, "MDMA: SRAM1->ASRAM   t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 300);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje serię transferów z pamięci RAM w celu określenia przepustowości
// Wyniki są wyświetlane na ekranie
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void TestPredkosciOdczytuRAM(void)
{
	HAL_StatusTypeDef chErr;
	uint16_t sBufor[ROZMIAR16_BUFORA];
	uint32_t x, y;
	uint32_t nCzas;
	extern uint8_t chRysujRaz;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Pomiary odczytu/zapisu SRAM");
		setColor(GRAY60);
		sprintf(chNapis, "Wdus ekran aby zakonczyc pomiar");
		print(chNapis, CENTER, 40);
		chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	}
	setColor(WHITE);

/*0x00,0x00,0x00,0x00,0x00,0x3C,0x06,0x06,0x3E,0x66,0x66,0x3E,0x04,0x06,0x00,0x00,	//100, "ą"
0x00,0x00,0x0C,0x18,0x00,0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,0x00,0x00,0x00,	//101, "ć"
0x00,0x00,0x00,0x00,0x00,0x3C,0x66,0x66,0x7E,0x60,0x60,0x3C,0x08,0x0C,0x00,0x00,	//102, "ę"
0x00,0x00,0x00,0x78,0x18,0x1A,0x1C,0x18,0x38,0x58,0x18,0x7E,0x00,0x00,0x00,0x00,	//103, "ł"
0x00,0x00,0x08,0x10,0x00,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00,	//104, "ń"
0x00,0x00,0x0C,0x18,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00,0x00,	//105, "ó"
0x00,0x00,0x0C,0x18,0x00,0x3E,0x60,0x60,0x3C,0x06,0x06,0x7C,0x00,0x00,0x00,0x00,	//106, "ś"
0x00,0x00,0x18,0x18,0x00,0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00,0x00,	//107, "ż"
0x00,0x00,0x0C,0x18,0x00,0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00,0x00,	//108, "ź"*/
	//Odczyt z zewnętrznego SRAM do RAM w pętli
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			 sBufor[x] = sExtSramBuf[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 80);
	}

	//Zapis do zewnętrznego SRAM w pętli
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sExtSramBuf[x] = sBufor[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): AxiSRAM->ExtSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 100);
		}


	//odczyt z zewnętrznego SRAM przez DMA
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sExtSramBuf, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "Blad odczytu przez DMA");
			print(chNapis, 10, 120);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: ExtSRAM->AxiSRAM    t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 120);
	}


	//zapis do zewnętrznego SRAM przez DMA
	nCzas = HAL_GetTick();
	for (y=0; y<4096; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBufor, (uint32_t)sExtSramBuf, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "Blad odczytu przez DMA");
			print(chNapis, 10, 140);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: AxiSRAM->ExtSRAM    t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 140);
	}


	//odczyt z zewnętrznego SRAM przez MDMA
	nCzas = HAL_GetTick();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sExtSramBuf, (uint32_t)sBufor, ROZMIAR16_BUFORA, 4096);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "Blad odczytu przez DMA");
		print(chNapis, 10, 160);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: ExtSRAM->AxiSRAM   t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 160);
	}


	//zapis zewnętrznego SRAM przez MDMA
	nCzas = HAL_GetTick();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBufor, (uint32_t)sExtSramBuf, ROZMIAR16_BUFORA, 4096);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "Blad odczytu przez DMA");
		print(chNapis, 10, 180);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: AxiSRAM->ExtSRAM   t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4000) / (nCzas * 1024));
		print(chNapis, 10, 180);
	}


	//zapis całego zewnętrznego SRAM w pętli
	nCzas = HAL_GetTick();
	for (y=0; y<ROZMIAR16_EXT_SRAM; y++)
		 sExtSramBuf[y] = y & 0xFFFF;

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): var->ExtSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR16_EXT_SRAM * 1000) / (nCzas * 1024 * 1024));
		print(chNapis, 10, 200);
	}


	//odczyt całego zewnętrznego SRAM w pętli
	nCzas = HAL_GetTick();
	for (y=0; y<ROZMIAR16_EXT_SRAM; y++)
		x = sExtSramBuf[y];

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): ExtSRAM->var  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR16_EXT_SRAM * 1000) / (nCzas * 1024 * 1024));
		print(chNapis, 10, 220);
	}


	sprintf(chNapis, "ąćęłńóśżź");
	for (x=0; x<20; x++)
		chNapis[x] = 'z'+x;
	chNapis[21] = 0;
	print(chNapis, 10, 260);
}
