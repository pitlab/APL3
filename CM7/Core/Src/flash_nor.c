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





//uint16_t sMPUFlash[ROZMIAR16_BUFORA] __attribute__((section(".text")));
uint16_t sFlashMem[ROZMIAR16_BUFORA] __attribute__((section(".FlashNorSection")));
uint16_t sBuforD1[ROZMIAR16_BUFORA]  __attribute__((section(".Bufory_SRAM1")));
uint16_t sExtSramBuf[ROZMIAR16_BUFORA] __attribute__((section(".ExtSramSection")));
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
			sprintf(chNapis, "B%c%cd w%c%cczenia zapisu", ł, ą, ł, ą);
			print(chNapis, 10, 20);
			return chErr;
		}
	}

	//zmierz czas kasowania sektora
	nCzas = PobierzCzasT6();
	for (y=0; y<4; y++)
	{
		nAdres = ADRES_NOR + ((26 + y) * ROZMIAR16_SEKTORA);
		chErr = KasujSektorFlashNOR(nAdres);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd kasowania sektora", ł, ą);
			print(chNapis, 10, 40);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Kasowanie sektora = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_SEKTORA * 4) / (nCzas * 1.048576f));
	print(chNapis, 10, 40);


	//odczyt jako test skasowania
	chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	for (y=0; y<64; y++)
	{
		chErr = CzytajDaneFlashNOR(nAdres + y * ROZMIAR16_BUFORA, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd odczytu", ł, ą);
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
			sprintf(chNapis, "B%c%cd w%c%cczenia zapisu", ł, ą, ł, ą);
			print(chNapis, 10, 20);
			return chErr;
		}
	}


	//zmierz czas programowania połowy sektora
	nCzas = PobierzCzasT6();
	nAdres = ADRES_NOR + 26 * ROZMIAR16_SEKTORA;	//sektor
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = x + (y<<8);

		chErr = ZapiszDaneFlashNOR(nAdres, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd na stronie =%ld", ł, ą, y);
			print(chNapis, 10, 60);
			return chErr;
		}
		nAdres += ROZMIAR8_BUFORA;	//adres musi wyrażać bajty a nie słowa bo w funkcji programowania jest mnożony x2
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Zapis 16 buforow  = %ld us => %.2f kB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.024f));
	print(chNapis, 10, 60);


	//zmierz czas odczytu
	chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	nAdres = ADRES_NOR + 26 * ROZMIAR16_SEKTORA;
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = CzytajDaneFlashNOR(nAdres, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd odczytu", ł, ą);
			print(chNapis, 10, 80);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Odczyt 1k buforow = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));

	print(chNapis, 10, 80);
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
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 40);
		chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	}
	setColor(WHITE);

	//odczyt z NOR metodą odczytu bufora
	nAdres = ADRES_NOR;
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		chErr = HAL_NOR_ReadBuffer(&hnor3, nAdres, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu", ł, ą);
			print(chNapis, 10, 80);
			return;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "HAL_NOR_ReadBuffer() t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 80);
	}


	//odczyt z NOR metodą widzianego jako zmienna
	nAdres = ADRES_NOR;
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sFlashMem[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): NOR->ASRAM    t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 100);
	}


	//odczyt z Flash kontrolera
/*	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sMPUFlash[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): Flash->ASRAM  t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 120);
	}*/

	//Odczyt z RAM do RAM
	uint16_t sBufor2[ROZMIAR16_BUFORA];
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sBufor2[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): ASRAM->ASRAM  t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 140);
	}


	//odczyt z NOR przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sFlashMem, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: NOR->ASRAM      t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 180);
	}


	//odczyt z Flash przez DMA
/*	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
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
		sprintf(chNapis, "DMA: Flash->ASRAM    t = %d us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 200);
	}*/


	//odczyt z RAM przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBufor2, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			print(chNapis, 10, 220);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: ASRAM->ASRAM    t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 220);
	}

	//odczyt z RAM D2 przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBuforD1, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			print(chNapis, 10, 240);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: SRAM1->ASRAM    t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 240);
	}

	//odczyt z NOR przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sFlashMem, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 260);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: NOR->ASRAM     t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 260);
	}


	//odczyt z Flash przez MDMA
/*	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sMPUFlash, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
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
		sprintf(chNapis, "MDMA: Flash->ASRAM   t = %d us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 280);
	} */


	//odczyt z RAM przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBuforD1, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 300);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: SRAM1->ASRAM   t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
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
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 40);
		chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	}
	setColor(WHITE);

	//Odczyt z zewnętrznego SRAM do RAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			 sBufor[x] = sExtSramBuf[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		//sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4096 * 1000000) / (nCzas * 1024 * 1024));
		//sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4096) / (nCzas * 1024 * 1024));
		//sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4096) / (nCzas * 1.024f * 1.024f));
		sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 80);
	}

	//Zapis do zewnętrznego SRAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sExtSramBuf[x] = sBufor[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): AxiSRAM->ExtSRAM  t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 100);
	}

	//odczyt z zewnętrznego SRAM przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sExtSramBuf, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			print(chNapis, 10, 120);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: ExtSRAM->AxiSRAM    t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 120);
	}


	//zapis do zewnętrznego SRAM przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBufor, (uint32_t)sExtSramBuf, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			print(chNapis, 10, 140);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: AxiSRAM->ExtSRAM    t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 140);
	}


	//odczyt z zewnętrznego SRAM przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sExtSramBuf, (uint32_t)sBufor, ROZMIAR16_BUFORA, 1000);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 160);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: ExtSRAM->AxiSRAM   t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 160);
	}


	//zapis zewnętrznego SRAM przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBufor, (uint32_t)sExtSramBuf, ROZMIAR16_BUFORA, 1000);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 180);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "MDMA: AxiSRAM->ExtSRAM   t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 180);
	}


	//zapis całego zewnętrznego SRAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<ROZMIAR16_EXT_SRAM; y++)
		 sExtSramBuf[y] = y & 0xFFFF;

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): var->ExtSRAM  t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_EXT_SRAM) / (nCzas * 1.048576f));
		print(chNapis, 10, 200);
	}


	//odczyt całego zewnętrznego SRAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<ROZMIAR16_EXT_SRAM; y++)
		x = sExtSramBuf[y];

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): ExtSRAM->var  t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_EXT_SRAM) / (nCzas * 1.048576f));
		print(chNapis, 10, 220);
	}

}
