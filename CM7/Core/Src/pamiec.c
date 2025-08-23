//////////////////////////////////////////////////////////////////////////////
//
// Obsługa testów i pomiarów pamieci
// Pamięci są dostępne pod następującymi zakresami adresów:
// Flash NOR: 	0x6800 0000..0x69FF FFFF (32M)
// Static RAM: 	0x6000 0000..0x603F FFFF (4M)
// Dynamic RAM:	0xC000 0000..0xC3FF FFFF (64M)
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <RPi35B_480x320.h>
#include "pamiec.h"
#include <stdio.h>
#include "czas.h"
#include "flash_nor.h"
#include "rysuj.h"


const uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaFlashCPU"))) 		sFlashMem[ROZMIAR16_BUFORA];	//bufor do odczytu z wewnętrznego flash
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaFlashNOR"))) 	sFlashNOR[ROZMIAR16_BUFORA];	//bufor do odczytu z Flash NOR
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM2"))) 	sBuforD2[ROZMIAR16_BUFORA];
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM")))		sBuforDram[ROZMIAR16_BUFORA];
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDTCM")))		sBuforDTCM[ROZMIAR16_BUFORA];
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM")))	sBufor[ROZMIAR16_BUFORA];
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM")))	sBufor2[ROZMIAR16_BUFORA];
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM"))) 	sBuforSektoraFlash[ROZMIAR16_BUF_SEKT];	//Bufor sektora Flash NOR umieszczony w AXI-SRAM
extern uint16_t __attribute__((section(".SekcjaZewnSRAM")))	sBuforKamery[ROZM_BUF16_KAM];

uint16_t sWskBufSektora;	//wskazuje na poziom zapełnienia bufora
volatile uint8_t chKoniecTransferuMDMA;
volatile uint8_t chBladTransferuMDMA;
MDMA_LinkNodeTypeDef  __attribute__ ((aligned (32))) Xfer_Node1;
MDMA_LinkNodeTypeDef  __attribute__ ((aligned (32))) Xfer_Node2;
extern SRAM_HandleTypeDef hsram1;
extern NOR_HandleTypeDef hnor3;
extern SDRAM_HandleTypeDef hsdram1;
extern DMA_HandleTypeDef hdma_memtomem_dma1_stream1;
extern MDMA_HandleTypeDef hmdma_mdma_channel0_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel1_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel2_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel3_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel4_dma1_stream1_tc_0;
extern RNG_HandleTypeDef hrng;
extern char chNapis[];
HAL_StatusTypeDef hsErr;





////////////////////////////////////////////////////////////////////////////////
// Inicjuje mechanizm odświeżania pamięci SDRAM
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void SDRAM_Initialization_Sequence(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command)
{
	__IO uint32_t tmpmrd =0;
	//* Step 3:  Configure a clock configuration enable command
	Command->CommandMode     = FMC_SDRAM_CMD_CLK_ENABLE;
	Command->CommandTarget    = FMC_SDRAM_CMD_TARGET_BANK1;
	Command->AutoRefreshNumber   = 1;
	Command->ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

	// Step 4: Insert 100us delay
	HAL_Delay(1);

	// Step 5: Configure a PALL (precharge all) command
	Command->CommandMode     = FMC_SDRAM_CMD_PALL;
	Command->CommandTarget       = FMC_SDRAM_CMD_TARGET_BANK1;
	Command->AutoRefreshNumber   = 1;
	Command->ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

	// Step 6 : Configure a Auto-Refresh command
	Command->CommandMode     = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
	Command->CommandTarget    = FMC_SDRAM_CMD_TARGET_BANK1;
	Command->AutoRefreshNumber   = 4;
	Command->ModeRegisterDefinition = 0;
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

	// Step 7: Program the external memory mode register
	tmpmrd = (uint32_t)0;

			 /*SDRAM_MODEREG_BURST_LENGTH_2          |
	SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |
	SDRAM_MODEREG_CAS_LATENCY_3           |
	SDRAM_MODEREG_OPERATING_MODE_STANDARD |
	SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;*/

	Command->CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
	Command->CommandTarget    = FMC_SDRAM_CMD_TARGET_BANK1;
	Command->AutoRefreshNumber   = 1;
	Command->ModeRegisterDefinition = tmpmrd;
	HAL_SDRAM_SendCommand(hsdram, Command, 0x1000);

	// Step 8: Set the refresh rate counter
	// Refresh Period (8192 cycles) = 64ms
	// COUNT = (SDRAM refresh period ⁄ Number of rows) – 20 = (8192 / 12) - 20 = 662
	HAL_SDRAM_ProgramRefreshRate(hsdram, 662);

	// (15.62 us x Freq) - 20
	// Set the device refresh counter
	//HAL_SDRAM_ProgramRefreshRate(hsdram, 0x056A);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja testowa do wywołania z zewnątrz. Wykonuje pomiary kasowania programowania i odczytu
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t TestPredkosciZapisuNOR(void)
{
	HAL_StatusTypeDef chErr;
	uint16_t x;
	uint32_t y, nAdres;
	uint32_t nCzas;
	extern uint8_t chRysujRaz;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Pomiar zapisu do Flash NOR");
		setColor(GRAY60);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		RysujNapis(chNapis, CENTER, 300);
	}
	setColor(WHITE);

	chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	setColor(WHITE);

	//zmierz czas kasowania sektora
	nCzas = PobierzCzasT6();
	for (y=0; y<4; y++)
	{
		nAdres = ADRES_NOR + ((SEKTOR_TESTOW_PROG + y) * ROZMIAR16_SEKTORA);
		chErr = KasujSektorFlashNOR(nAdres);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd kasowania sektora", ł, ą);
			RysujNapis(chNapis, 10, 40);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Kasowanie sektora = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_SEKTORA * 4) / (nCzas * 1.048576f));
	RysujNapis(chNapis, 10, 40);

	//odczyt jako test skasowania
	chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	for (y=0; y<64; y++)
	{
		chErr = CzytajDaneFlashNOR(nAdres + y * ROZMIAR16_BUFORA, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd odczytu", ł, ą);
			RysujNapis(chNapis, 10, 80);
			return chErr;
		}
	}

	//zmierz czas programowania połowy sektora
	nCzas = PobierzCzasT6();
	nAdres = ADRES_NOR + SEKTOR_TESTOW_PROG * ROZMIAR16_SEKTORA;	//sektor
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = x + (y<<8);

		chErr = ZapiszDaneFlashNOR(nAdres, sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			sprintf(chNapis, "B%c%cd na stronie =%ld", ł, ą, y);
			RysujNapis(chNapis, 10, 60);
			return chErr;
		}
		nAdres += ROZMIAR8_BUFORA;	//adres musi wyrażać bajty a nie słowa bo w funkcji programowania jest mnożony x2
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Zapis 16 buforow  = %ld us => %.2f kB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.024f));
	RysujNapis(chNapis, 10, 60);

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
			RysujNapis(chNapis, 10, 80);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Odczyt 1k buforow = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));

	RysujNapis(chNapis, 10, 80);
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
	uint32_t x, y, nAdres;
	uint32_t nCzas;
	extern uint8_t chRysujRaz;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Pomiar odczytu z Flash NOR");
		setColor(GRAY60);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		RysujNapis(chNapis, CENTER, 300);
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
			RysujNapis(chNapis, 10, 40);
			return;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "HAL_NOR_ReadBuffer()  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 40);
	}


	//odczyt z NOR metodą widzianego jako zmienna
	nAdres = ADRES_NOR;
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sFlashNOR[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): NOR->AxiRAM    t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 60);
	}


	//odczyt z Flash kontrolera
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = *(uint16_t*)ADRES_NOR;
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): Flash->AxiRAM  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 80);
	}

	//Odczyt z RAM do RAM

	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sBufor2[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): AxiRAM->AxiRAM t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 100);
	}


	//odczyt z NOR przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sFlashNOR, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: NOR->AxiRAM      t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 140);
	}

	//odczyt z Flash przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, ADRES_NOR, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "Blad odczytu przez DMA");
			RysujNapis(chNapis, 10, 160);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: Flash->AxiRAM    t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 160);
	}


	//odczyt z RAM przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBufor2, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 180);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: AxiRAM->AxiRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 180);
	}

	//odczyt z RAM D2 przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<16; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBuforD2, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 200);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: SRAM1->AxiRAM    t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 200);
	}

	//odczyt z NOR przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel2_dma1_stream1_tc_0, (uint32_t)sFlashNOR, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		RysujNapis(chNapis, 10, 220);
		return;
	}

	while(hmdma_mdma_channel2_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel2_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		if (chErr == ERR_OK)
			sprintf(chNapis, "MDMA: NOR->AxiRAM     t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		else
			sprintf(chNapis, "MDMA: NOR->AxiRAM     B%c%cd", ł, ą);
		RysujNapis(chNapis, 10, 240);
	}

	//odczyt z Flash przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel3_dma1_stream1_tc_0, (uint32_t)sFlashMem, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "Blad odczytu przez DMA");
		RysujNapis(chNapis, 10, 260);
		return;
	}

	while(hmdma_mdma_channel3_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel3_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		if (chErr == ERR_OK)
			sprintf(chNapis, "MDMA: Flash->AxiRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		else
			sprintf(chNapis, "MDMA: Flash->AxiRAM   B%c%cd", ł, ą);
		RysujNapis(chNapis, 10, 260);
	}


	//odczyt z RAM przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel4_dma1_stream1_tc_0, (uint32_t)sBuforD2, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		RysujNapis(chNapis, 10, 280);
		return;
	}
	while(hmdma_mdma_channel4_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel4_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);

	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		if (chErr == ERR_OK)
			sprintf(chNapis, "MDMA: SRAM1->AxiRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		else
			sprintf(chNapis, "MDMA: SRAM1->AxiRAM   B%c%cd", ł, ą);
		RysujNapis(chNapis, 10, 280);
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
	uint32_t nRrandom32;
	uint32_t x, y;
	uint32_t nCzas;
	extern uint8_t chRysujRaz;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Pomiary przepustowosci RAM");
		setColor(GRAY60);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		RysujNapis(chNapis, CENTER, 300);
		chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	}
	setColor(WHITE);

	//Odczyt z zewnętrznego SRAM do RAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			 sBufor[x] = sBuforKamery[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		//sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4096 * 1000000) / (nCzas * 1024 * 1024));
		//sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4096) / (nCzas * 1024 * 1024));
		//sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ldms, transfer = %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 4096) / (nCzas * 1.024f * 1.024f));
		sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 40);
	}

	//Zapis do zewnętrznego SRAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBuforKamery[x] = sBufor[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): AxiSRAM->ExtSRAM  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 60);
	}

	//odczyt z zewnętrznego SRAM przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBuforKamery, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 80);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: ExtSRAM->AxiSRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 80);
	}


	//zapis do zewnętrznego SRAM przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBufor, (uint32_t)sBuforKamery, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 100);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: AxiSRAM->ExtSRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 100);
	}


	//odczyt z zewnętrznego SRAM przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBuforKamery, (uint32_t)sBufor, ROZMIAR16_BUFORA, 1000);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		RysujNapis(chNapis, 10, 120);
		return;
	}

	while(hmdma_mdma_channel0_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		if (chErr == ERR_OK)
			sprintf(chNapis, "MDMA: ExtSRAM->AxiSRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		else
			sprintf(chNapis, "MDMA: ExtSRAM->AxiSRAM   B%c%cd", ł, ą);
		RysujNapis(chNapis, 10, 120);
	}


	//zapis zewnętrznego SRAM przez MDMA
	nCzas = PobierzCzasT6();
	//chErr = HAL_MDMA_Start(&hmdma_mdma_channel1_dma1_stream1_tc_0, (uint32_t)sBufor, (uint32_t)sBuforKamery, ROZMIAR16_BUFORA, 1000);
	chErr = HAL_MDMA_Start_IT(&hmdma_mdma_channel1_dma1_stream1_tc_0, (uint32_t)sBufor, (uint32_t)sBuforKamery, ROZMIAR16_BUFORA, 1);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		RysujNapis(chNapis, 10, 140);
		return;
	}

	while(hmdma_mdma_channel1_dma1_stream1_tc_0.State != HAL_MDMA_STATE_READY)
		chErr = HAL_MDMA_PollForTransfer(&hmdma_mdma_channel1_dma1_stream1_tc_0, HAL_MDMA_FULL_TRANSFER, 200);
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		if (chErr == ERR_OK)
			sprintf(chNapis, "MDMA: AxiSRAM->ExtSRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		else
			sprintf(chNapis, "MDMA: ExtSRAM->AxiSRAM   B%c%cd", ł, ą);
		RysujNapis(chNapis, 10, 140);
	}

	for (y=0; y<ROZMIAR16_BUFORA; y++)
		sBufor[y] = y;

	//zapis DRAM
	nCzas = PobierzCzasT6();
	for (x=0; x<1000; x++)
		chErr = HAL_SDRAM_Write_16b(&hsdram1, (uint32_t *)ADRES_DRAM, sBufor, ROZMIAR16_BUFORA);

	nCzas = MinalCzas(nCzas);
	if (chErr == ERR_OK)
	{
		sprintf(chNapis, "HAL_SDRAM_Write_16b()    t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 160);
	}

	for (y=0; y<ROZMIAR16_BUFORA; y++)
			sBufor[y] = 0;


	//odczyt DRAM
	nCzas = PobierzCzasT6();
	for (x=0; x<1000; x++)
		chErr = HAL_SDRAM_Read_16b(&hsdram1, (uint32_t *)ADRES_DRAM, sBufor, ROZMIAR16_BUFORA);
	nCzas = MinalCzas(nCzas);
	if (chErr == ERR_OK)
	{
		sprintf(chNapis, "HAL_SDRAM_Read_16b()     t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 180);
	}


	//Zapis do DRAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBuforDram[x] = sBufor[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): AxiSRAM->DRAM     t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 200);
	}


	//Odczyt z DRAM w pętli
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sBuforDram[x];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "for(): DRAM->AxiSRAM     t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 220);
	}

	/*HAL_Delay(100);
	for (y=0; y<ROZMIAR16_BUFORA; y++)
				sBufor[y] = 0;

	//odczyt z DRAM przez DMA
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		//chErr = HAL_SDRAM_Read_DMA(&hsdram1, (uint32_t *)ADRES_DRAM, (uint32_t*)sBufor, 1);
		chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBuforDram, (uint32_t)sBufor, ROZMIAR16_BUFORA, 1000);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 200);
			return;
		}

		//while(hsdram1.hmdma->State != HAL_MDMA_STATE_READY)
			//HAL_MDMA_PollForTransfer(hsdram1.hmdma, HAL_DMA_FULL_TRANSFER, 100);
			//HAL_DMA_PollForTransfer(&hsdram1.hmdma, HAL_DMA_FULL_TRANSFER, 100);
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		//sprintf(chNapis, "HAL_SDRAM_Read_DMA()     t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1) / (nCzas * 1.048576f));
		sprintf(chNapis, "MDMA: SDRAM->AxiSRAM     t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 200);
	}*/


	/*/zapis z DRAM przez MDMA
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sBufor, (uint32_t)sBuforDram, ROZMIAR16_BUFORA, 1000);
		//chErr = HAL_SDRAM_Write_DMA(&hsdram1, (uint32_t *)ADRES_DRAM, (uint32_t*)sBufor, 1);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 220);
			return;
		}

	//	while(hsdram1.hmdma->State != HAL_MDMA_STATE_READY)
		//	HAL_MDMA_PollForTransfer(hsdram1.hmdma, HAL_DMA_FULL_TRANSFER, 100);
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		//sprintf(chNapis, "HAL_SDRAM_Write_DMA()    t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1) / (nCzas * 1.048576f));
		sprintf(chNapis, "MDMA: AxiSRAM->SDRAM     t = %ld us => %.2f MB/s", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 220);
	}*/


	//zapis do  DRAM przez DMA1
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBufor, (uint32_t)sBuforDram, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 240);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: AxiSRAM->DRAM      t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 240);
	}


	//odczyt z DRAM przez DMA1
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		chErr = HAL_DMA_Start(&hdma_memtomem_dma1_stream1, (uint32_t)sBuforDram, (uint32_t)sBufor, ROZMIAR16_BUFORA);
		if (chErr != ERR_OK)
		{
			setColor(RED);
			sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
			RysujNapis(chNapis, 10, 260);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: DRAM->AxiSRAM      t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 260);
	}


	//przygotuj zestaw liczb losowych aby nie generować go w czasie testu
	uint8_t chRandom[ROZMIAR16_BUFORA];
	for (x=0; x<ROZMIAR16_BUFORA; x++)
	{
		HAL_RNG_GenerateRandomNumber(&hrng, &nRrandom32);
		chRandom[x] = (uint8_t)(nRrandom32 & 0xFF);
	}

	//sekwencyjny odczyt z DRAM
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sBuforDram[chRandom[x]];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "random DRAM->AxiSRAM     t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 280);
	}


	/*/sekwencyjny odczyt z external SRAM
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sBuforKamery[chRandom[x]];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "random ExtSRAM->AxiSRAM  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		RysujNapis(chNapis, 10, 300);
	}*/
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje konfigurację transferów MDMA
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujMDMA(void)
{
	 MDMA_LinkNodeConfTypeDef mdmaLinkNodeConfig;

	//kanał do odczytu zewnetrznego SRAM
	mdmaLinkNodeConfig.Init.Request              = MDMA_REQUEST_SW;
	mdmaLinkNodeConfig.Init.TransferTriggerMode  = MDMA_FULL_TRANSFER;
	mdmaLinkNodeConfig.Init.Priority             = MDMA_PRIORITY_LOW;
	mdmaLinkNodeConfig.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE;
	mdmaLinkNodeConfig.Init.SourceInc            = MDMA_SRC_INC_HALFWORD;
	mdmaLinkNodeConfig.Init.DestinationInc       = MDMA_DEST_INC_HALFWORD;
	mdmaLinkNodeConfig.Init.SourceDataSize       = MDMA_SRC_DATASIZE_HALFWORD;
	mdmaLinkNodeConfig.Init.DestDataSize         = MDMA_DEST_DATASIZE_HALFWORD;
	mdmaLinkNodeConfig.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;
	mdmaLinkNodeConfig.Init.SourceBurst          = MDMA_SOURCE_BURST_SINGLE;
	mdmaLinkNodeConfig.Init.DestBurst            = MDMA_DEST_BURST_SINGLE;
	mdmaLinkNodeConfig.Init.BufferTransferLength = ROZMIAR16_BUFORA;
	mdmaLinkNodeConfig.Init.SourceBlockAddressOffset  = 0;
	mdmaLinkNodeConfig.Init.DestBlockAddressOffset    = 0;

	mdmaLinkNodeConfig.SrcAddress      = (uint32_t)sBuforKamery;
	mdmaLinkNodeConfig.DstAddress      = (uint32_t)sBufor;
	mdmaLinkNodeConfig.BlockDataLength = (ROZMIAR16_BUFORA);
	mdmaLinkNodeConfig.BlockCount      = 1;

	HAL_MDMA_LinkedList_CreateNode(&Xfer_Node1, &mdmaLinkNodeConfig);

	//kanał do zapisu zewnetrznego SRAM
	mdmaLinkNodeConfig.Init.Request              = MDMA_REQUEST_SW;
	mdmaLinkNodeConfig.Init.TransferTriggerMode  = MDMA_FULL_TRANSFER;
	mdmaLinkNodeConfig.Init.Priority             = MDMA_PRIORITY_LOW;
	mdmaLinkNodeConfig.Init.Endianness           = MDMA_LITTLE_ENDIANNESS_PRESERVE;
	mdmaLinkNodeConfig.Init.SourceInc            = MDMA_SRC_INC_HALFWORD;
	mdmaLinkNodeConfig.Init.DestinationInc       = MDMA_DEST_INC_HALFWORD;
	mdmaLinkNodeConfig.Init.SourceDataSize       = MDMA_SRC_DATASIZE_HALFWORD;
	mdmaLinkNodeConfig.Init.DestDataSize         = MDMA_DEST_DATASIZE_HALFWORD;
	mdmaLinkNodeConfig.Init.DataAlignment        = MDMA_DATAALIGN_PACKENABLE;
	mdmaLinkNodeConfig.Init.SourceBurst          = MDMA_SOURCE_BURST_SINGLE;
	mdmaLinkNodeConfig.Init.DestBurst            = MDMA_DEST_BURST_SINGLE;
	mdmaLinkNodeConfig.Init.BufferTransferLength = ROZMIAR16_BUFORA;
	mdmaLinkNodeConfig.Init.SourceBlockAddressOffset  = 0;
	mdmaLinkNodeConfig.Init.DestBlockAddressOffset    = 0;

	mdmaLinkNodeConfig.SrcAddress      = (uint32_t)sBufor;
	mdmaLinkNodeConfig.DstAddress      = (uint32_t)sBuforKamery;
	mdmaLinkNodeConfig.BlockDataLength = (ROZMIAR16_BUFORA);
	mdmaLinkNodeConfig.BlockCount      = 1;

	HAL_MDMA_LinkedList_CreateNode(&Xfer_Node2, &mdmaLinkNodeConfig);

	// Link the different nodes
	HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_dma1_stream1_tc_0, &Xfer_Node1, 0);
	HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel1_dma1_stream1_tc_0, &Xfer_Node2, 0);

	//##-5- Select Callbacks functions called after Transfer complete and Transfer error
	HAL_MDMA_RegisterCallback(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_XFER_CPLT_CB_ID, MDMA_TransferCompleteCallback);
	HAL_MDMA_RegisterCallback(&hmdma_mdma_channel0_dma1_stream1_tc_0, HAL_MDMA_XFER_ERROR_CB_ID, MDMA_TransferErrorCallback);

	HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(MDMA_IRQn);
}



////////////////////////////////////////////////////////////////////////////////
// Callback zakończenia transferu MDMA
// Parametry: hmdma wskaźnik na strukturę konfiguracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void MDMA_TransferCompleteCallback(MDMA_HandleTypeDef *hmdma)
{
	chKoniecTransferuMDMA = 1;
}



////////////////////////////////////////////////////////////////////////////////
// Callback wystąpienia błędu transferu MDMA
// Parametry: hmdma wskaźnik na strukturę konfiguracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void MDMA_TransferErrorCallback(MDMA_HandleTypeDef *hmdma)
{
	chBladTransferuMDMA = 1;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje serię zapisów i odczytów do pamięci na równoległej magistrali danych aby sprawdzić czy wszystkie linie danych i adresowe są drożne
// Parametry: nAdresBazowy - adres początku pamięci
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t SprawdzMagistrale(uint32_t nAdresBazowy)
{
	uint16_t sBufTest , sWartoscTestowa;
	uint16_t *sAdres;
	uint16_t sLicznikBledow = 0;

	sAdres = (uint16_t*)nAdresBazowy;

	//for(;;)
	{
		for (uint8_t m=0; m<11; m++)	//przemiataj magistralę adresową
		{
			for (uint8_t n=0; n<16; n++)	//przemiataj magistralę danych
			{
				sWartoscTestowa = (1 << n);
				*sAdres = sWartoscTestowa;
				sBufTest = *sAdres;
				if (sBufTest != sWartoscTestowa)
					sLicznikBledow++;
			}
			sAdres = (uint16_t*)(nAdresBazowy + (2<<m));
		}
	}
	if (sLicznikBledow)
		return ERR_ZLE_DANE;
	else
		return ERR_OK;
}


