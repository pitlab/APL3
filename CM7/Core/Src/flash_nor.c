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
#include "czas.h"


/* Dodać polecnie pooling status bit
 * Sprawdzić jak działa przełaczanie między zapisem (HAL_NOR_WriteOperation_Enable) i odczytem (HAL_NOR_ReturnToReadMode)
 * */


ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaFlashNOR"))) 	sFlashNOR[ROZMIAR16_BUFORA]);	//bufor do odczytu z Flash NOR
ALIGN_32BYTES(const uint16_t __attribute__((section(".SekcjaFlashCPU"))) 		sFlashMem[ROZMIAR16_BUFORA]);	//bufor do odczytu z wewnętrznego flash
ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaSRAM2"))) 	sBuforD2[ROZMIAR16_BUFORA]);
//ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaZewnSRAM")))	sExtSramBuf[ROZMIAR16_BUFORA]);
ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaDRAM")))		sBuforDram[ROZMIAR16_BUFORA]);
ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaDTCM")))		sBuforDTCM[ROZMIAR16_BUFORA]);
ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaAxiSRAM")))	sBufor[ROZMIAR16_BUFORA]);
ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaAxiSRAM")))	sBufor2[ROZMIAR16_BUFORA]);
ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaAxiSRAM"))) 	sBuforSektoraFlash[ROZMIAR16_BUF_SEKT]);	//Bufor sektora Flash NOR umieszczony w AXI-SRAM

uint16_t sWskBufSektora;	//wskazuje na poziom zapełnienia bufora
volatile uint8_t chKoniecTransferuMDMA;
volatile uint8_t chBladTransferuMDMA;
ALIGN_32BYTES(MDMA_LinkNodeTypeDef Xfer_Node1);
ALIGN_32BYTES(MDMA_LinkNodeTypeDef Xfer_Node2);
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
// Inicjuj Flash NOR
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujFlashNOR(void)
{
	uint8_t chErr;
	FMC_NORSRAM_TimingTypeDef Timing = {0};
	FMC_SDRAM_TimingTypeDef SdramTiming = {0};

	extern uint32_t nZainicjowano[2];		//flagi inicjalizacji sprzętu
	extern void Error_Handler(void);

	/* Pamięć jest taktowana zegarem 200 MHz (max 240MHz), daje to okres 5ns (4,1ns)
	 * Włączenie FIFO, właczenie wszystkich uprawnień na MPU Cortexa nic nie daje
	  Dla tych parametrów zapis bufor 512B ma przepustowość 258kB/s, odczyt danych 45,5MB/s   */
	Timing.AddressSetupTime = 0;		//0ns
	Timing.AddressHoldTime = 6;			//45ns	45/8,3 => 6
	Timing.DataSetupTime = 18;			//18*5 = 90ns
	Timing.BusTurnAroundDuration = 3;	//tyko dla multipleksowanego NOR
	Timing.CLKDivision = 2;				//240/2=120MHz -> 8,3ns
	Timing.DataLatency = 2;
	Timing.AccessMode = FMC_ACCESS_MODE_A;
	if (HAL_NOR_Init(&hnor3, &Timing, NULL) != HAL_OK)
		Error_Handler( );

	chErr = SprawdzObecnoscFlashNOR();
	if (chErr == ERR_OK)
		nZainicjowano[0] |= INIT0_FLASH_NOR;
	HAL_NOR_ReturnToReadMode(&hnor3);		//ustaw pamięć w tryb odczytu

	//ręcznie włącz Bit 13 WAITEN: Wait enable bit oraz Bit 11 WAITCFG: Wait timing configuration,ponieważ sterownik HAL go nie włącza
	//indeksy struktury BTCR są wspólne dla rejestrów BCRx i RTRx, więc BCR3 ma indeks 4
	hnor3.Instance->BTCR[4] |= FMC_WAIT_SIGNAL_ENABLE | FMC_WAIT_TIMING_DURING_WS;


	//Analogicznie zrób tak dla SRAM
	Timing.AddressSetupTime = 0;		//Address Setup Time to Write End = 8ns
	Timing.AddressHoldTime = 1;			//Address Hold from Write End = 0
	Timing.DataSetupTime = 2;			//Read/Write Cycle Time = 10ns
	Timing.BusTurnAroundDuration = 0;	//tyko dla multipleksowanego NOR
	Timing.CLKDivision = 2;
	Timing.DataLatency = 0;				//Data Hold from Write End = 0
	Timing.AccessMode = FMC_ACCESS_MODE_A;
	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
	  Error_Handler( );


	//zegar jest 200 -> 5ns. Pamięć dla CL*=1 -> 20ns, [[[CL*=2 -> 10ns]]], CL*=3 -> 7,5ns
	//ustawiam 2 cykle zegara -> 10ns i CL =2
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;	//clock period: fmc_ker_ck/2 or fmc_ker_ck/3
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;


	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 2;
	SdramTiming.ExitSelfRefreshDelay = 8;	//Exit Self-Refresh to any Command = 75ns
	SdramTiming.SelfRefreshTime = 4;
	SdramTiming.RowCycleDelay = 6;
	SdramTiming.WriteRecoveryTime = 2;		//Write recovery time = 15ns
	SdramTiming.RPDelay = 2;
	SdramTiming.RCDDelay = 2;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
	{
	Error_Handler( );
	}

	FMC_SDRAM_CommandTypeDef Command;


	SDRAM_Initialization_Sequence(&hsdram1, &Command);



	return chErr;
}



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
// Kasuj sektor flash o rozmiarze 128kB (0xFFFF słów)
// Parametry: nAdres sektora do skasowania
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KasujSektorFlashNOR(uint32_t nAdres)
{
	uint8_t chErr;

	chErr = HAL_NOR_Erase_Block(&hnor3, ADRES_NOR, nAdres & 0x00FFFFFF);	//potrzebny jest adres względny
	if (chErr == ERR_OK)
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
	if (chErr == ERR_OK)
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
	return ERR_OK;
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
		print(chNapis, CENTER, 300);
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
			print(chNapis, 10, 40);
			return chErr;
		}
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Kasowanie sektora = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_SEKTORA * 4) / (nCzas * 1.048576f));
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
			print(chNapis, 10, 60);
			return chErr;
		}
		nAdres += ROZMIAR8_BUFORA;	//adres musi wyrażać bajty a nie słowa bo w funkcji programowania jest mnożony x2
	}
	nCzas = MinalCzas(nCzas);
	sprintf(chNapis, "Zapis 16 buforow  = %ld us => %.2f kB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.024f));
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
	sprintf(chNapis, "Odczyt 1k buforow = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));

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
	uint32_t x, y, nAdres;
	uint32_t nCzas;
	extern uint8_t chRysujRaz;

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		BelkaTytulu("Pomiar odczytu z Flash NOR");
		setColor(GRAY60);
		sprintf(chNapis, "Wdu%c ekran i trzymaj aby zako%cczy%c", ś, ń, ć);
		print(chNapis, CENTER, 300);
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
			print(chNapis, 10, 40);
			return;
		}
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "HAL_NOR_ReadBuffer()  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 40);
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
		print(chNapis, 10, 60);
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
		print(chNapis, 10, 80);
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
		print(chNapis, 10, 100);
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
		print(chNapis, 10, 140);
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
			print(chNapis, 10, 160);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: Flash->AxiRAM    t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 160);
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
			print(chNapis, 10, 180);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: AxiRAM->AxiRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 180);
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
			print(chNapis, 10, 200);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA: SRAM1->AxiRAM    t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 16) / (nCzas * 1.048576f));
		print(chNapis, 10, 200);
	}

	//odczyt z NOR przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel2_dma1_stream1_tc_0, (uint32_t)sFlashNOR, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 220);
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
		print(chNapis, 10, 240);
	}

	//odczyt z Flash przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel3_dma1_stream1_tc_0, (uint32_t)sFlashMem, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "Blad odczytu przez DMA");
		print(chNapis, 10, 260);
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
		print(chNapis, 10, 260);
	}


	//odczyt z RAM przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel4_dma1_stream1_tc_0, (uint32_t)sBuforD2, (uint32_t)sBufor, ROZMIAR16_BUFORA, 16);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 280);
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
		print(chNapis, 10, 280);
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
		print(chNapis, CENTER, 300);
		chErr = HAL_NOR_ReturnToReadMode(&hnor3);
	}
	setColor(WHITE);

	/*/Odczyt z zewnętrznego SRAM do RAM w pętli
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
		sprintf(chNapis, "for(): ExtSRAM->AxiSRAM  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 40);
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
		sprintf(chNapis, "for(): AxiSRAM->ExtSRAM  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 60);
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
			print(chNapis, 10, 80);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: ExtSRAM->AxiSRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 80);
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
			print(chNapis, 10, 100);
			return;
		}

		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: AxiSRAM->ExtSRAM   t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 100);
	}


	//odczyt z zewnętrznego SRAM przez MDMA
	nCzas = PobierzCzasT6();
	chErr = HAL_MDMA_Start(&hmdma_mdma_channel0_dma1_stream1_tc_0, (uint32_t)sExtSramBuf, (uint32_t)sBufor, ROZMIAR16_BUFORA, 1000);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 120);
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
		print(chNapis, 10, 120);
	}


	//zapis zewnętrznego SRAM przez MDMA
	nCzas = PobierzCzasT6();
	//chErr = HAL_MDMA_Start(&hmdma_mdma_channel1_dma1_stream1_tc_0, (uint32_t)sBufor, (uint32_t)sExtSramBuf, ROZMIAR16_BUFORA, 1000);
	chErr = HAL_MDMA_Start_IT(&hmdma_mdma_channel1_dma1_stream1_tc_0, (uint32_t)sBufor, (uint32_t)sExtSramBuf, ROZMIAR16_BUFORA, 1);
	if (chErr != ERR_OK)
	{
		setColor(RED);
		sprintf(chNapis, "B%c%cd odczytu przez DMA", ł, ą);
		print(chNapis, 10, 140);
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
		print(chNapis, 10, 140);
	}

	for (y=0; y<ROZMIAR16_BUFORA; y++)
		sBufor[y] = y;
		*/

	//zapis DRAM
	nCzas = PobierzCzasT6();
	for (x=0; x<1000; x++)
		chErr = HAL_SDRAM_Write_16b(&hsdram1, (uint32_t *)ADRES_DRAM, sBufor, ROZMIAR16_BUFORA);

	nCzas = MinalCzas(nCzas);
	if (chErr == ERR_OK)
	{
		sprintf(chNapis, "HAL_SDRAM_Write_16b()    t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 160);
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
		print(chNapis, 10, 180);
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
		print(chNapis, 10, 200);
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
		print(chNapis, 10, 220);
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
			print(chNapis, 10, 200);
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
		print(chNapis, 10, 200);
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
			print(chNapis, 10, 220);
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
		print(chNapis, 10, 220);
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
			print(chNapis, 10, 240);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: AxiSRAM->DRAM      t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 240);
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
			print(chNapis, 10, 260);
			return;
		}
		while(hdma_memtomem_dma1_stream1.State != HAL_DMA_STATE_READY)
			HAL_DMA_PollForTransfer(&hdma_memtomem_dma1_stream1, HAL_DMA_FULL_TRANSFER, 100);
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "DMA1: DRAM->AxiSRAM      t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 260);
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
		print(chNapis, 10, 280);
	}


	/*/sekwencyjny odczyt z external SRAM
	nCzas = PobierzCzasT6();
	for (y=0; y<1000; y++)
	{
		for (x=0; x<ROZMIAR16_BUFORA; x++)
			sBufor[x] = sExtSramBuf[chRandom[x]];
	}
	nCzas = MinalCzas(nCzas);
	if (nCzas)
	{
		sprintf(chNapis, "random ExtSRAM->AxiSRAM  t = %ld us => %.2f MB/s ", nCzas, (float)(ROZMIAR8_BUFORA * 1000) / (nCzas * 1.048576f));
		print(chNapis, 10, 300);
	}*/
}



////////////////////////////////////////////////////////////////////////////////
// Callback zakończenia transferu MDMA
// Parametry: hmdma wskaźnik na strukturę konfiguracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
static void MDMA_TransferCompleteCallback(MDMA_HandleTypeDef *hmdma)
{
	chKoniecTransferuMDMA = 1;
}



////////////////////////////////////////////////////////////////////////////////
// Callback wystąpienia błędu transferu MDMA
// Parametry: hmdma wskaźnik na strukturę konfiguracji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
static void MDMA_TransferErrorCallback(MDMA_HandleTypeDef *hmdma)
{
	chBladTransferuMDMA = 1;
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

	//mdmaLinkNodeConfig.SrcAddress      = (uint32_t)sExtSramBuf;
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
	//mdmaLinkNodeConfig.DstAddress      = (uint32_t)sExtSramBuf;
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
