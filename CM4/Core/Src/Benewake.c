//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi protokołu lidarów Benewake na przykładzie TF02-PRO
// Komunikacja po UART z prędkosścią 115200 bps - problemy z portem, na razie nie działa odbiór
// Komunikacja po I2C
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Benewake.h"
#include "Uarty.h"
#include "WymianaCM4.h"

uint8_t cBuforLidaru[ROZMIAR_BUF_LIDARU];
uint8_t cBuforAnalizyLidaru[ROZMIAR_BUF_ANA_LIDARU];
volatile uint8_t cWskNapBufAnaLidaru;
uint8_t cWskOprBufAnaLidaru;
extern UART_HandleTypeDef huart4;
extern volatile unia_wymianyCM4_t uDaneCM4;
uint8_t cStanProtokoluBenwake;
static uint8_t cLicznikPróbInicjalizacji = 0;
extern I2C_HandleTypeDef hi2c3;
extern volatile uint16_t sCzujnikZapisywanyNaI2CExt, sCzujnikOdczytywanyNaI2CExt;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje lidar sprawdzając jego obecność
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLidar(void)
{
	uint8_t cBłąd = BLAD_OK;
	uint8_t cBuforPolecenia[ROZMIAR_BUF_POLECEN];

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Alternate = GPIO_AF6_UART4;	//UART4 na porcie A zamiast RTS/CTS LPUART
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	cBłąd = InicjujUart4RxJakoLidar();

	//przełacz z UART na I2C, polecenie wysłane przez UART
	cBuforPolecenia[0] = LB_INTERFEJS; //przełacz na I2C
	cBuforPolecenia[1] = 0x01;			//0x01 = I2C
	cBłąd = WyślijPolecenieLidarowiUART(cBuforPolecenia, 2);

	cBuforPolecenia[0] = LB_ZAPISZ_UST;
	cBłąd = WyślijPolecenieLidarowiUART(cBuforPolecenia, 1);
	cBłąd = OdbierzPolecenieLidaraUART(cBuforAnalizyLidaru, 5);
	if (cBłąd)
		return BLAD_BRAK_CZUJNIKA;

	/*/obsługa po I2C - przełacz z UART na I2C
	cBuforLidaru[0] = NAGLOWEK_BENEWAKE;
	cBuforLidaru[1] = 0x05;			//rozmiar ramki wychodzącej
	cBuforLidaru[2] = LB_INTERFEJS;	//przełacz na I2C
	cBuforLidaru[3] = 0x01;			//0x01 = I2C
	cBuforLidaru[4] = 0x00;
	for (uint8_t n=0; n<4; n++)
		cBuforLidaru[4] += cBuforLidaru[n];			//suma kontrolna
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 5, TIMEOUT_POLECENIA);

	//zapisz polecenie
	cBuforLidaru[0] = NAGLOWEK_BENEWAKE;
	cBuforLidaru[1] = 0x04;			//rozmiar ramki wychodzącej
	cBuforLidaru[2] = LB_ZAPISZ_UST;
	cBuforLidaru[3] = 0x6F;			//suma kontrolna
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 4, TIMEOUT_POLECENIA);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 5, TIMEOUT_POLECENIA); */

	cBuforLidaru[0] = NAGLOWEK_BENEWAKE;
	cBuforLidaru[1] = 0x04;			//rozmiar ramki wychodzącej
	cBuforLidaru[2] = LB_WERSJA;
	cBuforLidaru[3] = 0x5F;			//suma kontrolna
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 4, TIMEOUT_POLECENIA);
	cBłąd = HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 7, TIMEOUT_POLECENIA);

	return cBłąd;
}


////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową lidaru
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaLidaruBenewake(void)
{
	uint8_t cBłąd;

	//po MAX_PROB_INICJALIZACJI ustawiany jest bit braku czujnika. Taki czujnik nie jest dłużej obsługiwany
	if (uDaneCM4.dane.nBrakCzujnika & INIT_LIDAR_BENEWAKE)
		return BLAD_OK;		//czujnik jest opcjonalny, więc nie raportuj jego braku jako błąd

	if ((uDaneCM4.dane.nZainicjowano & INIT_LIDAR_BENEWAKE) != INIT_LIDAR_BENEWAKE)	//jeżeli czujnik nie jest zainicjowany
	{
		cLicznikPróbInicjalizacji++;
		cBłąd = InicjujLidar();
		if (cBłąd)
		{
			/*if (cLicznikPróbInicjalizacji > MAX_PROB_INICJALIZACJI)
				uDaneCM4.dane.nBrakCzujnika |= INIT_LIDAR_BENEWAKE;
			return cBłąd;*/
		}
		uDaneCM4.dane.nZainicjowano |= INIT_LIDAR_BENEWAKE;
//		WłączOdbiórUART4();	//ustawia DMA odbiornika gotowe na przyjęcie pierwszej ramki
	}
	else	//czujnik jest zainicjowany
	{
		//Pierwsza część odczytu danych: wysłanie polecenia odczytu. Finalny odczyt jest uruchamiany w callbacku HAL_I2C_MasterRxCpltCallback w PetlaGlowna.c
		cBuforLidaru[0] = NAGLOWEK_BENEWAKE;
		cBuforLidaru[1] = 0x05;	//rozamiar ramki wychodzącej
		cBuforLidaru[2] = 0x00;
		cBuforLidaru[3] = 0x01;
		cBuforLidaru[4] = 0x60;	//suma kontrolna
		cBłąd = HAL_I2C_Master_Seq_Transmit_IT(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 5, I2C_FIRST_FRAME);	//wyślij polecenie odczytu pomiarów nie kończąc transferu STOP-em
		sCzujnikZapisywanyNaI2CExt = LIDAR_BENEWAKE;



		/*if (cWskNapBufAnaLidaru != cWskOprBufAnaLidaru)
		{
			cBłąd = AnalizujProtokolBenewake(cBuforAnalizyLidaru[cWskOprBufAnaLidaru]);
			cWskOprBufAnaLidaru++;
			if (cWskOprBufAnaLidaru == ROZMIAR_BUF_ANA_LIDARU)
				cWskOprBufAnaLidaru = 0;
		}*/
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje USART4 do obsługi lidara podczas konfiguracji w trybie blokujacym
// Parametry:
// *InitGPIO - wskaźnik na strukturę inicjalizacji portu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujUart4RxJakoLidar(void)
{
	uint8_t cBłąd;

	__HAL_RCC_UART4_CLK_ENABLE();
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
	cBłąd = HAL_UART_Init(&huart4);
	cBłąd |= HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2);
	cBłąd |= HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2);
	cBłąd |= HAL_UARTEx_DisableFifoMode(&huart4);

	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła polecenie budując ramkę
// Parametry:
//  *cBufor - wskaźnik na bufor z danymi polecenia bez stałych elementów ramki
//  cRozmiar - rozmiar danych w buforze
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t WyślijPolecenieLidarowiUART(uint8_t *cBufor, uint16_t sRozmiar)
{
	uint8_t cBłąd = BLAD_OK;
	uint8_t cSuma = 0;
	if (sRozmiar > ROZMIAR_BUF_POLECEN)
		return BLAD_ZLA_ILOSC_DANYCH;

	cBuforLidaru[0] = NAGLOWEK_BENEWAKE;	//nagłówek
	cBuforLidaru[1] = (uint8_t)sRozmiar + 3;
	for (uint16_t n=0; n<sRozmiar; n++)
		cBuforLidaru[2 + n] = *(cBufor + n);
	for (uint16_t n=0; n<sRozmiar+2; n++)
		cSuma += cBuforLidaru[n];
	cBuforLidaru[sRozmiar + 2] = cSuma;
	cBłąd = HAL_UART_Transmit(&huart4, cBuforLidaru, sRozmiar + 3, TIMEOUT_POLECENIA);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Odbiera polecenie w trybie blokującym wyłącznie do obsługi inicjalizacji
// Parametry:
//  *cBufor - wskaźnik na bufor z danymi polecenia bez stałych elementów ramki
//  cRozmiar - rozmiar danych w buforze
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t OdbierzPolecenieLidaraUART(uint8_t *cBufor, uint16_t sRozmiar)
{
	uint8_t cBłąd;
	//uint8_t cSuma = 0;

	cBłąd = HAL_UART_Receive(&huart4, cBufor, sRozmiar, TIMEOUT_POLECENIA);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja analizuje odebrane dane, składa w ramki i dekoduje informacje
// Parametry:
//  cZnak - przychodzący znak do analizy
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujProtokolBenewake(uint8_t cZnak)
{
	uint8_t cBłąd = BLAD_OK;

	switch (cStanProtokoluBenwake)
	{
	case PB_NAGLOWEK:
		if (cZnak == NAGLOWEK_BENEWAKE)
			cStanProtokoluBenwake++;
		break;

	case PB_ROZMIAR:
	case PB_POLECENIE:
	case PB_DANE:
	case PB_SUMA:
		break;

	}
	return cBłąd;
}


////////////////////////////////////////////////////////////////////////////////
// Polecenie będące drugą częścią sekwencji podzielonej operacji odczytu danych.
// Będzie uruchomione w callbacku zakończenia operacji wysłania polecenia odczytu danych
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajDaneLidaru()
{
	sCzujnikOdczytywanyNaI2CExt = LIDAR_BENEWAKE;		//w callbacku interpretuj odczytane dane jako pomiar lidaru
	return HAL_I2C_Master_Seq_Receive_DMA(&hi2c3, ADRES_I2C_BENEWAKE + I2C_READ, cBuforLidaru, 6, I2C_LAST_FRAME);		//odczytaj dane i zakończ STOP
}
