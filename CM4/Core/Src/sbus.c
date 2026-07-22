//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi wejść protokołu S-Bus
//
// (c) PitLab 2026
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <SBus.h>
#include "Czas.h"
#include "Uarty.h"


uint8_t cBuforNadawczySBus[ROZMIAR_RAMKI_SBUS] =  {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00};
uint8_t cBuforAnalizySBus1[ROZMIAR_BUF_ANA_SBUS];
uint8_t cBuforAnalizySBus2[ROZMIAR_BUF_ANA_SBUS];
volatile uint8_t cWskNapBufAnaSBus1, cWskOprBufAnaSBus1; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus1
volatile uint8_t cWskNapBufAnaSBus2, cWskOprBufAnaSBus2; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus2
uint8_t cWskNapRamkiSBus1, cWskNapRamkiSBus2;				//wskaźniki napełniania ramek SBus
uint8_t cRamkaSBus1[ROZMIAR_RAMKI_SBUS];
uint8_t cRamkaSBus2[ROZMIAR_RAMKI_SBUS];
uint8_t cKorektaPoczatkuRamki;
uint32_t nCzasWysylkiSbus;
extern stRC_t stRC1, stRC2;	//struktura danych odbiorników RC1 i RC2
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart2_rx;

extern UART_HandleTypeDef huart4;
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern uint8_t cKonfigWyRC[LICZBA_WYJSC_RC];



//
void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}



void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)		//wysłano ramkę SBUS1
	{

	}
}


//fizyczny odbiór HAL_UART_RxCpltCallback znajduje się w pliku GNSS.c



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje USART2 jako wejcie sygnału S-Bus
// Parametry:
// *InitGPIO - wskaźnik na strukturę inicjalizacji portu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujUart2RxJakoSbus(GPIO_InitTypeDef *InitGPIO)
{
	uint8_t cBłąd;

	InitGPIO->Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, InitGPIO);
	__HAL_RCC_USART2_CLK_ENABLE();

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 100000;
	huart2.Init.WordLength = UART_WORDLENGTH_9B;
	huart2.Init.StopBits = UART_STOPBITS_2;
	huart2.Init.Parity = UART_PARITY_EVEN;
	huart2.Init.Mode = UART_MODE_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
	huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	//huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
	//huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;

	cBłąd  = HAL_UART_Init(&huart2);
	cBłąd |= HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_2);
	//cBłąd |= HAL_UARTEx_EnableFifoMode(&huart2);
	cBłąd |= HAL_UARTEx_DisableFifoMode(&huart2);

	//ponieważ UART jest współdzielony z timerem a część TC jest wyłączona, więc nie da się go skonfigurować w cube
	hdma_uart2_rx.Instance = DMA1_Stream0;				//UWAGA! zasób przydziony ręcznie, może wystąpić konflikt
	hdma_uart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
	hdma_uart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_uart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_uart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_uart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_uart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_uart2_rx.Init.Mode = DMA_CIRCULAR;
	hdma_uart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_uart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	cBłąd |= HAL_DMA_Init(&hdma_uart2_rx);

    __HAL_LINKDMA(&huart2, hdmarx, hdma_uart2_rx);

	//ponieważ nie ma jak zarezerwować kanału DMA w HAL bo dostępna jest tylko część odbiorcza, więc USART2 będzie pracował na przerwaniach
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	WłączOdbiórUART2();	//ustawia odbiornik gotowy na przyjęcie pierwszej ramki
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje USART4 jako wejcie sygnału S-Bus
// Parametry:
// *InitGPIO - wskaźnik na strukturę inicjalizacji portu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujUart4RxJakoSbus(GPIO_InitTypeDef *InitGPIO)
{
	uint8_t cBłąd;

	InitGPIO->Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOB, InitGPIO);
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_UART4_CLK_ENABLE();

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 100000;
	huart4.Init.WordLength = UART_WORDLENGTH_9B;	//bit parzystości liczy sie jako dane
	huart4.Init.StopBits = UART_STOPBITS_2;
	huart4.Init.Parity = UART_PARITY_EVEN;
	if (huart4.Init.Mode == UART_MODE_TX)//jeżeli właczony jest nadajnik to włącz jeszcze odbiornik a jeżeli nie to tylko odbiornik
		huart4.Init.Mode = UART_MODE_TX_RX;
	else
		huart4.Init.Mode = UART_MODE_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
	huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
	cBłąd = HAL_UART_Init(&huart4);
	cBłąd |= HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2);
	cBłąd |= HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2);
	//cBłąd |= HAL_UARTEx_EnableFifoMode(&huart4);
	cBłąd |= HAL_UARTEx_DisableFifoMode(&huart4);

	// UART4 interrupt Init
	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	WłączOdbiórUART4();	//ustawia odbiornik gotowy na przyjęcie pierwszej ramki
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje USART4 jako wyjście sygnału S-Bus
// Parametry:
// *InitGPIO - wskaźnik na strukturę inicjalizacji portu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujUart4TxJakoSbus(GPIO_InitTypeDef *InitGPIO)
{
	uint8_t cBłąd;

	InitGPIO->Alternate = GPIO_AF8_UART4;
	HAL_GPIO_Init(GPIOB, InitGPIO);

	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_UART4_CLK_ENABLE();

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 100000;
	huart4.Init.WordLength = UART_WORDLENGTH_9B;
	huart4.Init.StopBits = UART_STOPBITS_2;
	huart4.Init.Parity = UART_PARITY_EVEN;
	if (huart4.Init.Mode == UART_MODE_RX)		//jeżeli właczony jest odbiornik to włącz jeszcze nadajnik a jeżeli nie to tylko nadajnik
		huart4.Init.Mode = UART_MODE_TX_RX;
	else
		huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	//huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
	huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
	cBłąd = HAL_UART_Init(&huart4);
	cBłąd |= HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_2);
	cBłąd |= HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_2);
	//cBłąd = HAL_UARTEx_EnableFifoMode(&huart4);
	cBłąd |= HAL_UARTEx_DisableFifoMode(&huart4);

	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Odbiór danych SBus z bufora kołowego i formowanie ich w ramkę
// Parametry:
// [we] *cRamkaSBus - wskaźnik na dane formowanej ramki
// [we/wy] *cWskNapRamki - wskaźnik na wskaźnik napełniania ramki SBus
// [we] *cBuforAnalizy - wskaźnik na bufor z danymi wejsciowymi
// [we] cWskNapBuf - wskaźnik napełniania bufora wejsciowego
// [wy] *cWskOprBuf - wskaźnik na wskaźnik opróżniania bufora wejściowego
// Zwraca: kod wykonania operacji: BLAD_GOTOWE gdy jest kompletna ramka, BLAD_OK gdy skończyły się dane ale nie skompetowano jeszcze ramki
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t OdbiórRamkiSBus(uint8_t *cRamkaSBus, uint8_t *cWskNapRamki, uint8_t *cBuforAnalizy, uint8_t cWskNapBuf, uint8_t *cWskOprBuf)
{
	uint8_t cDane;

	while (cWskNapBuf != *cWskOprBuf)
	{
		cDane = cBuforAnalizy[*cWskOprBuf];

		// detekcja początku ramki po wykryciu nagłówka i poprzedzającej go stopki
		if ((cDane == SBUS_NAGLOWEK) && (cBuforAnalizy[(*cWskOprBuf - 1) & MASKA_ROZM_BUF_ANA_SBUS] == SBUS_STOPKA))
			*cWskNapRamki = 0;

		cRamkaSBus[*cWskNapRamki] = cDane;
		if (*cWskNapRamki < ROZMIAR_RAMKI_SBUS)
			(*cWskNapRamki)++;

		(*cWskOprBuf)++;
		(*cWskOprBuf) &= MASKA_ROZM_BUF_ANA_SBUS;	//zapętlenie wskaźnika bufora kołowego

		if ((cDane == SBUS_STOPKA ) && (*cWskNapRamki == ROZMIAR_RAMKI_SBUS))
			return BLAD_GOTOWE;	//odebrano całą ramkę. Reszta danych będzie obrobiona w następnym przebiegu
	}
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Dekoduje dane z ramki wejściowej odbiorników RC
//i skaluje je do wygodnego w obsłudze zakresu PPM_MIN..PPM_MAX
// Parametry:
// [we] *cRamkaWe - wskaźnik na dane ramki wejsciowej
// [wy] *sKanaly - wskaźnik na tablicę kanałów RC
// Zwraca: kod błędu
// Czas wykonania: ok. 5us
////////////////////////////////////////////////////////////////////////////////
uint8_t DekodowanieRamkiBSBus(uint8_t* cRamkaWe, stRC_t *stRC)
{
	uint8_t* cNaglowek;
	uint8_t n;
	uint16_t sWartoscKanalu;
	uint8_t cBłąd = BLAD_OK;

	//dane mogą być przesunięte wzgledem początku więc znajdź nagłówek i synchronizuj się do niego
	for (n=0; n<KANALY_ODB_RC; n++)
	{
		if (*(cRamkaWe + n) == SBUS_NAGLOWEK)
		{
			cNaglowek = cRamkaWe + n;
			break;
		}
	}
	//ramka S-Bus zaczyna się od nagłówka 0x0F, który powinien być pierwszym odebranym bajtem.
	//Jeżeli nie trafia na początek, to zmniejsz liczbę odebieranych danych, tak aby przesunął się na początek
	cKorektaPoczatkuRamki = n;
	if (n > MAX_PRZESUN_NAGL)
		return BLAD_ZLA_ILOSC_DANYCH;

	sWartoscKanalu = ((uint16_t)*(cNaglowek +  1)       | (((uint16_t)*(cNaglowek +  2) << 8) & 0x7E0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[0] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek +  2) >> 3) | (((uint16_t)*(cNaglowek +  3) << 5) & 0x7E0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[1] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek +  3) >> 6) | (((uint16_t)*(cNaglowek +  4) << 2) & 0x3FC) | (((uint16_t)*(cNaglowek + 5) << 10) & 0x400));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[2] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek +  5) >> 1) | (((uint16_t)*(cNaglowek +  6) << 7) & 0x780));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[3] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek +  6) >> 4) | (((uint16_t)*(cNaglowek +  7) << 4) & 0x7F0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[4] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek +  7) >> 7) | (((uint16_t)*(cNaglowek +  8) << 1) & 0x1FE) | (((uint16_t)*(cNaglowek + 9) << 9) & 0x600));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[5] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek +  9) >> 2) | (((uint16_t)*(cNaglowek + 10) << 6) & 0x7C0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[6] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 10) >> 5) | (((uint16_t)*(cNaglowek + 11) << 3) & 0x7F8));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[7] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 12) >> 0) | (((uint16_t)*(cNaglowek + 13) << 8) & 0x700));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[8] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 13) >> 3) | (((uint16_t)*(cNaglowek + 14) << 5) & 0x7E0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[9] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 14) >> 6) | (((uint16_t)*(cNaglowek + 15) << 2) & 0x3FC) | (((uint16_t)*(cNaglowek + 16) << 10) & 0x400));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[10] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 16) >> 1) | (((uint16_t)*(cNaglowek + 17) << 7) & 0x780));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[11] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 17) >> 4) | (((uint16_t)*(cNaglowek + 18) << 4) & 0x7F0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[12] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 18) >> 7) | (((uint16_t)*(cNaglowek + 19) << 1) & 0x1FE) | (((uint16_t)*(cNaglowek + 20) << 9) & 0x600));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[13] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 20) >> 2) | (((uint16_t)*(cNaglowek + 21) << 6) & 0x7C0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[14] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(cNaglowek + 21) >> 5) | (((uint16_t)*(cNaglowek + 22) << 3) & 0x7F8));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[15] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	//ostatni bajt 23 z flagami:
	//bit 0 = kanał 17
	//bit 1 = kanał 18
	if (*(cNaglowek + 23) & 0x04)	//bit 2 = Frame Lost
		stRC->cFlagi = FRC_FAILSAFE;
	if (*(cNaglowek + 23) & 0x08)	//bit 3 = FailSafe
		stRC->cFlagi = FRC_FRAME_LOST;
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja nadrzędna wywoływana z pętli głównej, skupiająca w sobie funkcje potrzebne do obsługi ramki S-Bus
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaRamkiSBus(void)
{
	uint8_t cBłąd;

	//obsługa kanału 1
	cBłąd = OdbiórRamkiSBus(cRamkaSBus1, &cWskNapRamkiSBus1, cBuforAnalizySBus1, (uint8_t)cWskNapBufAnaSBus1, (uint8_t*)&cWskOprBufAnaSBus1);
	if (cBłąd == BLAD_GOTOWE)
	{
		cBłąd = DekodowanieRamkiBSBus(cRamkaSBus1, &stRC1);
		if (cBłąd == BLAD_OK)
		{
			stRC1.sZdekodowaneKanaly = 0xFFFF;
			stRC1.nCzasOdOstatniejRamki = PobierzCzasT7();
		}
	}

	//obsługa kanału 2
	cBłąd = OdbiórRamkiSBus(cRamkaSBus2, &cWskNapRamkiSBus2, cBuforAnalizySBus2, (uint8_t)cWskNapBufAnaSBus2, (uint8_t*)&cWskOprBufAnaSBus2);
	if (cBłąd == BLAD_GOTOWE)
	{
		cBłąd = DekodowanieRamkiBSBus(cRamkaSBus2, &stRC2);
		if (cBłąd == BLAD_OK)
		{
			stRC2.sZdekodowaneKanaly = 0xFFFF;
			stRC2.nCzasOdOstatniejRamki = PobierzCzasT7();
		}
	}

	//Jeżeli wyjscie RC1 jest ustawione jako S-Bus
	if (cKonfigWyRC[KANAL_RC1] == SERWO_SBUS)
	{
		//sprawdź czy trzeba już wysłać nową ramkę Sbus
		uint32_t nCzasRC = MinalCzasT7(nCzasWysylkiSbus);
		if (nCzasRC >= OKRES_RAMKI_SBUS)
		{
			nCzasWysylkiSbus = PobierzCzasT7();
			HAL_UART_Transmit_IT(&huart4, cBuforNadawczySBus, ROZM_BUF_ODB_SBUS);	//wyślij kolejną ramkę
		}
	}
	return cBłąd;
}



