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


uint8_t chBuforNadawczySBus[ROZMIAR_RAMKI_SBUS] =  {0x0f,0x01,0x04,0x20,0x00,0xff,0x07,0x40,0x00,0x02,0x10,0x80,0x2c,0x64,0x21,0x0b,0x59,0x08,0x40,0x00,0x02,0x10,0x80,0x00};
uint8_t chBuforAnalizySBus1[ROZMIAR_BUF_ANA_SBUS];
uint8_t chBuforAnalizySBus2[ROZMIAR_BUF_ANA_SBUS];
volatile uint8_t chWskNapBufAnaSBus1, chWskOprBufAnaSBus1; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus1
volatile uint8_t chWskNapBufAnaSBus2, chWskOprBufAnaSBus2; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus2
uint8_t chWskNapRamkiSBus1, chWskNapRamkiSBus2;				//wskaźniki napełniania ramek SBus
uint8_t chRamkaSBus1[ROZMIAR_RAMKI_SBUS];
uint8_t chRamkaSBus2[ROZMIAR_RAMKI_SBUS];
uint8_t chKorektaPoczatkuRamki;
uint32_t nCzasWysylkiSbus;
extern stRC_t stRC1, stRC2;	//struktura danych odbiorników RC1 i RC2
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart2_rx;

extern UART_HandleTypeDef huart4;
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern uint8_t chKonfigWyRC[LICZBA_WYJSC_RC];



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
// [we] *chRamkaSBus - wskaźnik na dane formowanej ramki
// [we/wy] *chWskNapRamki - wskaźnik na wskaźnik napełniania ramki SBus
// [we] *chBuforAnalizy - wskaźnik na bufor z danymi wejsciowymi
// [we] chWskNapBuf - wskaźnik napełniania bufora wejsciowego
// [wy] *chWskOprBuf - wskaźnik na wskaźnik opróżniania bufora wejściowego
// Zwraca: kod wykonania operacji: BLAD_GOTOWE gdy jest kompletna ramka, BLAD_OK gdy skończyły się dane ale nie skompetowano jeszcze ramki
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t OdbiórRamkiSBus(uint8_t *chRamkaSBus, uint8_t *chWskNapRamki, uint8_t *chBuforAnalizy, uint8_t chWskNapBuf, uint8_t *chWskOprBuf)
{
	uint8_t chDane;

	while (chWskNapBuf != *chWskOprBuf)
	{
		chDane = chBuforAnalizy[*chWskOprBuf];

		// detekcja początku ramki po wykryciu nagłówka i poprzedzającej go stopki
		if ((chDane == SBUS_NAGLOWEK) && (chBuforAnalizy[(*chWskOprBuf - 1) & MASKA_ROZM_BUF_ANA_SBUS] == SBUS_STOPKA))
			*chWskNapRamki = 0;

		chRamkaSBus[*chWskNapRamki] = chDane;
		if (*chWskNapRamki < ROZMIAR_RAMKI_SBUS)
			(*chWskNapRamki)++;

		(*chWskOprBuf)++;
		(*chWskOprBuf) &= MASKA_ROZM_BUF_ANA_SBUS;	//zapętlenie wskaźnika bufora kołowego

		if ((chDane == SBUS_STOPKA ) && (*chWskNapRamki == ROZMIAR_RAMKI_SBUS))
			return BLAD_GOTOWE;	//odebrano całą ramkę. Reszta danych będzie obrobiona w następnym przebiegu
	}
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Dekoduje dane z ramki wejściowej odbiorników RC
//i skaluje je do wygodnego w obsłudze zakresu PPM_MIN..PPM_MAX
// Parametry:
// [we] *chRamkaWe - wskaźnik na dane ramki wejsciowej
// [wy] *sKanaly - wskaźnik na tablicę kanałów RC
// Zwraca: kod błędu
// Czas wykonania: ok. 5us
////////////////////////////////////////////////////////////////////////////////
uint8_t DekodowanieRamkiBSBus(uint8_t* chRamkaWe, stRC_t *stRC)
{
	uint8_t* chNaglowek;
	uint8_t n;
	uint16_t sWartoscKanalu;
	uint8_t cBłąd = BLAD_OK;

	//dane mogą być przesunięte wzgledem początku więc znajdź nagłówek i synchronizuj się do niego
	for (n=0; n<KANALY_ODB_RC; n++)
	{
		if (*(chRamkaWe + n) == SBUS_NAGLOWEK)
		{
			chNaglowek = chRamkaWe + n;
			break;
		}
	}
	//ramka S-Bus zaczyna się od nagłówka 0x0F, który powinien być pierwszym odebranym bajtem.
	//Jeżeli nie trafia na początek, to zmniejsz liczbę odebieranych danych, tak aby przesunął się na początek
	chKorektaPoczatkuRamki = n;
	if (n > MAX_PRZESUN_NAGL)
		return BLAD_ZLA_ILOSC_DANYCH;

	sWartoscKanalu = ((uint16_t)*(chNaglowek +  1)       | (((uint16_t)*(chNaglowek +  2) << 8) & 0x7E0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[0] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek +  2) >> 3) | (((uint16_t)*(chNaglowek +  3) << 5) & 0x7E0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[1] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek +  3) >> 6) | (((uint16_t)*(chNaglowek +  4) << 2) & 0x3FC) | (((uint16_t)*(chNaglowek + 5) << 10) & 0x400));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[2] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek +  5) >> 1) | (((uint16_t)*(chNaglowek +  6) << 7) & 0x780));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[3] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek +  6) >> 4) | (((uint16_t)*(chNaglowek +  7) << 4) & 0x7F0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[4] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek +  7) >> 7) | (((uint16_t)*(chNaglowek +  8) << 1) & 0x1FE) | (((uint16_t)*(chNaglowek + 9) << 9) & 0x600));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[5] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek +  9) >> 2) | (((uint16_t)*(chNaglowek + 10) << 6) & 0x7C0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[6] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 10) >> 5) | (((uint16_t)*(chNaglowek + 11) << 3) & 0x7F8));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[7] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 12) >> 0) | (((uint16_t)*(chNaglowek + 13) << 8) & 0x700));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[8] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 13) >> 3) | (((uint16_t)*(chNaglowek + 14) << 5) & 0x7E0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[9] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 14) >> 6) | (((uint16_t)*(chNaglowek + 15) << 2) & 0x3FC) | (((uint16_t)*(chNaglowek + 16) << 10) & 0x400));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[10] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 16) >> 1) | (((uint16_t)*(chNaglowek + 17) << 7) & 0x780));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[11] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 17) >> 4) | (((uint16_t)*(chNaglowek + 18) << 4) & 0x7F0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[12] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 18) >> 7) | (((uint16_t)*(chNaglowek + 19) << 1) & 0x1FE) | (((uint16_t)*(chNaglowek + 20) << 9) & 0x600));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[13] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 20) >> 2) | (((uint16_t)*(chNaglowek + 21) << 6) & 0x7C0));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[14] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	sWartoscKanalu = (((uint16_t)*(chNaglowek + 21) >> 5) | (((uint16_t)*(chNaglowek + 22) << 3) & 0x7F8));
	if (sWartoscKanalu < WE_RC_MAX)
		stRC->sKanaly[15] = sWartoscKanalu;
	else
		cBłąd = BLAD_ZLE_DANE;

	//ostatni bajt 23 z flagami:
	//bit 0 = kanał 17
	//bit 1 = kanał 18
	if (*(chNaglowek + 23) & 0x04)	//bit 2 = Frame Lost
		stRC->cFlagi = FRC_FAILSAFE;
	if (*(chNaglowek + 23) & 0x08)	//bit 3 = FailSafe
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
	cBłąd = OdbiórRamkiSBus(chRamkaSBus1, &chWskNapRamkiSBus1, chBuforAnalizySBus1, (uint8_t)chWskNapBufAnaSBus1, (uint8_t*)&chWskOprBufAnaSBus1);
	if (cBłąd == BLAD_GOTOWE)
	{
		cBłąd = DekodowanieRamkiBSBus(chRamkaSBus1, &stRC1);
		if (cBłąd == BLAD_OK)
		{
			stRC1.sZdekodowaneKanaly = 0xFFFF;
			stRC1.nCzasOdOstatniejRamki = PobierzCzasT7();
		}
	}

	//obsługa kanału 2
	cBłąd = OdbiórRamkiSBus(chRamkaSBus2, &chWskNapRamkiSBus2, chBuforAnalizySBus2, (uint8_t)chWskNapBufAnaSBus2, (uint8_t*)&chWskOprBufAnaSBus2);
	if (cBłąd == BLAD_GOTOWE)
	{
		cBłąd = DekodowanieRamkiBSBus(chRamkaSBus2, &stRC2);
		if (cBłąd == BLAD_OK)
		{
			stRC2.sZdekodowaneKanaly = 0xFFFF;
			stRC2.nCzasOdOstatniejRamki = PobierzCzasT7();
		}
	}

	//Jeżeli wyjscie RC1 jest ustawione jako S-Bus
	if (chKonfigWyRC[KANAL_RC1] == SERWO_SBUS)
	{
		//sprawdź czy trzeba już wysłać nową ramkę Sbus
		uint32_t nCzasRC = MinalCzasT7(nCzasWysylkiSbus);
		if (nCzasRC >= OKRES_RAMKI_SBUS)
		{
			nCzasWysylkiSbus = PobierzCzasT7();
			HAL_UART_Transmit_DMA(&huart4, chBuforNadawczySBus, ROZM_BUF_ODB_SBUS);	//wyślij kolejną ramkę
		}
	}
	return cBłąd;
}



