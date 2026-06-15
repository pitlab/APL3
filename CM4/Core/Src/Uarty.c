//////////////////////////////////////////////////////////////////////////////
//
// Moduł wspólnej obsługi przerwań UART-ów
//
// Wszystkie UART-y obsługiwane przez rdzeń CM4 w trybie nadawczum pracują z buforem liniowym na przerwaniach.
// W trybie odbiorczym pracują w trybie circular DMA i są obsługiwane w zbiorczych callbackach od napełnienia polowy i całego bufora
// Dzięki trybowi circular bufor nie musi być duży i nie trzeba długo czekać na napelnienie bufora.
// W callbackach dane są kopiowane do większych buforów analizy, które opróżniane są okresowo w pętli głównej.
// Rozmiar bufora analizy powinien być na tyle duży, aby pomieścił dane mogące się zebrać w trakcie obiegu pętli głównej.
// Wszystkie bufory analizy danych są kołowe. Mają dwa wskaźniki: napełniania i opróżniania bufora.
// Gdy bufor jest pusty to oba wskaźniki wskazują na tą samą pozycję.
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Uarty.h"
#include <GNSS.h>
#include <stdio.h>
#include <SBus.h>
#include "WeWyRC.h"
#include "Czas.h"
#include "Crossfire.h"


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart8;
uint8_t cKonfiguracjaUart8 = U_CRSF1;		//Tymczasowo zamiast GNSS jest Crossfire
//uint8_t chBuforNadawczyUART8[ROZMIAR_BUF_NAD_UART8];
uint8_t chBuforOdbioruUart8[ROZMIAR_BUF_ODB_UART8];
uint8_t chBuforOdbioruUart2[ROZMIAR_BUF_ODB_UART2];
uint8_t chBuforOdbioruUart4[ROZMIAR_BUF_ODB_UART4];


extern uint8_t chBuforAnalizySBus1[ROZMIAR_BUF_ANA_SBUS];
extern volatile uint8_t chWskNapBufAnaSBus1; 	//wskaźniki napełniania kołowego bufora odbiorczego analizy danych S-Bus1
extern uint8_t chBuforAnalizySBus2[ROZMIAR_BUF_ANA_SBUS];
extern volatile uint8_t chWskNapBufAnaSBus2; 	//wskaźniki napełniania kołowego bufora odbiorczego analizy danych S-Bus2
extern uint8_t chBuforAnalizyCrossfire[ROZMIAR_BUF_ANA_CRSF];
extern volatile uint8_t chWskNapBufAnaCRSF;
extern uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern volatile uint8_t chWskNapBufAnaGNSS;		//wskaźnik napełniania kołowego bufora odbiorczego analizy danych GNSS



////////////////////////////////////////////////////////////////////////////////
// Callback zapełnienia połowy bufora UARTA.
// Parametry:
// *huart - uchwyt uarta
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART8)
	{
		switch (cKonfiguracjaUart8)
		{
		case U_GNSS1:
			for (uint8_t n = 0; n < ROZMIAR_BUF_ODB_UART8 / 2; n++)
			{
				chBuforAnalizyGNSS[chWskNapBufAnaGNSS] = chBuforOdbioruUart8[n];
				chWskNapBufAnaGNSS++;
				if (chWskNapBufAnaGNSS >= ROZMIAR_BUF_ANA_GNSS)
					chWskNapBufAnaGNSS = 0;		//zapętlenie wskaźnika bufora kołowego
			}
			break;

		case U_CRSF1:
			for (uint8_t n = 0; n < ROZMIAR_BUF_ODB_UART8 / 2; n++)
			{
				chBuforAnalizyCrossfire[chWskNapBufAnaCRSF] = chBuforOdbioruUart8[n];
				chWskNapBufAnaCRSF++;
				if (chWskNapBufAnaCRSF >= ROZMIAR_BUF_ANA_CRSF)
				{
					chWskNapBufAnaCRSF = 0;		//zapętlenie wskaźnika bufora kołowego
					//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);			//kanał serw 7 skonfigurowany jako IO
				}
			}
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);		//kanał serw 1 skonfigurowany jako IO
			break;
		}
	}

	if (huart->Instance == UART4)	//dane z SBUS1
	{
		for (uint8_t n = 0; n < ROZMIAR_BUF_ODB_UART4 / 2; n++)
		{
			chBuforAnalizySBus1[chWskNapBufAnaSBus1] = chBuforOdbioruUart4[n];
			chWskNapBufAnaSBus1++;
			if (chWskNapBufAnaSBus1 >= ROZMIAR_BUF_ANA_SBUS)
				chWskNapBufAnaSBus1 = 0;	//zapętlenie wskaźnika bufora kołowego
		}
	}

	if (huart->Instance == USART2)		//dane z SBUS2
	{
		for (uint8_t n = 0;  n < ROZMIAR_BUF_ODB_UART2 / 2; n++)
		{
			chBuforAnalizySBus2[chWskNapBufAnaSBus2] = chBuforOdbioruUart2[n];
			chWskNapBufAnaSBus2++;
			if (chWskNapBufAnaSBus2 >= ROZMIAR_BUF_ANA_SBUS)
				chWskNapBufAnaSBus2 = 0;		//zapętlenie wskaźnika bufora kołowego
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Callback zapełnienia bufora UARTA.
// Parametry:
// *huart - uchwyt uarta
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Przepisuje odebrane dane GNSS z małego bufora do większego bufora kołowego analizy protokołu
	if (huart->Instance == UART8)
	{
		switch (cKonfiguracjaUart8)
		{
		case U_GNSS1:
			for (uint8_t n = 0; n < ROZMIAR_BUF_ODB_UART8 / 2; n++)
			{
				chBuforAnalizyGNSS[chWskNapBufAnaGNSS] = chBuforOdbioruUart8[n + ROZMIAR_BUF_ODB_UART8 / 2];
				chWskNapBufAnaGNSS++;
				if (chWskNapBufAnaGNSS >= ROZMIAR_BUF_ANA_GNSS)
					chWskNapBufAnaGNSS = 0;		//zapętlenie wskaźnika bufora kołowego
			}
			break;

		case U_CRSF1:
			for (uint8_t n = 0; n < ROZMIAR_BUF_ODB_UART8 / 2; n++)
			{
				chBuforAnalizyCrossfire[chWskNapBufAnaCRSF] = chBuforOdbioruUart8[n + ROZMIAR_BUF_ODB_UART8 / 2];
				chWskNapBufAnaCRSF++;
				if (chWskNapBufAnaCRSF >= ROZMIAR_BUF_ANA_CRSF)
				{
					chWskNapBufAnaCRSF = 0;
					//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);			//kanał serw 7 skonfigurowany jako IO
				}
			}
			//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);			//kanał serw 1 skonfigurowany jako IO
			break;
		}
	}

	if (huart->Instance == UART4)	//dane z SBUS1
	{
		for (uint8_t n = 0;  n<ROZMIAR_BUF_ODB_UART4 / 2; n++)
		{
			chBuforAnalizySBus1[chWskNapBufAnaSBus1] = chBuforOdbioruUart4[n + ROZMIAR_BUF_ODB_UART4 / 2];
			chWskNapBufAnaSBus1++;
			if (chWskNapBufAnaSBus1 >= ROZMIAR_BUF_ANA_SBUS)
				chWskNapBufAnaSBus1 = 0;		//zapętlenie wskaźnika bufora kołowego
		}
	}

	if (huart->Instance == USART2)		//dane z SBUS2
	{
		for (uint8_t n = 0;  n<ROZMIAR_BUF_ODB_UART2 / 2; n++)
		{
			chBuforAnalizySBus2[chWskNapBufAnaSBus2] = chBuforOdbioruUart2[n + ROZMIAR_BUF_ODB_UART2 / 2];
			chWskNapBufAnaSBus2++;
			if (chWskNapBufAnaSBus2 >= ROZMIAR_BUF_ANA_SBUS)
				chWskNapBufAnaSBus2 = 0;		//zapętlenie wskaźnika bufora kołowego
		}
	}
}




////////////////////////////////////////////////////////////////////////////////
// Włącza odbiór danych na UART2
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączOdbiórUART2(void)
{
	//HAL_UART_Receive_IT(&huart2, chBuforOdbioruUart2, ROZMIAR_BUF_ODB_UART2);
	HAL_UART_Receive_DMA(&huart2, chBuforOdbioruUart2, ROZMIAR_BUF_ODB_UART2);
}



////////////////////////////////////////////////////////////////////////////////
// Włącza odbiór danych na UART4
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączOdbiórUART4(void)
{
	HAL_UART_Receive_DMA(&huart4, chBuforOdbioruUart4, ROZMIAR_BUF_ODB_UART4);
}



////////////////////////////////////////////////////////////////////////////////
// Włącza odbiór danych na UART8
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączOdbiórUART8(void)
{
	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruUart8, ROZMIAR_BUF_ODB_UART8);
}





