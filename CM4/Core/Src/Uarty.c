//////////////////////////////////////////////////////////////////////////////
//
// Moduł wspólnej obsługi przerwań UART-ów
//
// Wszystkie UARTy obsługiwane przez rdzeń CM4 pracują w trybie DMA i są obsługiwane w zbiorczym callbacku odbiorczym
// Dane odbierane są niewielkimi porcjami, po to aby nie musiały zbierać się zbyt długo.
// W callbacku dane są kopiowane do większych buforów analizy, które opróżniane są okresowo w pętli głównej.
// Rozmiar bufora analizy powinien być na tyle duży, aby pomieścił dane mogące się zebrać w trakcie obiegu pętli głównej.
// Wszystkie bufory odbiorcze są buformami kołowymi. Mają dwa wskaźniki: napełniania i opróżniania bufora.
// Gdy bufor jest pusty to oba wskaźniki wskazują na tą samą pozycję.
// Rozmiar bufora odbiorczego powinien być liczbą 2 do n-tej potęgi, aby w prosty sposób zawijać bufor.
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Uarty.h"
#include <GNSS.h>
#include <stdio.h>
#include "WeWyRC.h"
#include "PetlaGlowna.h"
#include "SBus.h"
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
extern volatile uint8_t chWskNapBufAnaSBus1, chWskOprBufAnaSBus1; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus1
extern uint8_t chBuforAnalizySBus2[ROZMIAR_BUF_ANA_SBUS];
extern volatile uint8_t chWskNapBufAnaSBus2, chWskOprBufAnaSBus2; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych S-Bus2
extern uint8_t chBuforAnalizyCrossfire[ROZMIAR_BUF_ANA_CRSF];
extern volatile uint8_t chWskNapBufAnaSRSF, chWskOprBufAnaCRSF;
extern uint8_t chBuforAnalizyGNSS[ROZMIAR_BUF_ANA_GNSS];
extern volatile uint8_t chWskNapBaGNSS, chWskOprBaGNSS;		//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych GNSS


////////////////////////////////////////////////////////////////////////////////
// Callback przerwania UARTA.
// Parametry:
// *huart - uchwyt uarta
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	extern stRC_t stRC;	//struktura danych odbiorników RC

	//Przepisuje odebrane dane GNSS z małego bufora do większego bufora kołowego analizy protokołu
	if (huart->Instance == UART8)
	{
		switch (cKonfiguracjaUart8)
		{
		case U_GNSS1:
			for (uint8_t n=0; n<ROZMIAR_BUF_ODB_GNSS; n++)
			{
				chBuforAnalizyGNSS[chWskNapBaGNSS] = chBuforOdbioruUart8[n];
				chWskNapBaGNSS++;
				chWskNapBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;	//zapętlenie wskaźnika bufora kołowego
			}
			break;

		case U_CRSF1:
			for (uint8_t n=0; n<ROZMIAR_BUF_ODB_GNSS; n++)
			{
				chBuforAnalizyCrossfire[chWskNapBufAnaSRSF] = chBuforOdbioruUart8[n];
				chWskNapBufAnaSRSF++;
				chWskNapBufAnaSRSF &= MASKA_ROZM_BUF_ANA_CRSF;
			}
			break;
		}
		WłączOdbiórUART8();	//uzbraja odbiór następnej porcji danych
	}

	if (huart->Instance == UART4)	//dane z SBUS1
	{
		stRC.nCzasWe1 = PobierzCzas();	//czas przyjścia ramki SBus1
		for (uint8_t n=0; n<ROZMIAR_BUF_ODB_UART4; n++)
		{
			chBuforAnalizySBus1[chWskNapBufAnaSBus1] = chBuforOdbioruUart4[n];
			chWskNapBufAnaSBus1++;
			chWskNapBufAnaSBus1 &= MASKA_ROZM_BUF_ANA_SBUS;	//zapętlenie wskaźnika bufora kołowego
		}
		WłączOdbiórUART4();	//uzbraja odbiór następnej porcji danych
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);			//kanał serw 2 skonfigurowany jako IO
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);				//kanał serw 4 skonfigurowany jako IO
	}

	if (huart->Instance == USART2)		//dane z SBUS2
	{
		stRC.nCzasWe2 = PobierzCzas();	//czas przyjścia ramki SBus2
		for (uint8_t n=0; n<ROZMIAR_BUF_ODB_UART2; n++)
		{
			chBuforAnalizySBus2[chWskNapBufAnaSBus2] = chBuforOdbioruUart2[n];
			chWskNapBufAnaSBus2++;
			chWskNapBufAnaSBus2 &= MASKA_ROZM_BUF_ANA_SBUS;	//zapętlenie wskaźnika bufora kołowego
		}
		WłączOdbiórUART2();	//uzbraja odbiór następnej porcji danych
	}
}




////////////////////////////////////////////////////////////////////////////////
// Włącza odbiór danych na UART2
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączOdbiórUART2(void)
{
	HAL_UART_Receive_IT(&huart2, chBuforOdbioruUart2, ROZMIAR_BUF_ODB_UART2);
}



////////////////////////////////////////////////////////////////////////////////
// Włącza odbiór danych na UART2
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączOdbiórUART4(void)
{
	HAL_UART_Receive_DMA(&huart4, chBuforOdbioruUart4, ROZMIAR_BUF_ODB_UART4);
}



////////////////////////////////////////////////////////////////////////////////
// Włącza odbiór danych na UART2
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WłączOdbiórUART8(void)
{
	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruUart8, ROZMIAR_BUF_ODB_UART8);
}



////////////////////////////////////////////////////////////////////////////////
// Callback odbiorczy UART - obecnie nie używane
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	//odbiór GPS
	if (huart->Instance == UART8)
	{
		//przepisz dane odebrane do większego bufora kołowego analizy danych
		for (uint8_t n=0; n<Size; n++)
		{
			chBuforAnalizyGNSS[chWskNapBaGNSS] = chBuforOdbioruUart8[n];
			chWskNapBaGNSS++;
			chWskNapBaGNSS &= MASKA_ROZM_BUF_ANA_GNSS;
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart8, chBuforOdbioruUart8, ROZMIAR_BUF_ODB_GNSS);	//wznów odbiór
	}

	//odbiór SBus2
	if (huart->Instance == USART2)
	{

		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, chBuforOdbioruUart2, ROZMIAR_BUF_ODB_UART2);	//wznów odbiór
	}

	//odbiór SBus1
	if (huart->Instance == UART4)
	{

		HAL_UARTEx_ReceiveToIdle_DMA(&huart4, chBuforOdbioruUart4, ROZMIAR_BUF_ODB_UART4);	//wznów odbiór
	}
}




