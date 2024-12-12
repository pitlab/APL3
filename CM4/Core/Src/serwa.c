//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa serw, regulatorów i odbiorników RC
//
// (c) Pit Lab 2004
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
// Wszystkie timery są taktowane pełnym zegarem kontrolera 200MHz
#include "serwa.h"

extern uint16_t sSerwo[KANALY_SERW];	//sterowane kanałów serw
extern volatile uint16_t sOdbRC1[KANALY_ODB];	//odbierane kanały na odbiorniku 1 RC
extern volatile uint16_t sOdbRC2[KANALY_ODB];	//odbierane kanały na odbiorniku 2 RC
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern volatile uint8_t chNumerKanSerw;

////////////////////////////////////////////////////////////////////////////////
// konfiguracja timerów do generowania sygnału serw i odbioru danych RC z kanałów PPM
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujSerwa(void)
{
	//włącz odbługę timerów do generowania sygnałów serw i dekodowania PPM z odbiorników RC
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);		//generowanie impulsów dla serw [8..15] 50Hz
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);		//generowanie impulsów dla serwo[2]
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);		//odczyt wejscia RC2
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_3);		//generowanie impulsów dla serwo[3]
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);		//generowanie impulsów dla serwo[4]
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);		//generowanie impulsów dla serwo[0]
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);		//odczyt wejscia RC1
	HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);		//generowanie impulsów dla serwo[5]
	HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3);		//generowanie impulsów dla serwo[7]

	//kanał 7 korzysta z wyjscia 3NE. Nie potrafię ustawić go z poziomu HAL-a, więc niech będzie z poziomu CMSIS
	htim8.Instance->CCER |= TIM_CCER_CC3NE;

	//przeładuj 32-bitowy rejestr compare wartością startową, bo pełen obrót timera to 2^32us = 1,2 godziny. Jeżeli nie będzie zainicjowany, to generowanie impulsów ruszy dopiero po tym czasie
	htim2.Instance->CCR1 += 1500;

	for (uint8_t n=0; n<KANALY_SERW; n++)
	  sSerwo[n] = 1000 + 50*n;	//sterowane kanałów serw
	chNumerKanSerw = 8;		//wskaźnik kanału obsługuje dekoder serw, wiec inicjuj go wartoscią pierwszego kanału
}

