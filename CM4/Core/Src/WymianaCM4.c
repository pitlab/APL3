//////////////////////////////////////////////////////////////////////////////
//
// Międzyprocesorowa wymiana danych na rdzeniu CM4
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "WymianaCM4.h"
#include "KodyBledow.h"

/* Do wymiany danych używam bufora pamięci SRAM4 w domenie D3. Dostęp do pamięci może być 8, 16 lub 32 bitowy, więc
 * bufor wymiany deklaruję z jednej strony jako unię słów 32-bitowych z drugiej strukturę taką jaką będzie trzeba
 * */

//volatile uint32_t nFlagiMiedzyrdzeniowe __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile uint16_t sFlagiCM4 __attribute__((section(".BuforyWymianyCM7CM4_SRAM4"), used));
volatile uint16_t sFlagiCM7 __attribute__((section(".BuforyWymianyCM7CM4_SRAM4"), used));
volatile uint32_t nBuforWymianyCM4[ROZMIAR_BUF32_WYMIANY_CM4] __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile uint32_t nBuforWymianyCM7[ROZMIAR_BUF32_WYMIANY_CM7] __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile unia_wymianyCM4_t uDaneCM4;
volatile unia_wymianyCM7_t uDaneCM7;



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane rdzenia CM7 ze wspólnej pamięci
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania: 3,68us na 17 słów -> 0,22us/słowo
// Czas wykonania: 2,8ms na 14 słów -> 200us/słowo wersja 485
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneWymiany_CM7(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef cBłąd = BLAD_SEMAFOR_ZAJETY;

	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM7_TO_CM4);
	if (!nStanSemafora)
	{
		cBłąd = HAL_HSEM_Take(HSEM_CM7_TO_CM4, 0);
		if (cBłąd == BLAD_OK)
		{
			__DMB();	//Data Memory Barrier. Ensures the apparent order of the explicit memory operations before and after the instruction, without ensuring their completion.
			if (sFlagiCM7 & FMR_SA_DANE_CM7)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);			//kanał serw 1 skonfigurowany jako IO
				for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM7; n++)
					uDaneCM7.nSlowa[n] = nBuforWymianyCM7[n];
				sFlagiCM7 &= ~FMR_SA_DANE_CM7;
				__DSB();	//Data Synchronization Barrier. Acts as a special kind of Data Memory Barrier. It completes when all explicit memory accesses before this instruction complete.
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);		//kanał serw 1 skonfigurowany jako IO
			}
			HAL_HSEM_Release(HSEM_CM7_TO_CM4, 0);
		}
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła dane z rdzenia CM4 do wspólnej pamięci
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania: 27,4us na 186 słów -> 0,15us/słowo
// Czas wykonania: 44ms na 350 słów -> 126us/słowo	wersja 485
// Czas wykonania: 10,5ms na 350 słów -> 126us/słowo	wersja 497
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDaneWymiany_CM4(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef cBłąd = BLAD_SEMAFOR_ZAJETY;

	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM4_TO_CM7);
	if (!nStanSemafora)
	{
		cBłąd = HAL_HSEM_Take(HSEM_CM4_TO_CM7, 0);
		if (cBłąd == BLAD_OK)
		{

			__DMB();	//Data Memory Barrier. Ensures the apparent order of the explicit memory operations before and after the instruction, without ensuring their completion.
			//if ((sFlagiCM4 & FMR_SA_DANE_CM4) != FMR_SA_DANE_CM4)	//ustaw tylko gdy poprzednie zostały odczytane - nie czyta stanu zmiennej
			{
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_SET);			//kanał serw 7 skonfigurowany jako IO
				for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM4; n++)
					nBuforWymianyCM4[n] = uDaneCM4.nSlowa[n];
				sFlagiCM4 |= FMR_SA_DANE_CM4;	//ustaw flagę obecności nowych danych
				__DSB();	//Data Synchronization Barrier. Acts as a special kind of Data Memory Barrier. It completes when all explicit memory accesses before this instruction complete.
				HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_RESET);			//kanał serw 7 skonfigurowany jako IO
			}
			HAL_HSEM_Release(HSEM_CM4_TO_CM7, 0);
		}
	}
	return cBłąd;
}
