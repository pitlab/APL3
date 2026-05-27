//////////////////////////////////////////////////////////////////////////////
//
// Międzyprocesorowa wymiana danych na rdzeniu CM7
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

/* Do wymiany danych używam bufora pamięci SRAM4 w domenie D3. Dostęp do pamięci może być 8, 16 lub 32 bitowy, więc
 * bufor wymiany deklaruję z jednej jako unię słów 32-bitowych z drugiej struktuje taką jaką będzie trzeba */

#include "SysDefCM7.h"
#include "WymianaCM7.h"
#include "KodyBledow.h"

//volatile uint32_t nFlagiMiedzyrdzeniowe __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile uint16_t sFlagiCM4 __attribute__((section(".BuforyWymianyCM7CM4_SRAM4"), used));
volatile uint16_t sFlagiCM7 __attribute__((section(".BuforyWymianyCM7CM4_SRAM4"), used));
volatile uint32_t nBuforWymianyCM4[ROZMIAR_BUF32_WYMIANY_CM4] __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile uint32_t nBuforWymianyCM7[ROZMIAR_BUF32_WYMIANY_CM7] __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
unia_wymianyCM4_t uDaneCM4;
unia_wymianyCM7_t uDaneCM7;
uint32_t nLicznikSynchronizacji = 0;



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane z rdzenia CM4
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneWymiany_CM4(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef cBłąd = BLAD_SEMAFOR_ZAJETY;

	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM4_TO_CM7);
	if (!nStanSemafora)
	{
		cBłąd = HAL_HSEM_Take(HSEM_CM4_TO_CM7, HSEM_CM7);
		if (cBłąd == BLAD_OK)
		{
			//Pobierz dane tylko gdy są ustawione nowe
			__DMB();	//Data Memory Barrier. Ensures the apparent order of the explicit memory operations before and after the instruction, without ensuring their completion.
			if (sFlagiCM4 & FMR_SA_DANE_CM4)
			{
				for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM4; n++)
					uDaneCM4.nSlowa[n] = nBuforWymianyCM4[n];
				sFlagiCM4 &= ~FMR_SA_DANE_CM4;
				__DSB();	//Data Synchronization Barrier. Acts as a special kind of Data Memory Barrier. It completes when all explicit memory accesses before this instruction complete.
			}
			HAL_HSEM_Release(HSEM_CM4_TO_CM7, HSEM_CM7);
		}
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Ustaw dane z rdzenia CM7
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDaneWymiany_CM7(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef cBłąd = BLAD_SEMAFOR_ZAJETY;
	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM7_TO_CM4);
	if (!nStanSemafora)
	{
		cBłąd = HAL_HSEM_Take(HSEM_CM7_TO_CM4, HSEM_CM7);
		if (cBłąd == BLAD_OK)
		{
			__DMB();	//Data Memory Barrier. Ensures the apparent order of the explicit memory operations before and after the instruction, without ensuring their completion.
			//if ((sFlagiCM7 & FMR_SA_DANE_CM7) != FMR_SA_DANE_CM7)	//ustaw tylko gdy poprzednie są odczytane - blokuje LCD
			{
				for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM7; n++)
					nBuforWymianyCM7[n] = uDaneCM7.nSlowa[n];
				__DSB();	//Data Synchronization Barrier. Acts as a special kind of Data Memory Barrier. It completes when all explicit memory accesses before this instruction complete.
				HAL_HSEM_Release(HSEM_CM7_TO_CM4, HSEM_CM7);
			}
			sFlagiCM7 |= FMR_SA_DANE_CM7;

		}
	}
	return cBłąd;
}
