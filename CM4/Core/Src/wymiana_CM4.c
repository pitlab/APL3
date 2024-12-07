//////////////////////////////////////////////////////////////////////////////
//
// Międzyprocesorowa wymiana danych CM4-CM7
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

/* Do wymiany danych używam bufora pamięci SRAM4 w domenie D3. Dostęp do pamięci może być 8, 16 lub 32 bitowy, więc
 * bufor wymiany deklaruję z jednej jako unię słów 32-bitowych z drugiej struktuje taką jaką będzie trzeba
 *
 * */

#include "wymiana_CM4.h"
#include "errcode.h"



//volatile uint32_t nBuforWymianyCM4[ROZMIAR_BUF32_WYMIANY_CM4]  __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
//volatile uint32_t nBuforWymianyCM7[ROZMIAR_BUF32_WYMIANY_CM7]  __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));

volatile uint32_t* nBuforWymianyCM4 = (void*) ADRES_BUF_WYMIANY_CM4;
volatile uint32_t* nBuforWymianyCM7 = (void*) ADRES_BUF_WYMIANY_CM7;
volatile unia_wymianyCM4 uDaneCM4;
volatile unia_wymianyCM7 uDaneCM7;

////////////////////////////////////////////////////////////////////////////////
// Pobiera dane rdzenia CM7 ze wspólnej pamięci
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneWymiany_CM7(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef chErr = ERR_SEMAFOR_ZAJETY;

	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM7_TO_CM4);
	if (!nStanSemafora)
	{
		//chErr = HAL_HSEM_FastTake(HSEM_CM7_TO_CM4);
		chErr = HAL_HSEM_Take(HSEM_CM7_TO_CM4, 0);
		if (chErr == ERR_OK)
		{
			for (uint8_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM7; n++)
			{
				uDaneCM7.nSlowa[n] = nBuforWymianyCM7[n];
			}
			HAL_HSEM_Release(HSEM_CM7_TO_CM4, 0);
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wysyła dane z rdzenia CM4 do wspólnej pamięci
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDaneWymiany_CM4(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef chErr = ERR_SEMAFOR_ZAJETY;

	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM4_TO_CM7);
	if (!nStanSemafora)
	{
		//chErr = HAL_HSEM_FastTake(HSEM_CM4_TO_CM7);
		chErr = HAL_HSEM_Take(HSEM_CM4_TO_CM7, 0);
		if (chErr == ERR_OK)
		{
			for (uint8_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM4; n++)
			{
				nBuforWymianyCM4[n] = uDaneCM4.nSlowa[n];
			}
			HAL_HSEM_Release(HSEM_CM4_TO_CM7, 0);
		}
	}
	return chErr;
}
