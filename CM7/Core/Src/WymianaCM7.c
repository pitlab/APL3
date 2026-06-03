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
volatile uint16_t sFlagiCM4 __attribute__((section(".SekcjaWymiany_SRAM4"), used));
volatile uint16_t sFlagiCM7 __attribute__((section(".SekcjaWymiany_SRAM4"), used));
volatile uint32_t nBuforWymianyCM4[ROZMIAR_BUF32_WYMIANY_CM4] __attribute__((section(".SekcjaWymiany_SRAM4"), used));
volatile uint32_t nBuforWymianyCM7[ROZMIAR_BUF32_WYMIANY_CM7] __attribute__((section(".SekcjaWymiany_SRAM4")));
unia_wymianyCM4_t uDaneCM4;
unia_wymianyCM7_t uDaneCM7;
uint32_t nLicznikSynchronizacji = 0;



////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja miedzyrdzeniowej wymiany danych
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void InicjujWymiane(void)
{
	HAL_NVIC_SetPriority(HSEM1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(HSEM1_IRQn);
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_CM4_TO_CM7));
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane z rdzenia CM4
// Parametry: nic
// Zwraca: kod błędu
// Czas trwania 12,0us dla 350 słów v506. Bez ustawionego MPU_ACCESS_SHAREABLE trwa 10,4 ale nie działa.
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneWymiany_CM4(void)
{
	uint8_t cBłąd = BLAD_SEMAFOR_ZAJETY;

	if (sFlagiCM4 & FMR_SPRAWDZ_CM4)
	{
		cBłąd = HAL_HSEM_Take(HSEM_CM4_TO_CM7, HSEM_CM7);
		if (cBłąd == BLAD_OK)
		{
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_SET);			//kanał serw 7 skonfigurowany jako IO
			for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM4; n++)
				uDaneCM4.nSlowa[n] = nBuforWymianyCM4[n];
			HAL_GPIO_WritePin(GPIOI, GPIO_PIN_10, GPIO_PIN_RESET);			//kanał serw 7 skonfigurowany jako IO
			sFlagiCM4 &= ~(FMR_SA_DANE_CM4 | FMR_SPRAWDZ_CM4);		//flagi zdejmij dopiero po udanym odczycie
			//__DSB();	//Data Synchronization Barrier. Acts as a special kind of Data Memory Barrier. It completes when all explicit memory accesses before this instruction complete.
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
	uint8_t cBłąd = BLAD_SEMAFOR_ZAJETY;
	cBłąd = HAL_HSEM_Take(HSEM_CM7_TO_CM4, HSEM_CM7);
	if (cBłąd == BLAD_OK)
	{
		for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM7; n++)
			nBuforWymianyCM7[n] = uDaneCM7.nSlowa[n];
		HAL_HSEM_Release(HSEM_CM7_TO_CM4, HSEM_CM7);
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Callback uruchamiany gdy został zwolniony semafor
// Parametry: SemMask
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_HSEM_FreeCallback(uint32_t SemMask)
{
      /* Reactivate the HSEM notification for Semaphore 0 */
      HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_CM4_TO_CM7));
      sFlagiCM4 |= FMR_SPRAWDZ_CM4;

}
