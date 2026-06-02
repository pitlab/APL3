/////////////////////////////////////////////////////////////////////////////
//
// Międzyprocesorowa wymiana danych na rdzeniu CM4
//
//
// (c) PitLab 2024-6
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "WymianaCM4.h"
#include "KodyBledow.h"

/* Do wymiany danych używam bufora pamięci SRAM4 w domenie D3. Dostęp do pamięci może być 8, 16 lub 32 bitowy, więc
 * bufor wymiany deklaruję z jednej strony jako unię słów 32-bitowych z drugiej strukturę taką jaką będzie trzeba
 * */

volatile uint16_t sFlagiCM4 __attribute__((section(".BuforyWymianyCM7CM4_SRAM4"), used));
volatile uint16_t sFlagiCM7 __attribute__((section(".BuforyWymianyCM7CM4_SRAM4"), used));
volatile uint32_t nBuforWymianyCM4[ROZMIAR_BUF32_WYMIANY_CM4] __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile uint32_t nBuforWymianyCM7[ROZMIAR_BUF32_WYMIANY_CM7] __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile unia_wymianyCM4_t uDaneCM4;
volatile unia_wymianyCM7_t uDaneCM7;
uint8_t cLicznikOdswiezaniaCM4;


////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja miedzyrdzeniowej wymiany danych
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void InicjujWymiane(void)
{
	HAL_NVIC_SetPriority(HSEM2_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(HSEM2_IRQn);
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_CM7_TO_CM4));
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane rdzenia CM7 ze wspólnej pamięci
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania: 3,68us na 17 słów -> 0,22us/słowo
// Czas wykonania: 2,8ms na 14 słów -> 200us/słowo wersja 485
// Czas wykonania: 2,04us wersja 506
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneWymiany_CM7(void)
{
	HAL_StatusTypeDef cBłąd = BLAD_SEMAFOR_ZAJETY;

	if (sFlagiCM7 & FMR_SPRAWDZ_CM7)	//jeżeli przyszedł callback z informacją o zwolnieniu semafora CM7
	{
		cBłąd = HAL_HSEM_Take(HSEM_CM7_TO_CM4, HSEM_CM4);
		if (cBłąd == BLAD_OK)
		{
			for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM7; n++)
				uDaneCM7.nSlowa[n] = nBuforWymianyCM7[n];
			HAL_HSEM_Release(HSEM_CM7_TO_CM4, HSEM_CM4);
			sFlagiCM7 &= ~(FMR_SA_DANE_CM7 | FMR_SPRAWDZ_CM7);		//flagi zdejmij po udanym odczycie
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
// Czas wykonania: 41,2us na 350 słów - wersja 506
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDaneWymiany_CM4(void)
{
	HAL_StatusTypeDef cBłąd = BLAD_SEMAFOR_ZAJETY;

	cBłąd = HAL_HSEM_Take(HSEM_CM4_TO_CM7, HSEM_CM4);
	if (cBłąd == BLAD_OK)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);			//kanał serw 1 skonfigurowany jako IO
		//__DMB();	//Data Memory Barrier. Ensures the apparent order of the explicit memory operations before and after the instruction, without ensuring their completion.
		if (((sFlagiCM4 & FMR_SA_DANE_CM4) != FMR_SA_DANE_CM4) || (cLicznikOdswiezaniaCM4 == 0))	//ustaw tylko gdy poprzednie zostały odczytane
		{
			for (uint16_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM4; n++)
				nBuforWymianyCM4[n] = uDaneCM4.nSlowa[n];
			sFlagiCM4 |= FMR_SA_DANE_CM4;	//ustaw flagę obecności nowych danych
			//__DSB();	//Data Synchronization Barrier. Acts as a special kind of Data Memory Barrier. It completes when all explicit memory accesses before this instruction complete.
		}
		HAL_HSEM_Release(HSEM_CM4_TO_CM7, HSEM_CM4);

		//Rdzeń CM7 pobiera dane z częstotliwością 100Hz (10ms).
		//Rdzeń CM4 produkuje dane co ok. 0,5ms, więc jeżeli nie będzie flagi od CM7, to 16 okresów wstaw nowe dane
		cLicznikOdswiezaniaCM4++;
		cLicznikOdswiezaniaCM4 &= 0x0F;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);		//kanał serw 1 skonfigurowany jako IO
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
      HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_CM7_TO_CM4));
      sFlagiCM7 |= FMR_SPRAWDZ_CM7;

}
