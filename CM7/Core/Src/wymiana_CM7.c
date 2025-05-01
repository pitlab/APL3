//////////////////////////////////////////////////////////////////////////////
//
// Międzyprocesorowa wymiana danych na rdzeniu CM7
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

/* Do wymiany danych używam bufora pamięci SRAM4 w domenie D3. Dostęp do pamięci może być 8, 16 lub 32 bitowy, więc
 * bufor wymiany deklaruję z jednej jako unię słów 32-bitowych z drugiej struktuje taką jaką będzie trzeba
 *
 * */

#include "sys_def_CM7.h"
#include "wymiana_CM7.h"
#include "errcode.h"

volatile uint32_t nBuforWymianyCM4[ROZMIAR_BUF32_WYMIANY_CM4]  __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
volatile uint32_t nBuforWymianyCM7[ROZMIAR_BUF32_WYMIANY_CM7]  __attribute__((section(".BuforyWymianyCM7CM4_SRAM4")));
char chBuforNapisowCM4[ROZMIAR_BUF_NAPISOW_CM4];
uint8_t chWskNapBufNapisowCM4 = 0;
uint8_t chWskOprBufNapisowCM4 = 0;
unia_wymianyCM4_t uDaneCM4;
unia_wymianyCM7_t uDaneCM7;



////////////////////////////////////////////////////////////////////////////////
// Pobiera dane z rdzenia CM4
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDaneWymiany_CM4(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef chErr = ERR_SEMAFOR_ZAJETY;

	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM4_TO_CM7);
	if (!nStanSemafora)
	{
		chErr = HAL_HSEM_Take(HSEM_CM4_TO_CM7, 0);
		if (chErr == ERR_OK)
		{
			for (uint8_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM4; n++)
			{
				uDaneCM4.nSlowa[n] = nBuforWymianyCM4[n];
			}

			//jeżeli było wysłane polecenie to sprawdź odpowiedź
			if (uDaneCM4.dane.chOdpowiedzNaPolecenie > ERR_OK)	//ERR_OK jest poleceniem neutralnym, ERR_DONE  - potwierdzenie wykonania, inna odpowiedź to kod błędu
			{
				//uDaneCM7.dane.chWykonajPolecenie = POL_NIC;	//po skutecznym wysłaniu polecenia ustaw polecenie wychodzące jako neutralne
			}

			//przepisz napis z CM4 do bufora napisów
			for (uint8_t n=0; n<ROZMIAR_BUF_NAPISU_WYMIANY; n++)
			{
				if (uDaneCM4.dane.chNapis[n] != 0)
				{
					chBuforNapisowCM4[chWskNapBufNapisowCM4] = uDaneCM4.dane.chNapis[n];
					chWskNapBufNapisowCM4++;
					if (chWskNapBufNapisowCM4 == ROZMIAR_BUF_NAPISOW_CM4)
						chWskNapBufNapisowCM4 = 0;
				}
				else
					break;
			}

			//wyzeruj pierwszy znak napisu w buforze wymiany żeby zaznaczyć że został odczytany
			uint32_t* nAdresNapisu = (uint32_t*)uDaneCM4.dane.chNapis;
			uint32_t* nAdresBufora =  (uint32_t*)&uDaneCM4.dane.fAkcel1[0];
			nBuforWymianyCM4[nAdresNapisu - nAdresBufora] = 0;
			HAL_HSEM_Release(HSEM_CM4_TO_CM7, 0);
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Ustaw dane z rdzenia CM7
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDaneWymiany_CM7(void)
{
	uint32_t nStanSemafora;
	HAL_StatusTypeDef chErr = ERR_SEMAFOR_ZAJETY;
	nStanSemafora = HAL_HSEM_IsSemTaken(HSEM_CM7_TO_CM4);
	if (!nStanSemafora)
	{
		chErr = HAL_HSEM_Take(HSEM_CM7_TO_CM4, 0);
		if (chErr == ERR_OK)
		{
			for (uint8_t n=0; n<ROZMIAR_BUF32_WYMIANY_CM7; n++)
			{
				nBuforWymianyCM7[n] = uDaneCM7.nSlowa[n];
			}

			HAL_HSEM_Release(HSEM_CM7_TO_CM4, 0);
		}
	}
	return chErr;
}
