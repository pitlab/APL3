//////////////////////////////////////////////////////////////////////////////
//
// Obsługa odmierzania i synchronizacji czasu
//
// (c) PitLab 20245
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "czas.h"



extern RTC_HandleTypeDef hrtc;

//pole bitowe określające stan synchronizacji lokalnego zegara z GNSS. Wartość 1 gdy jest zsynchronizowana
uint8_t chStanSynchronizacjiCzasu = 0;
static uint8_t chPoprzedniaSekunda;
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
uint8_t chMinuta, chPoprzedniaMinuta;		//zmienne potrzebne do detekcji zmiany czasu przy jego wyświetlaniu
extern TIM_HandleTypeDef htim6;



////////////////////////////////////////////////////////////////////////////////
// Synchronizuje czas lokalny z GNSS
// Parametry: wskaźnik na strukturę danych z GNSS
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t SynchronizujCzasDoGNSS(stGnss_t *stGnss)
{
	uint8_t chErr = ERR_PROCES_TRWA;

	//synchronizację robię tylko wraz z pojawieniem się nowego odczytu czasu
	if (stGnss->chSek != chPoprzedniaSekunda)
	{
		chPoprzedniaSekunda = stGnss->chSek;
		if ((chStanSynchronizacjiCzasu & SSC_MASKA_CZASU) != (SSC_GODZ_SYNCHR + SSC_MIN_SYNCHR + SSC_SEK_SYNCHR))	//gdy brak pełnej synchronizacji czasu
		{
			//synchronizacja tylko w niezerowych sekundach lub gdy sekundy już wcześniej były zsynchronizowane
			if (((stGnss->chSek != 0) && (stGnss->chSek < 60)) || ((chStanSynchronizacjiCzasu & SSC_SEK_SYNCHR) == 0))
			{
				sTime.Seconds = stGnss->chSek;
				chStanSynchronizacjiCzasu |= SSC_SEK_SYNCHR;	//ustaw flagę synchronizacji
				if ((stGnss->chMin != 0) && (stGnss->chMin < 60))	//synchronizacja tylko w niezerowych minutach
				{
					sTime.Minutes = stGnss->chMin;
					chStanSynchronizacjiCzasu |= SSC_MIN_SYNCHR;	//ustaw flagę synchronizacji
					//godzina == 0 to zwykle brak poprawnego czasu, ale jeżeli minuta i sekundą są niezerowe to może być czas między północą a pierwszą
					if (stGnss->chGodz < 24)
					{
						sTime.Hours = stGnss->chGodz;
						chStanSynchronizacjiCzasu |= SSC_GODZ_SYNCHR;
						chErr = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);		//ustaw czas gdy wszystko poprawne
					}
				}
			}
		}

		if ((chStanSynchronizacjiCzasu & SSC_MASKA_DATY) != (SSC_ROK_SYNCHR + SSC_MIES_SYNCHR + SSC_DZIEN_SYNCHR))	//gdy brak pełnej synchronizacji daty
		{
			if ((stGnss->chDzien != 0) && (stGnss->chDzien <= 31))
			{
				sDate.Date = stGnss->chDzien;
				chStanSynchronizacjiCzasu |= SSC_DZIEN_SYNCHR;	//ustaw flagę synchronizacji
				if ((stGnss->chMies != 0) && (stGnss->chMies <= 12))
				{
					sDate.Month = stGnss->chMies;
					chStanSynchronizacjiCzasu |= SSC_MIES_SYNCHR;	//ustaw flagę synchronizacji
					if ((stGnss->chRok != 0) && (stGnss->chRok < 50))
					{
						sDate.Year = stGnss->chRok;
						chStanSynchronizacjiCzasu |= SSC_ROK_SYNCHR;
						chErr = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);		//ustaw datę gdy wszystko jest poprawne
					}
				}
			}
		}
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera datę i czas z RTC
// Parametry: wskaźnik na zmienne z datą i czasem
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t PobierzDateCzas(RTC_DateTypeDef *sData, RTC_TimeTypeDef *sCzas)
{
	uint8_t chErr;

	chErr  = HAL_RTC_GetTime(&hrtc, sCzas, RTC_FORMAT_BIN);
	chErr |= HAL_RTC_GetDate(&hrtc, sData, RTC_FORMAT_BIN);		// Uwaga: GetDate MUSI być po GetTime
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Pobiera stan licznika pracującego na 200MHz/200
// Parametry: brak
// Zwraca: stan licznika w mikrosekundach
////////////////////////////////////////////////////////////////////////////////
uint32_t PobierzCzasT6(void)
{
	extern volatile uint16_t sCzasH;
	return htim6.Instance->CNT + ((uint32_t)sCzasH <<16);
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu
// Parametry: nStart - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu startu
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas(uint32_t nPoczatek)
{
	uint32_t nCzas, nCzasAkt;

	nCzasAkt = PobierzCzasT6();
	if (nCzasAkt >= nPoczatek)
		nCzas = nCzasAkt - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nCzasAkt;
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu
// Parametry: nStart - licznik czasu na na początku pomiaru
// nKoniec - licznik czasu na na końcu pomiaru
// Zwraca: ilość czasu w mikrosekundach jaki upłynął do podanego czasu startu
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas2(uint32_t nPoczatek, uint32_t nKoniec)
{
	uint32_t nCzas;

	if (nKoniec >= nPoczatek)
		nCzas = nKoniec - nPoczatek;
	else
		nCzas = 0xFFFFFFFF - nPoczatek + nKoniec;
	return nCzas;
}



////////////////////////////////////////////////////////////////////////////////
// Czekaj z timeoutem dopóki zmienna ma wartość niezerową
// Parametry: chZajety - zmienna, która ma się ustawić na zero
// nCzasOczekiwania - czas przez jaki czekamy na wyzerowanie się zmiennej w mikrosekundach
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzekajNaZero(uint8_t chZajety, uint32_t nCzasOczekiwania)
{
	uint32_t nPoczatek, nCzas;

	nPoczatek = PobierzCzasT6();
	do
	{
		nCzas = MinalCzas(nPoczatek);
	}
	while(chZajety && (nCzas < nCzasOczekiwania));
	if (!chZajety)
		return BLAD_OK;

	return BLAD_TIMEOUT;
}


////////////////////////////////////////////////////////////////////////////////
// Dostarcza czas dla FATFS
// Parametry: brak
// Zwraca: czas
////////////////////////////////////////////////////////////////////////////////
DWORD PobierzCzasFAT(void)
{
    PobierzDateCzas(&sDate, &sTime);
    return ((DWORD)(sDate.Year + 20) << 25)    // Rok = 2000 + sDate.Year, FAT zaczyna od 1980
         | ((DWORD)sDate.Month << 21)
         | ((DWORD)sDate.Date << 16)
         | ((DWORD)sTime.Hours << 11)
         | ((DWORD)sTime.Minutes << 5)
         | ((DWORD)sTime.Seconds >> 1);        // FAT ma dokładność do 2 s
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna pomiar cykli maszynowych. Ważne narzędzie do optymalizacji procesów
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void StartPomiaruCykli(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // włącz DWT
	DWT->CYCCNT = 0;    //Processor clock cycle counter
	DWT->CPICNT = 0;	//Counts additional cycles required to execute multi-cycle instructions, except those recorded by DWT_LSUCNT, and counts any instruction fetch stalls.
	DWT->EXCCNT = 0;	//Counts the number of cycles spent in exception processing.
	DWT->SLEEPCNT = 0;	//Counts the number of cycles spent in sleep mode (WFI, WFE, sleep-on-exit).
	DWT->LSUCNT = 0;	//Counts additional cycles required to execute load and store instructions
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}



////////////////////////////////////////////////////////////////////////////////
// Zwraca wynik pomiaru liczby cykli maszynowych.
// Parametry: brak
// Zwraca: liczba cykli od czasu włączenia procesu
////////////////////////////////////////////////////////////////////////////////
uint32_t WynikPomiaruCykli(uint8_t *CPI, uint8_t *EXC, uint8_t *SLEEP, uint8_t *LSU)
{
	*CPI = DWT->CPICNT;
	*EXC = DWT->EXCCNT;
	*SLEEP = DWT->SLEEPCNT;
	*LSU = DWT->LSUCNT;
	CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;   // wyłącz DWT
	return DWT->CYCCNT;
}



////////////////////////////////////////////////////////////////////////////////
// Pisze na konsoli wynik pomiaru liczby cykli maszynowych.
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void WyswietlCykle(void)
{
	printf("Cykle: %ld (%ld us) multi: %d wyjatki: %d dodatk: %d sleep: %d\r\n", DWT->CYCCNT, DWT->CYCCNT / (HAL_RCC_GetSysClockFreq()/1000000), (uint8_t)DWT->CPICNT, (uint8_t)DWT->EXCCNT, (uint8_t)DWT->LSUCNT, (uint8_t)DWT->SLEEPCNT);
}

