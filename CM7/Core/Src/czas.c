//////////////////////////////////////////////////////////////////////////////
//
// Obsługa odmierzania i synchronizacji czasu
//
// (c) PitLab 20245
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "czas.h"



extern RTC_HandleTypeDef hrtc;

//pole bitowe określające potrzebę i stan synchronizacji lokalnego zegara z GNSS. Wartość 0 gdy nie potrzeba synchronizacji
uint8_t chStanSynchronizacjiCzasu  = SSC_CZAS_NIESYNCHR | SSC_DATA_NIESYNCHR;
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

		if ((chStanSynchronizacjiCzasu & SSC_CZAS_NIESYNCHR) && (stGnss->chGodz != 0) && (stGnss->chMin != 0))
		{
			sTime.Seconds = stGnss->chSek;
			sTime.Minutes = stGnss->chMin;
			sTime.Hours = stGnss->chGodz;
			chErr = HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			chStanSynchronizacjiCzasu &= ~SSC_CZAS_NIESYNCHR;
		}

		if ((chStanSynchronizacjiCzasu & SSC_DATA_NIESYNCHR) && (stGnss->chRok != 0))
		{
			sDate.Date = stGnss->chDzien;
			sDate.Month = stGnss->chMies;
			sDate.Year = stGnss->chRok;
			chErr = HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			chStanSynchronizacjiCzasu &= ~SSC_DATA_NIESYNCHR;
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

