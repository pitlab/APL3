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



////////////////////////////////////////////////////////////////////////////////
// Synchronizuje czas lokalny z GNSS
// Parametry: wskaźnik na strukturę danych z GNSS
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void SynchronizujCzasDoGNSS(stGnss_t *stGnss)
{
	//synchronizację robię tylko wraz z pojawieniem się nowego odczytu czasu
	if (stGnss->chSek != chPoprzedniaSekunda)
	{
		chPoprzedniaSekunda = stGnss->chSek;

		if ((chStanSynchronizacjiCzasu & SSC_CZAS_NIESYNCHR) && (stGnss->chGodz != 0) && (stGnss->chMin != 0))
		{
			sTime.Seconds = stGnss->chSek;
			sTime.Minutes = stGnss->chMin;
			sTime.Hours = stGnss->chGodz;
			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			chStanSynchronizacjiCzasu &= ~SSC_CZAS_NIESYNCHR;
		}

		if ((chStanSynchronizacjiCzasu & SSC_DATA_NIESYNCHR) && (stGnss->chRok != 0))
		{
			sDate.Date = stGnss->chDzien;
			sDate.Month = stGnss->chMies;
			sDate.Year = stGnss->chRok;
			HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			chStanSynchronizacjiCzasu &= ~SSC_DATA_NIESYNCHR;
		}
	}
}
