//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi odbiornika RC Crossfire na UART8
//
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Crossfire.h"


extern UART_HandleTypeDef huart8;
extern volatile uint8_t chWskNapBaGNSS, chWskOprBaGNSS;		//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych GNSS


////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę odbiornika RC w gnieździe GNSS (z powodu braku innego)
// Parametry: brak
// Zwraca: kod zakończenia inicjalizacji: ERR_DONE = zakończono, BLAD_OK - w trakcie
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujCrossfire(void)
{
	uint8_t cBłąd = BLAD_OK;

	chWskNapBaGNSS = chWskOprBaGNSS = 0;	//inicjuj wskaźniki napełniania i opróżniania buforma kołowego analizy danych z UART8
	huart8.Init.BaudRate = 416666;
	cBłąd = UART_SetConfig(&huart8);

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Analizuje dane odebrane z odbiornika, dekoduje je i wstawia do struktury danych
// Parametry:
// [wy] *psDaneCM4 - wskaźnik na strukturę danych CM4
// [wy] *psDaneCM7 - wskaźnik na strukturę danych CM7
// Zwraca: kod zakończenia inicjalizacji: ERR_DONE = zakończono, BLAD_OK - w trakcie
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujSygnalCrossfire(stWymianyCM4_t *psDaneCM4, stWymianyCM7_t *psDaneCM7)
{
	uint8_t cBłąd = BLAD_OK;


	chWskNapBaGNSS


	return cBłąd;
}
