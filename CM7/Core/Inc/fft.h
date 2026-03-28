/*
 * fft.h
 *
 *  Created on: 23 mar 2026
 *      Author: PitLab
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#include "sys_def_CM7.h"
#include "wymiana.h"

#define FFT_WYKLADNIK_MIN	6		//najmniejszy wykładnik FFT 2^6 = 64
#define FFT_WYKLADNIK_MAX	12		//największy wykładnik FFT 2^12 = 4096
#define FFT_MAX_ROZMIAR		4096	//największy rozmiar danych do liczenia FFT
#define LICZBA_TESTOW_FFT	100		//liczba warunków pracy dla których zdeto komplet FFT np, różnych wysterowań silników
#define LICZBA_ZMIENNYCH_FFT	6
#define LICZBA_WYKRESOW_FFT	3
//definicje  bajtu statusu
#define FFT_NOWE_DANE		1
#define FFT_GOTOWY_WYNIK	2


typedef struct
{
	float Re;
	float Im;
} stZesp_t;


typedef struct
{
	uint8_t chStatus;	//pole boitowe do testowania przepl=ływem danych wejściowych i wyjściowych
	uint8_t chRodzajOkna;		//indeks okna FFT
	uint8_t chIndeksZmiennejWe;
	uint8_t chWykladnikPotegi;	//wykładnik potęgi 2 okreslajacy rozmiar FFT
	uint16_t sLiczbaProbek;
	uint8_t chIndeksTestu;		//licznik testów dla kompletu FFT w różnych warunkach pracy np dla różnego wysterowania napędu. Max = LICZBA_TESTOW_FFT
	uint8_t chAktywnSilniki;	//maska bitowa określająca które silniki maja być aktywne w trakcie testu rezonansu ramy
	uint16_t sMaxWysterowanie;	//wartość maksymalnego wysterowania silników dla testu rezonansu
} stFFT_t;


void PobierzDaneDoFFT(void);
void InicjujFFT(void);
uint8_t ZapiszKonfiguracjejFFT(void);
void LiczFFT(stFFT_t *konfig, uint8_t chCzujnik);
void FFT_Motyl(stZesp_t *stWeWyP, stZesp_t *stWeWyN, stZesp_t *stWnk);
stZesp_t MulComplex(stZesp_t stWejscie1, stZesp_t stWejscie2);
stZesp_t AddComplex(stZesp_t stWejscie1, stZesp_t stWejscie2);
stZesp_t PowComplex(stZesp_t stPodstawa, float fWykładnik);
stZesp_t PowEComplex(stZesp_t stWejscie);

#endif /* INC_FFT_H_ */
