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


#define LICZBA_WYKRESOW_FFT	3
//definicje  bajtu statusu
#define FFT_NOWE_DANE		1
#define FFT_GOTOWY_WYNIK	2

//definicje bitów zmiennych FFT
#define AKCEL_X		0x01
#define AKCEL_Y		0x02
#define AKCEL_Z		0x04
#define ZYRO_X		0x08
#define ZYRO_Y		0x10
#define ZYRO_Z		0x20

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
	uint8_t chMaxWysterowanie;	//procentowa wartość maksymalnego wysterowania silników dla testu rezonansu drgań
} stFFT_t;


void PobierzDaneDoFFT(void);
uint8_t PobierzWynikiFFT(uint8_t *chBufWyniku, uint8_t chBityZmiennej, uint8_t *chIndeksTestu, uint16_t *sIndeksFFT);
void InicjujFFT(void);
uint8_t ZapiszKonfiguracjejFFT(void);
void LiczFFT(stFFT_t *konfig, uint8_t chCzujnik);
void FFT_Motyl(stZesp_t *stWeWyP, stZesp_t *stWeWyN, stZesp_t *stWnk);
stZesp_t MulComplex(stZesp_t stWejscie1, stZesp_t stWejscie2);
stZesp_t AddComplex(stZesp_t stWejscie1, stZesp_t stWejscie2);
stZesp_t PowComplex(stZesp_t stPodstawa, float fWykładnik);
stZesp_t PowEComplex(stZesp_t stWejscie);

#endif /* INC_FFT_H_ */
