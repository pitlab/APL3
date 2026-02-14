/*
 * WeWyRC.h
 *
 *  Created on: Jun 15, 2025
 *      Author: PitLab
 */

#ifndef INC_ODBIORNIKRC_H_
#define INC_ODBIORNIKRC_H_

#include "sys_def_CM4.h"
#include "wymiana.h"


#define OKRES_RAMKI_PPM_RC	30000	//Czas między kolejnymi ramkami PPM odbiornika RC [us]
//#define ROZM_BUF_ODB_SBUS	25
//#define ROZMIAR_RAMKI_SBUS	25
//#define OKRES_RAMKI_SBUS	1300	//Czas między kolejnymi wychodzącymi ramkami SBus czas jest odmierzany w kwantach 5ms. Niepełne 3 kwanty są po to aby nie porzucał ramek jeżeli pętla wykona się odrobinę szybciej
//#define SBUS_NAGLOWEK		0x0F	//bajt nagłówkowy ramki SBus
//#define SBUS_STOPKA			0x00	//bajt stopki protokołu BSBus
//#define MAX_PRZESUN_NAGL	2		//nagłówek ramki S-Bus może być przesunięty maksymalnie o tyle bajtów do tyłu
//#define SBUS_MIN			380		//minimalna wartość jaką przenosi protokół SBus
//#define SBUS_NEU			1030	//wartość SBus w pozycji neutralnej
//#define SBUS_MAX			1680	//maksymalna wartość jaką przenosi protokół SBus

#define ZEGAR_PWM			2000000	//[Hz] aby osiągnać krok 500ns
#define IMPULS_PWM			1000
#define LICZBA_WEJSC_RC		2
#define LICZBA_WYJSC_RC		9
#define LICZBA_KONFIG_WYJSC_RC	5	//pierwsze 8 wyjść zajmują pół bajtu, ostatnie wyjście definiujące grupę 8..16 cały bajt



//znaczenia bitów zmiennej chStatus
#define STATRC_RAMKA1_OK		0x01
#define STATRC_RAMKA2_OK		0x02
#define STATRC_ZBIERAJ_EKSTREMA	0x04
#define STATRC_ZEBRANO_EKSTR1	0x08
#define STATRC_ZEBRANO_EKSTR2	0x10

typedef struct
{
	int16_t sOdb1[KANALY_ODB_RC];	//dane z odbiornika RC1
	int16_t sOdb2[KANALY_ODB_RC];	//dane z odbiornika RC2
	int16_t sMin1[KANALY_ODB_RC];	//esktrema: minimalne wartości kanałów odbiornika 1
	int16_t sMin2[KANALY_ODB_RC];	//esktrema: minimalne wartości kanałów odbiornika 2
	int16_t sMax1[KANALY_ODB_RC];	//esktrema: maksymalne wartości kanałów odbiornika 1
	int16_t sMax2[KANALY_ODB_RC];	//esktrema: maksymalne wartości kanałów odbiornika 2
	uint8_t chStatus;				//określaja stan odbiorników
	uint8_t chNrKan1;						//wskaźnik numeru kanału odbiornika 1
	uint8_t chNrKan2;						//wskaźnik numeru kanału odbiornika 2
	uint32_t nCzasWe1;						//czas początku dekodowania  ostatniego pakietu danych z odbiornika 1
	uint32_t nCzasWe2;						//czas początku dekodowania  ostatniego pakietu danych z odbiornika 2
	uint16_t sZdekodowaneKanaly1;			//bity zdekodowanych kanałów odbiornika 1
	uint16_t sZdekodowaneKanaly2;			//bity zdekodowanych kanałów odbiornika 2
	volatile uint16_t sPoprzedniaWartoscTimera1;	//zapamiętuje wartość timera wejścia odbiornika 1 dla poprzedniego zbocza aby z tego policzyć czas
	volatile uint16_t sPoprzedniaWartoscTimera2;	//zapamiętuje wartość timera wejścia odbiornika 2 dla poprzedniego zbocza aby z tego policzyć czas
} stRC_t;


//deklaracja funkcji
uint8_t InicjujWejsciaRC(void);	//odbiorniki
uint8_t InicjujWyjsciaRC(void);	//serwa, ESC
uint8_t AktualizujWyjsciaRC(stWymianyCM4_t *dane);
uint8_t DywersyfikacjaOdbiornikowRC(stRC_t *stRC, stWymianyCM4_t *psDaneCM4, stWymianyCM7_t *psDaneCM7);
//uint8_t DekodowanieRamkiBSBus(uint8_t* chRamkaWe, int16_t *sKanaly);
//uint8_t FormowanieRamkiSBus(uint8_t *chRamkaSBus, uint8_t *chWskNapRamki, uint8_t *chBuforAnalizy, uint8_t chWskNapBuf, uint8_t *chWskOprBuf);
//uint8_t ObslugaRamkiSBus(void);
void RozpocznijZbieranieEkstremowWejscRC(void);
void ZapiszEkstremaWejscRC(void);
void ZbierajEkstremaWejscRC(stRC_t *stRC);
void NormalizujWejsciaRC(uint16_t sWejscie, uint16_t sMin, uint16_t sMax, uint16_t *sWyjscie);
uint8_t AnalizujSygnalRC(stWymianyCM4_t* psDaneCM4);
uint32_t PobierzFunkcjeWyjsciaRC(uint8_t chNrWyjscia, uint8_t *chFunkcja, stWymianyCM4_t *daneCM4);

#endif /* INC_ODBIORNIKRC_H_ */
