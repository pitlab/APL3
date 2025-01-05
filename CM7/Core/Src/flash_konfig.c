//////////////////////////////////////////////////////////////////////////////
//
// Obsługa pamięci konfiguracji w pamięci flash
//
// (c) PitLab 15 maj 2019
// http://www.pitlab.pl
///////////////////////////////////////////////////////////////////////////////*
#include "flash_konfig.h"
#include "sys_def_CM7.h"
#include "flash_nor.h"

/*
 * W sektorach 0 i 1 zewnętrznej pamięci Flash NOR przechowywane są paczki danych po 16 słów 16-bitowych co odpowiada stronie pamięci
 * Zawierają 1-bajtowy identyfikator paczki, 1-bajtową sumę kontrolną i 30 bajtów danych.
 * Jeżeli jedna ze zmiennych ulega zmianie, cała paczka jest przepisywanana koniec sektora. Aktualna jest ostatnia paczka w sektorze.
 * Jeżeli zostanie zapełniony cały sektor to wszystkie aktualne paczki są przepisywane do drugiego sektora
  */

uint32_t nAdresZapisuKonfigu;	//adres ostatnio zapisanej paczki danych


////////////////////////////////////////////////////////////////////////////////
// inicjalizuje stan emulatora. Ustala adres zapisu następnej paczki danych do Flash
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujKonfigFlash(void)
{
	uint8_t chIdentyfikatorPaczki;
	uint16_t n;

	nAdresZapisuKonfigu = ADRES_SEKTORA0;		//jeżeli nie znajdzie danych to ten adres będzie domyślny

	//Czyta oba sektory od końca aby stwierdzić gdzie jest koniec danych
	for (n=(ROZMIAR_SEKTORA / ROZMIAR_PACZKI_KONF)-1; n>0; n--)
	{
		chIdentyfikatorPaczki = *(__IO unsigned char*)(ADRES_SEKTORA0 + (n-1)*ROZMIAR_PACZKI_KONF);
		if (chIdentyfikatorPaczki != 0xFF)	//testowany jest pierwszy bajt paczki zawierajacy identyfikator
		{
			//sprawdź czy zgadza się suma kontrolna paczki
			//if (SprawdzPaczke(ADRES_SEKTORA0 + (n-1)*ROZMIAR_PACZKI_KONF))
			{
				nAdresZapisuKonfigu = ADRES_SEKTORA0 + n*ROZMIAR_PACZKI_KONF;
				break;
			}
		}
		chIdentyfikatorPaczki = *(__IO unsigned char*)(ADRES_SEKTORA1 + (n-1)*ROZMIAR_PACZKI_KONF);
		if (chIdentyfikatorPaczki != 0xFF)
		{
			//if (SprawdzPaczke(ADRES_SEKTORA1 + (n-1)*ROZMIAR_PACZKI_KONF))
			{
				nAdresZapisuKonfigu = ADRES_SEKTORA1 + n*ROZMIAR_PACZKI_KONF;
				break;
			}
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// sprawdza poprawność sumy kontrolnej paczki we Flash
// Parametry: nAdres - adres paczki w pamięci flash
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
unsigned char SprawdzPaczke(unsigned int nAdres)
{
	unsigned char n, chSuma;

	chSuma = *(__IO unsigned char*)nAdres;
	for (n=2; n<ROZMIAR_PACZKI_KONF8; n++)
		chSuma += *(__IO unsigned char*)(nAdres + n);

	n = *(__IO unsigned char*)(nAdres + 1);
	return (chSuma == n);
}



////////////////////////////////////////////////////////////////////////////////
// zapisuje 16-bajtową paczkę danych do pamięci Flash - wersja na zewnątrz z domyślnym adresem
// Parametry:
// *chDane - wskaźnik na strukturę danych do zaprogramowania
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszPaczkeKonfigu(uint8_t* chDane)
{
	uint32_t nRozmiar;
	uint8_t chErr;

	//sprawdź czy jest jeszcze miejsce w sektorze
	if (nAdresZapisuKonfigu > ADRES_SEKTORA1)
		nRozmiar = ADRES_SEKTORA1 - nAdresZapisuKonfigu + ROZMIAR8_SEKTORA;
	else
		nRozmiar = ADRES_SEKTORA0 - nAdresZapisuKonfigu + ROZMIAR8_SEKTORA;

	if (nRozmiar < ROZMIAR_PACZKI_KONF8)
		PrzepiszDane();

	chErr = ZapiszPaczkeAdr(nAdresZapisuKonfigu, chDane);
	nAdresZapisuKonfigu += ROZMIAR_PACZKI_KONF;
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// zapisuje paczkę z konfiguracją do zewnętrznej pamięci Flash pod jawnie podany adres
// Parametry:
// nAdres - adres do zapisu paczki konfiguracji
// *chDane - wskaźnik na strukturę danych do zaprogramowania
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszPaczkeAdr(uint32_t nAdres, uint8_t* chDane)
{
	uint8_t n, chSuma;

	//policz sumę kontrolną z danych
	chSuma = *chDane;	//załaduj pierwszy składnik sumy: ID
	for (n=2; n<ROZMIAR_PACZKI_KONF; n++)
		chSuma += *(chDane+n);
	*(chDane+1) = chSuma;		//zapisz sumę zaraz za ID

	return ZapiszDaneFlashNOR(nAdres, (uint16_t*)chDane, ROZMIAR_PACZKI_KONF16);	//rzutuj 8-bitową paczke na 16-bitową
}



////////////////////////////////////////////////////////////////////////////////
// odczytuje paczkę konfiguracji z pamięci Flash
// Parametry:
// *chDane - wskaźnik na strukturę danych paczki
// chIdent - identyfikator paczki
// Zwraca: ilość odczytanych danych
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajPaczkeKonfigu(uint8_t* chDane, uint8_t chIdent)
{
	uint8_t n, m, chID;
	//uint8_t m = 0; //liczba odczytanych danych
	uint32_t nAdresOdczytu;

	//określ adres początku sektora w którym są zapisane dane
	if (nAdresZapisuKonfigu > ADRES_SEKTORA1)
		nAdresOdczytu = ADRES_SEKTORA1;
	else
		nAdresOdczytu = ADRES_SEKTORA0;

	//określ ile paczek trzeba odczytać
	n = (nAdresZapisuKonfigu - nAdresOdczytu) / ROZMIAR_PACZKI_KONF16;
	//Czyta bieżący sektor od końca aby stwierdzić gdzie jest ostatnia poprawna paczka o tym identyfikatorze
	for (; n>0; n--)
	{
		chID = *(__IO unsigned char*)(nAdresOdczytu + (n-1)*ROZMIAR_PACZKI_KONF16);
		if (chID == chIdent)	//testowany jest pierwszy bajt paczki zawierajacy identyfikator
		{
			if (SprawdzPaczke(nAdresOdczytu + (n-1)*ROZMIAR_PACZKI_KONF16))
			{
				for (m=0; m<ROZMIAR_PACZKI_KONF8; m++)
					*(chDane++) = *(__IO unsigned char*)(m + nAdresOdczytu + (n-1)*ROZMIAR_PACZKI_KONF16);
				break;
			}
		}
	}
	return m;
}



////////////////////////////////////////////////////////////////////////////////
// przepisuje najświeższe dane z bieżącego sektora do drugiego
// Parametry: chPage - numer strony
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t PrzepiszDane(void)
{
	uint8_t chPaczka[ROZMIAR_PACZKI_KONF8];
	uint8_t n, m;
	unsigned int nNowyAdresZapisu, nAdresKasowanegoSektora;
	uint8_t Err;

	//oblicz nowy adres zapisu w drugim sektorze
	if (nAdresZapisuKonfigu >= ADRES_SEKTORA1 + ROZMIAR8_SEKTORA)
	{
		nNowyAdresZapisu = ADRES_SEKTORA0;
		nAdresKasowanegoSektora = ADRES_SEKTORA1;
	}
	else
	{
		nNowyAdresZapisu = ADRES_SEKTORA1;
		nAdresKasowanegoSektora = ADRES_SEKTORA0;
	}

	//szukaj wszystkich typów paczek i przepisz ich najnowszą wersję do drugiego sektora
	for (n=0; n<LICZBA_TYPOW_PACZEK; n++)
	{
		m = CzytajPaczkeKonfigu(chPaczka, n);
		if (m == ROZMIAR_PACZKI_KONF8)		//czy odczytało poprawną paczkę
		{
			ZapiszPaczkeAdr(nNowyAdresZapisu, chPaczka);
			nNowyAdresZapisu += ROZMIAR_PACZKI_KONF16;
		}
	}

	//skasuj zapełniony sektor
	Err = KasujSektorFlashNOR(nAdresKasowanegoSektora);

	nAdresZapisuKonfigu = nNowyAdresZapisu;
	return Err;
}



////////////////////////////////////////////////////////////////////////////////
// konwertuje 2-bajtową tablicę unsigned char na zmienną typu float
// Parametry: *chData - wskaźnik na tablicę zmiennych unsigned char
// Zwraca: zmienna float
////////////////////////////////////////////////////////////////////////////////
float KonwChar2Float(unsigned char *chData)
{
	unsigned char n;
    typedef union
    {
		unsigned char array[sizeof(float)];
		float fuData;
    } fUnion;
    fUnion temp;

    for (n=0; n<sizeof(float); n++)
    	temp.array[n] = *(chData++);

    if ((float)temp.fuData)
    	return temp.fuData;
    else
    	return (float)0.0;
}



////////////////////////////////////////////////////////////////////////////////
// konwertuje zmienną typu float na 4-bajtową tablicę unsigned char
// Parametry:
// [i] *fData - wskaźnik na dana do zapisu
// [i] *chData - wskaźnik na tablicę zmiennych unsigned char
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwFloat2Char(float fData, unsigned char *chData)
{
	unsigned char n;
    typedef union
    {
    	unsigned char array[sizeof(float)];
    	float fuData;
    } fUnion;
    fUnion temp;

    temp.fuData = fData;

    for (n=0; n<sizeof(float); n++)
    	*(chData++) = temp.array[n];
}



////////////////////////////////////////////////////////////////////////////////
// konwertuje 2-bajtową tablicę unsigned char na zmienną typu unsigned short
// Parametry: *chData - wskaźnik na tablicę zmiennych unsigned char
// Zwraca: zmienna unsignned short
////////////////////////////////////////////////////////////////////////////////
unsigned short KonwChar2UShort(unsigned char *chData)
{
	unsigned char n;
    typedef union
    {
    	unsigned char array[sizeof(unsigned short)];
    	unsigned short usData;
    } sUnion;
    sUnion temp;

    for (n=0; n<sizeof(unsigned short); n++)
    	temp.array[n] = *(chData++);

    if ((unsigned short)temp.usData)
    	return temp.usData;
    else
    	return (unsigned short)0;
}



////////////////////////////////////////////////////////////////////////////////
// konwertuje 2-bajtową tablicę unsigned char na zmienną typu signed short
// Parametry: *chData - wskaźnik na tablicę zmiennych unsigned char
// Zwraca: zmienna signned short
////////////////////////////////////////////////////////////////////////////////
signed short KonwChar2SShort(unsigned char *chData)
{
	unsigned char n;
    typedef union
    {
    	unsigned char array[sizeof(unsigned short)];
    	signed short ssData;
    } sUnion;
    sUnion temp;

    for (n=0; n<sizeof(signed short); n++)
    	temp.array[n] = *(chData++);

    if ((unsigned short)temp.ssData)
    	return temp.ssData;
    else
    	return (signed short)0;
}


////////////////////////////////////////////////////////////////////////////////
// konwertuje zmienną typu unsigned short na 2-bajtową tablicę unsigned char
// Parametry:
// [i] sData - dana do zapisu
// [i] *chData - wskaźnik na tablicę zmiennych unsigned char
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwUShort2Char(unsigned short sData, unsigned char *chData)
{
	unsigned char n;
    typedef union
    {
    	unsigned char array[sizeof(unsigned short)];
    	unsigned short usData;
	} sUnion;
	sUnion temp;

    temp.usData = sData;

    for (n=0; n<sizeof(unsigned short); n++)
    	*(chData++) = temp.array[n];
}


////////////////////////////////////////////////////////////////////////////////
// konwertuje zmienną typu signed short na 2-bajtową tablicę unsigned char
// Parametry:
// [i] sData - dana do zapisu
// [i] *chData - wskaźnik na tablicę zmiennych unsigned char
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwSShort2Char(signed short sData, unsigned char *chData)
{
	unsigned char n;
    typedef union
    {
    	unsigned char array[sizeof(unsigned short)];
    	signed short ssData;
	} sUnion;
	sUnion temp;

    temp.ssData = sData;

    for (n=0; n<sizeof(unsigned short); n++)
    	*(chData++) = temp.array[n];
}

