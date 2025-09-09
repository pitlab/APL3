//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi sterownika ekranu dotykowego na układzie ADS7846 lub XPT2046
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "dotyk.h"
#include <rysuj.h>
#include <ili9488.h>
#include <RPi35B_480x320.h>
#include "moduly_SPI.h"
#include "errcode.h"
#include "LCD.h"
#include "display.h"
#include "sys_def_CM7.h"
#include <stdio.h>
#include <string.h>
#include "flash_konfig.h"
#include "semafory.h"
#include "czas.h"
#include "napisy.h"

//deklaracje zmiennych
extern SPI_HandleTypeDef hspi5;
uint8_t chEtapKalibr = 0;	//etap kalibracji kolejnego punktu
struct _statusDotyku statusDotyku;
struct _kalibracjaDotyku kalibDotyku;
uint16_t sXd[PKT_KAL];	//surowy pomiar panelu rezystancyjnego dla danego punktu kalibracyjnego X
uint16_t sYd[PKT_KAL];	//surowy pomiar panelu rezystancyjnego dla danego punktu kalibracyjnego Y
uint16_t sXe[PKT_KAL];	//współrzędne ekranowe dla danego punktu kalibracyjnego X
uint16_t sYe[PKT_KAL];	//współrzędne ekranowe dla danego punktu kalibracyjnego Z
uint8_t chLicznikDotkniec;
//deklaracje zmiennych
extern uint8_t MidFont[];
extern char chNapis[50];
extern const char *chNapisLcd[];

////////////////////////////////////////////////////////////////////////////////
// Czyta dane ze sterownika ekranu dotykowego
// Parametry: chChannel - kod kanału X, Y, Z1, Z2
// Zwraca: wartość rezystancji
////////////////////////////////////////////////////////////////////////////////
uint16_t CzytajKanalDotyku(uint8_t  chKanal)
{
	uint8_t chKonfig, chDane[2];
	uint16_t sDotyk;


	chKonfig = (3<<0)|	//PD1-PD0 Power-Down Mode Select Bits: 0=Power-Down Between Conversions, 1=Reference is off and ADC is on, 2=Reference is on and ADC is off, 3=Device is always powered.
			   (0<<2)|	//SER/DFR 1=Single-Ended 0=Differential Reference Select Bit
			   (0<<3)|	//MODE 12-Bit/8-Bit Conversion Select Bit. This bit controls the number of bits for the next conversion: 0=12-bits, 1=8-bits
			   (chKanal<<4)|	//A2-A0 Channel Select Bits
			   (1<<7);	//S Start Bit.

	UstawDekoderZewn(CS_TP);											//TP_CS=0
	HAL_SPI_Transmit(&hspi5, &chKonfig, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi5, chDane, 2, HAL_MAX_DELAY);
	UstawDekoderZewn(CS_NIC);							//TP_CS=1

	//złóż liczbę 12-bitową i wyrównaj do lewej
	sDotyk = chDane[0];
	sDotyk <<= 8;
	sDotyk |= chDane[1];
	sDotyk >>= 3;
	return sDotyk;
}



////////////////////////////////////////////////////////////////////////////////
// Czytaj komplet danych ze sterownika ekranu dotykowego
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajDotyk(void)
{
	uint32_t nCzasDotyku;
	uint32_t nStanSemaforaSPI;
	uint32_t nZastanaKonfiguracja_SPI_CFG1;

	//sprawdź czy upłyneło wystarczająco czasu od ostatniego odczytu
	nCzasDotyku = MinalCzas(statusDotyku.nOstCzasPomiaru);
	if (nCzasDotyku < 50000)	//50ms -> 20Hz
		return BLAD_OK;
	statusDotyku.nOstCzasPomiaru = PobierzCzasT6();

	//użyj sprzętowego semafora HSEM_SPI5_WYSW do określenia dostępu do SPI5
	nStanSemaforaSPI = HAL_HSEM_IsSemTaken(HSEM_SPI5_WYSW);
	if (!nStanSemaforaSPI)
	{
		if (HAL_HSEM_Take(HSEM_SPI5_WYSW, 0) == BLAD_OK)
		{
				//Ponieważ zegar SPI = 100MHz a układ może pracować z prędkością max 2,5MHz a jest na tej samej magistrali co TFT przy każdym odczytcie przestaw dzielnik zegara z 4 na 64
				nZastanaKonfiguracja_SPI_CFG1 = hspi5.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
				hspi5.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;	//maska preskalera
				hspi5.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_64;	//Bits 30:28 MBR[2:0]: master baud rate: 011: SPI master clock/64

				//domyslnie używam innej orientacji, więc zaimeń osie X i Y
				statusDotyku.sAdc[0] = CzytajKanalDotyku(TPCHY);	//najlepiej się zmienia dla osi X.
#ifdef LCD_ILI9488 			//wyświetlacz z Aliexpress ma odwrócony kierunek w osi Y
				statusDotyku.sAdc[1] = CzytajKanalDotyku(TPCHX);	//najlepiej się zmienia dla osi Y
#else
				statusDotyku.sAdc[1] = 4096 - CzytajKanalDotyku(TPCHX);	//Ponieważ wartość maleje dla większych Y, więc używam odwrotności
#endif
				statusDotyku.sAdc[2] = CzytajKanalDotyku(TPCHZ1);
				statusDotyku.sAdc[3] = CzytajKanalDotyku(TPCHZ2);
				HAL_HSEM_Release(HSEM_SPI5_WYSW, 0);
				hspi5.Instance->CFG1 = nZastanaKonfiguracja_SPI_CFG1;	//przywróc poprzednie nastawy
		}
	}
	else
		return ERR_ZAJETY_SEMAFOR;

	if (statusDotyku.sAdc[2] > MIN_Z)	//czy siła nacisku jest wystarczająca
	{
		if (statusDotyku.sAdc[2] == 0x1FFF) //czy jest to brak sterownika dotyku
			return ERR_BRAK_WYSWIETLACZA;

		if (statusDotyku.chCzasDotkniecia)
		{
			statusDotyku.chCzasDotkniecia--;
			if (!statusDotyku.chCzasDotkniecia)
			{
				statusDotyku.chFlagi |= DOTYK_DOTKNIETO;

				if (statusDotyku.chFlagi & DOTYK_SKALIBROWANY)
				{
					//oblicz współrzędne ekranowe
					float fTemp;
					fTemp = kalibDotyku.fAx * statusDotyku.sAdc[0] + kalibDotyku.fBx * statusDotyku.sAdc[1] + kalibDotyku.fDeltaX;
					statusDotyku.sX = (uint16_t)(fTemp + 0.5f);	//uwzględnij zaokrąglenie
					fTemp = kalibDotyku.fAy * statusDotyku.sAdc[0] + kalibDotyku.fBy * statusDotyku.sAdc[1] + kalibDotyku.fDeltaY;
					statusDotyku.sY = (uint16_t)(fTemp + 0.5f);
				}
			}
		}
	}
	else
	{
		statusDotyku.chCzasDotkniecia = MIN_T;	//jeżeli jest puszczenie to przeładuj czas dotknięcia
		statusDotyku.chFlagi |= DOTYK_ZWOLNONO;
	}

	/*/oblicz siłę dotyku - coś jest skopane, wskazania są odwrotnie proporcjonalne do siły nacisku
	if (sDotykAdc[2])
		sDotykAdc[4] = (200 * sDotykAdc[0]/4096) * ((sDotykAdc[3]/sDotykAdc[2])-1);
	else
		sDotykAdc[4] = 0; */

	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Kalibruje ekran rezystancyjny. Wyświetla punkty do naciśnięcia i zbiera pomiary z układu dotykowego
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujDotyk(void)
{
	uint8_t chErr = BLAD_OK;	//domyślny kod błedu;
	uint16_t sKolor;

	switch(chEtapKalibr)
	{
	case 0:	//inicjalizacja zmiennych
		//LCD_Orient(POZIOMO);
		WypelnijEkran(BLACK);
		//TestObliczenKalibracji();		//testowo sprawdzenie poprawności obliczeń
		setColor(WHITE);
		UstawCzcionke(MidFont);
		/*sprintf(chNapis, "Dotknij krzyzyk aby");
		RysujNapis(chNapis, CENTER, 60);
		sprintf(chNapis, "skalibrowac ekran dotykowy");
		RysujNapis(chNapis, CENTER, 80);*/

		//RysujNapiswRamce((char*)chNapisLcd[STR_DOTKNIJ_ABY_SKALIBROWAC], 0, 60, DISP_X_SIZE, 20);
		RysujNapis((char*)chNapisLcd[STR_DOTKNIJ_ABY_SKALIBROWAC], CENTER, 70);
		statusDotyku.chFlagi = 0;
		chEtapKalibr++;
		break;

	case 1:	//wyświetl krzyżyk w prawym górnym rogu
		sXe[0] = DISP_X_SIZE - BRZEG;
		sYe[0] = BRZEG;
		break;

	case 2:	//wyświetl krzyżyk w lewym górnym rogu
		sXe[1] = BRZEG;
		sYe[1] = BRZEG;
		break;

	case 3:	//wyświetl krzyżyk w lewym dolnym rogu
		sXe[2] = BRZEG;
		sYe[2] = DISP_Y_SIZE - BRZEG;
		break;

	case 4:	//wyświetl krzyżyk w prawym dolnym rogu
		sXe[3] = DISP_X_SIZE - BRZEG;
		sYe[3] = DISP_Y_SIZE - BRZEG;
		break;

	case 5:	//wyświetl krzyżyk na środku
		sXe[4] = DISP_X_SIZE/2;
		sYe[4] = DISP_Y_SIZE/2;
		break;
	}

	//czekaj na naciśnięcie
	if (statusDotyku.sAdc[2] > MIN_Z)		//czy siła nacisku jest wystarczająca
	{
		//if ((statusDotyku.chFlagi & DOTYK_DOTKNIETO) && (!(statusDotyku.chFlagi & DOTYK_ZAPISANO)))
		if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)
		{
			sXd[chEtapKalibr-1] = statusDotyku.sAdc[0];
			sYd[chEtapKalibr-1] = statusDotyku.sAdc[1];
			setColor(BLACK);	//ustaw czarny kolor aby zamazać stary krzyżyk
			statusDotyku.sAdc[2] = 0;
			//
		}
	}
	else	//ekran nie dotknięty
	{
		if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)		//jeżeli był dotknięty
		{
			setColor(WHITE);	//ponownie rysuj krzyzyk na biało
			chEtapKalibr++;
			if (chEtapKalibr == PKT_KAL+1)
			{
				ObliczKalibracjeDotyku3Punktowa();		//OK
				//ObliczKalibracjeDotykuWielopunktowa();	//Źle
				statusDotyku.chFlagi |= DOTYK_SKALIBROWANY;

				uint8_t chDane[ROZMIAR_PACZKI_KONFIG-2];

				for (uint8_t n=0; n<ROZMIAR_PACZKI_KONFIG-2; n++)
					chDane[n] = 0;

				KonwFloat2Char(kalibDotyku.fAx, &chDane[0]);
				KonwFloat2Char(kalibDotyku.fAy, &chDane[4]);
				KonwFloat2Char(kalibDotyku.fDeltaX, &chDane[8]);

				KonwFloat2Char(kalibDotyku.fBx, &chDane[12]);
				KonwFloat2Char(kalibDotyku.fBy, &chDane[16]);
				KonwFloat2Char(kalibDotyku.fDeltaY, &chDane[20]);
				chErr = ZapiszPaczkeKonfigu(FKON_KALIBRACJA_DOTYKU, chDane);
				if (chErr == BLAD_OK)
					statusDotyku.chFlagi |= DOTYK_ZAPISANO;
				chEtapKalibr = 0;
				WypelnijEkran(BLACK);
				return ERR_GOTOWE;	//kod wyjścia z procedury kalibracji
			}
			statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
		}
	}

	//rysuj krzyżyk kalibracyjny dla etapów większych niż 0
	if (chEtapKalibr)
	{
		RysujLiniePozioma(sXe[chEtapKalibr-1] - KRZYZ/2, sYe[chEtapKalibr-1], KRZYZ);
		RysujLiniePionowa(sXe[chEtapKalibr-1], sYe[chEtapKalibr-1] - KRZYZ/2, KRZYZ);
	}

	sKolor = getColor();	//zapamiętaj kolor

	setColor(YELLOW);
	sprintf(chNapis, "Dotyk ADC:");
	RysujNapis(chNapis, 80, 140);
	sprintf(chNapis, "X = %d   ", statusDotyku.sAdc[0]);
	RysujNapis(chNapis, 80, 160);
	sprintf(chNapis, "Y = %d   ", statusDotyku.sAdc[1]);
	RysujNapis(chNapis, 80, 180);
	sprintf(chNapis, "Z = %d   ", statusDotyku.sAdc[2]);
	RysujNapis(chNapis, 80, 200);
	setColor(sKolor);	//przywróć kolor
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kalibracja ekranu dotykowego na podstawie dokumentu: Calibration in touch-screen systems https://www.ti.com/lit/pdf/slyt277
// Xe1, Xe2, Xe3 oraz Ye1, Ye2, Ye3 - współrzędne X i Y punktów kalibracyjnych wyświetlanych na ekranie
// Xd1, Xd2, Xd3 oraz Yd1, Yd2, Yd3 - wartości X i Y odczytane z panelu dotykowego
// ax, ay, bx, by - wspólczynniki kalibracyjne równania uwzględniającego skalowania i rotację
// deltaX, deltaY - przesunięcie między punktami
//     |Xd1  Yd1  1|
// A = |Xd2  Yd2  1|  macierz A
//     |Xd3  Yd3  1|
//
// |Xe1|       |ax    |
// |Xe2| = A x |bx    |
// |Xe3|       |deltaX|
//
// |Ye1|       |ay    |
// |Ye2| = A x |by    |
// |Ye3|       |deltaY|
//
// n - liczba punktów kalibracji
// Parametry: nic
// Zwraca: nic
/////////////////////////
void ObliczKalibracjeDotykuWielopunktowa(void)
{
	uint8_t n;
	uint32_t nX1, nX2, nX3;
	uint32_t nY1, nY2, nY3;
	uint32_t a, b, c, d, e;
	float fDeltaX1, fDeltaX2, fDeltaX3;
	float fDeltaY1, fDeltaY2, fDeltaY3;
	float fWyznacznik;

	// X1 = suma od k=1 do n Xd[k] * Xe[k]
	// X2 = suma od k=1 do n Yd[k] * Xe[k]
	// X3 = suma od k=1 do n Xe[k]
	// Y1 = suma od k=1 do n Xd[k] * Ye[k]
	// Y2 = suma od k=1 do n Yd[k] * Ye[k]
	// Y3 = suma od k=1 do n Ye[k]
	nX1 =  nX2 = nX3 = 0;
	for (n=0; n<PKT_KAL; n++)
	{
		nX1 += sXd[n] * sXe[n];
		nX2 += sYd[n] * sXe[n];
		nX3 += sXe[n];
		nY1 += sXd[n] * sYe[n];
		nY2 += sYd[n] * sYe[n];
		nY3 += sYe[n];
	}

	// a = suma od k=1 do n Xd[k]^2
	// b = suma od k=1 do n Yd[k]^2
	// c = suma od k=1 do n Xd[k] * Yd[k]
	// d = suma od k=1 do n Xd[k]
	// e = suma od k=1 do n Yd[k]
	a = b = c = d = e = 0;
	for (n=0; n<PKT_KAL; n++)
	{
		a += sXd[n] * sXd[n];
		b += sYd[n] * sYd[n];
		c += sXd[n] * sYd[n];
		d += sXd[n];
		e += sYd[n];
	}

	// deltaX1 = n * (X1 * b - X2 * c) + e * (X2 * d - X1 * e) + X3 * (c * e - b * d)
	fDeltaX1 = PKT_KAL * (nX1 * b - nX2 * c) + e * (nX2 * d - nX1 * e) + nX3 * (c * e - b * d);

	// deltaX2 = n * (X2 * a - X1 * c) + d * (X1 * e - X2 * d) + X3 * (c * d - a * e)
	fDeltaX2 = PKT_KAL * (nX2 * a - nX1 * c) + d * (nX1 * e - nX2 * d) + nX3 * (c * d + a * e);

	// deltaX3 = X3 * (a * b - c^2) + X1 * (c * e - b * d) + X2 * (c * d - a * e)
	fDeltaX3 = nX3 * (a * b - c * c) + nX1 + (c * e - b * d) + nX2 * (c * d - a * e);

	// deltaY1 = n * (Y1 * b - Y2 * c) + e * (Y2 * d - Y1 * e) + Y3 * (c * e - b * d)
	fDeltaY1 = PKT_KAL * (nY1 * b - nY2 * c) + e * (nY2 * d - nY1 * e) + nY3 + (c * e - b * d);

	// deltaY2 = n * (Y2 * a - Y1 * c) + d * (Y1 * e - Y2 * d) + Y3 * (c * d - a * e)
	fDeltaY2 = PKT_KAL * (nY2 * a - nY1 * c) + d * (nY1 * e - nY2 * d) + nY3 * (c * d - a * e);

	// deltaY3 = Y3 * (a * b - c^2) + Y1 * (c * e - b * d) + Y2 * (c * d - a * e)
	fDeltaY3 = nY3 * (a * b * c * c) + nY1 + (c * e - b * d) + nY2 * (c * d - a * e);

	// delta = n * (a * b - c^2) + 2 * c * d * e - a * e^2 - b * d^2  (wyznacznik macierzy A)
	fWyznacznik =  PKT_KAL * (a * b - c * c) + 2 * c * d * e - a * e * e - b * d * d;

	//oblicz iz zapisz finalne współczynniki kalibracji
	kalibDotyku.fAx = fDeltaX1 / fWyznacznik;		// ax = deltaX1 / delta
	kalibDotyku.fBx = fDeltaX2 / fWyznacznik;		// bx = deltaX2 / delta
	kalibDotyku.fDeltaX = fDeltaX3 / fWyznacznik;	// deltaX = deltaX3 / delta
	kalibDotyku.fAy = fDeltaY1 / fWyznacznik;		// ay = deltaY1 / delta
	kalibDotyku.fBy = fDeltaY2 / fWyznacznik;		// by = deltaY2 / delta
	kalibDotyku.fDeltaY = fDeltaY3 / fWyznacznik;	// deltaY = deltaY3 / delta
}



////////////////////////////////////////////////////////////////////////////////
// Kalibracja ekranu dotykowego na podstawie dokumentu: Calibration in touch-screen systems https://www.ti.com/lit/pdf/slyt277
// kalibracja dla tylko 3 punktów. Przetestowana, działa poprawnie
// Genialny kalkulator macierzy do testów: https://matrixcalc.org/pl/#%7B%7B678,2169,1%7D,%7B2807,1327,1%7D,%7B2629,3367,1%7D%7D%5E%28-1%29
void ObliczKalibracjeDotyku3Punktowa(void)
{
	float fWyznacznik;
	int32_t a11, a12, a13, a21, a22, a23, a31, a32, a33;
	//     |Xd1  Yd1  1|
	// A = |Xd2  Yd2  1|  macierz A
	//     |Xd3  Yd3  1|
	//
	// gdzie A^-1 oznacza macierz odwrotną

	//Aby ją wyznaczyć najpierw liczę wyznacznik macierzy metodą Sarrusa (https://www.naukowiec.org/wzory/matematyka/wyznacznik-macierzy-3x3--metoda-sarrusa_618.html):
	//          |a11  a12  a13|
	// det(A) = |a21  a22  a23|  = a11 a22 a33 + a12 a23 a31 + a13 a21 a32 − (a13 a22 a31 + a11 a23 a32 + a12 a21 a33)
	//          |a31  a32  a33|
	//
	fWyznacznik = sXd[0] * sYd[1] * 1 + sYd[0] * 1 * sXd[2] + 1 * sXd[1] * sYd[2] - (1 * sYd[1] * sXd[2] + sXd[0] * 1 * sYd[2] + sYd[0] * sXd[1] * 1);	//OK

	//Aby wyznaczyć macierz odwrotną korzystam z metody dopełnień algebraicznych: https://www.naukowiec.org/wiedza/matematyka/macierz-odwrotna_607.html
	a11 = sYd[1] - sYd[2];							// a11 = +1 *(a22 * a33 - a23 * a32)
	a12 = -1 * (sXd[1] - sXd[2]);					// a12 = -1 * (a21 * a33 - a23 * a31)
	a13 = sXd[1] * sYd[2] - sYd[1] * sXd[2];		// a13 = +1 * (a21 * a32 - a22 * a31)
	a21 = -1 * (sYd[0] - sYd[2]);					// a21 = -1 * (a12 * a33 - a13 * a32)
	a22 = sXd[0] - sXd[2];							// a22 = +1 * (a11 * a33 - a13 * a31)
	a23 = -1 * (sXd[0] * sYd[2] - sYd[0] * sXd[2]);	// a23 = -1 * (a11 * a32 - a12 * a31)	- tutaj jest błąd na naukowiec.org
	a31 = sYd[0] - sYd[1];							// a31 = +1 * (a12 * a23 - a13 * a22)
	a32 = -1 * (sXd[0] - sXd[1]);					// a32 = -1 * (a11 * a23 - a13 * a21)
	a33 = sXd[0] * sYd[1] - sYd[0] * sXd[1];		// a33 = +1 * (a11 * a22 - a12 * a21)

	//teraz trzeba transponować macierz, czyli zamienić wiersze z kolumnami, ma to wygladać tak
	//       |a11 a21 a31|
	// A^T = |a12 a22 a32|
	//       |a13 a23 a33|

	//Finalnym krokiem uzyskania macierzy odwrotnej jest podzielenie transponowanej macierzy odwrotnej przez wyznacznik a nastepnie pomnożenie przez wektor współrzędnych ekranowych
	// |ax|              |sXe1|
	// |bx|     = A^-1 x |sXe2|
	// |deltaX|          |sXe3|
	//
	// |ay|              |sYe1|
	// |by|     = A^-1 x |sYe2|
	// |deltay|          |sYe3|

	kalibDotyku.fAx = (a11 * sXe[0] + a21 * sXe[1] + a31 * sXe[2]) / fWyznacznik;
	kalibDotyku.fBx = (a12 * sXe[0] + a22 * sXe[1] + a32 * sXe[2]) / fWyznacznik;
	kalibDotyku.fDeltaX = (a13 * sXe[0] + a23 * sXe[1] + a33 * sXe[2]) / fWyznacznik;

	kalibDotyku.fAy = (a11 * sYe[0] + a21 * sYe[1] + a31 * sYe[2]) / fWyznacznik;
	kalibDotyku.fBy = (a12 * sYe[0] + a22 * sYe[1] + a32 * sYe[2]) / fWyznacznik;
	kalibDotyku.fDeltaY = (a13 * sYe[0] + a23 * sYe[1] + a33 * sYe[2]) / fWyznacznik;
}



////////////////////////////////////////////////////////////////////////////////
// Przykład obliczeń z 3 strony dokumentu slyt277.pdf
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t TestObliczenKalibracji(void)
{
	sXe[0] = 64;
	sYe[0] = 384;
	sXd[0] = 678;
	sYd[0] = 2169;

	sXe[1] = 192;
	sYe[1] = 192;
	sXd[1] = 2807;
	sYd[1] = 1327;

	sXe[2] = 192;
	sYe[2] = 576;
	sXd[2] = 2629;
	sYd[2] = 3367;

	//ObliczKalibracjeDotykuWielopunktowa();	//wszystko jest złe
	ObliczKalibracjeDotyku3Punktowa();			//zwraca dobre wartości

	if ((kalibDotyku.fAx == 0.0623) && (kalibDotyku.fBx == 0.0054) && (kalibDotyku.fDeltaX == 9.9951) && (kalibDotyku.fAy == -0.0163) && (kalibDotyku.fBy == 0.01868) && (kalibDotyku.fDeltaY == -10.1458))
		return BLAD_OK;
	else
		return ERR_ZLE_DANE;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje linie na ekranie
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t TestDotyku(void)
{
	extern uint8_t chRysujRaz;
	uint16_t sKolor = getColor();	//zapamiętaj kolor

	if (chRysujRaz)
	{
		chRysujRaz = 0;
		setColor(GRAY40);
		sprintf(chNapis, "Test kalibracji");
		RysujNapis(chNapis, CENTER, 60);
		setColor(GRAY50);
		sprintf(chNapis, "Nacisnij ekran 6 razy aby zakonczyc");
		RysujNapis(chNapis, CENTER, 80);
	}

	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)		//jeżeli był dotknięty
	{
		setColor(WHITE);
		RysujLiniePozioma(statusDotyku.sX - KRZYZ/2, statusDotyku.sY, KRZYZ);
		RysujLiniePionowa(statusDotyku.sX, statusDotyku.sY - KRZYZ/2, KRZYZ);


		//setColor(RED);
		setColor(YELLOW);
		sprintf(chNapis, "Dotyk @ (%d, %d) = %d ", statusDotyku.sX, statusDotyku.sY, statusDotyku.sAdc[2]);
		RysujNapis(chNapis, 80, 140);

		setColor(sKolor);	//przywróć kolor
		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
		if (chLicznikDotkniec++ == 6)		//gdy licznik dotknięć doliczy do zadanej wartosci wtedy zakończ test
		{
			WypelnijEkran(BLACK);
			chLicznikDotkniec = 0;
			return ERR_GOTOWE;
		}
	}
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytaj współczynniki kalibracji ekranu z EEPROM
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujDotyk(void)
{
	uint8_t n, chPaczka[ROZMIAR_PACZKI_KONFIG];
	uint8_t chErr = ERR_BRAK_KONFIG;
	extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu

	for (n=0; n<4; n++)
		statusDotyku.sAdc[n] = 0;
	statusDotyku.chFlagi = 0;
	statusDotyku.sX = 0;
	statusDotyku.sY = 0;
	statusDotyku.chCzasDotkniecia = 0;
	statusDotyku.nOstCzasPomiaru = 0;

	n = CzytajPaczkeKonfigu(chPaczka, FKON_KALIBRACJA_DOTYKU);
	if (n == ROZMIAR_PACZKI_KONFIG)
	{
		kalibDotyku.fAx = KonwChar2Float(&chPaczka[2]);
		kalibDotyku.fAy = KonwChar2Float(&chPaczka[6]);
		kalibDotyku.fDeltaX = KonwChar2Float(&chPaczka[10]);
		kalibDotyku.fBx = KonwChar2Float(&chPaczka[14]);
		kalibDotyku.fBy = KonwChar2Float(&chPaczka[18]);
		kalibDotyku.fDeltaY = KonwChar2Float(&chPaczka[22]);

		statusDotyku.chFlagi |= DOTYK_SKALIBROWANY;
		chErr = BLAD_OK;
	}
	statusDotyku.chCzasDotkniecia = 1;	//potrzebne do uruchomienia pierwszej kalibracji przez trzymanie ekranu podczas włączania
	//normalnie potrzeba trzymać ekran przez kilka cykli aby zostało to uznane za dotknięcie, natomiast podcza uruchomienia jest tylko jedna próba stąd licznik musi być =1

	//sprawdź obecność panelu dotykowego
	CzytajDotyk();
	if (statusDotyku.sAdc[0] == 0x1FFF)		//taką wartość zwraca gdy nie ma podłaczonego ekranu
	{
		nZainicjowanoCM7 &= ~INIT_DOTYK;
		chErr = ERR_BRAK_DANYCH;
	}
	else
		nZainicjowanoCM7 |= INIT_DOTYK;

	return chErr;
}


