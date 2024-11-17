//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi sterownika ekranu dotykowego na układzie ADS7846
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "dotyk.h"
#include "moduly_SPI.h"
#include "errcode.h"
#include "LCD.h"
#include "display.h"
#include "RPi35B_480x320.h"
#include "sys_def_CM7.h"
#include <stdio.h>
#include <string.h>


//deklaracje zmiennych
extern SPI_HandleTypeDef hspi5;
uint8_t chEtapKalibr;	//etap kalibracji kolejnego punktu
struct _statusDotyku statusDotyku;
struct _kalibracjaDotyku kalibDotyku;
uint16_t sXd[PKT_KAL];	//surowy pomiar panelu rezystancyjnego dla danego punktu kalibracyjnego X
uint16_t sYd[PKT_KAL];	//surowy pomiar panelu rezystancyjnego dla danego punktu kalibracyjnego Y
uint16_t sXe[PKT_KAL];	//współrzędne ekranowe dla danego punktu kalibracyjnego X
uint16_t sYe[PKT_KAL];	//współrzędne ekranowe dla danego punktu kalibracyjnego Z
//deklaracje zmiennych
extern uint8_t MidFont[];
extern char chNapis[50];

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
void CzytajDotyk(void)
{
	uint32_t nOld_SPI_CFG1;
	uint32_t nCzasDotyku;

	//sprawdź czy upłyneło wystarczająco czasu od ostatniego odczytu
	nCzasDotyku = MinalCzas(statusDotyku.nOstCzasPomiaru);
	if (nCzasDotyku < 500)	//50ms -> 20Hz
		return;
	statusDotyku.nOstCzasPomiaru = HAL_GetTick();

	nOld_SPI_CFG1 = hspi5.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI

	//Ponieważ zegar SPI = 100MHz a układ może pracować z prędkością max 2,5MHz a jest na tej samej magistrali co TFT przy każdym odczytcie przestaw dzielnik zegara z 4 na 64
	hspi5.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;	//maska preskalera
	hspi5.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_64;	//Bits 30:28 MBR[2:0]: master baud rate: 011: SPI master clock/64

	//domyslnie używam innej orientacji, więc zaimeń osie X i Y
	statusDotyku.sAdc[0] = CzytajKanalDotyku(TPCHY);	//najlepiej się zmienia dla osi X
	statusDotyku.sAdc[1] = CzytajKanalDotyku(TPCHX);	//najlepiej się zmienia dla osi Y
	statusDotyku.sAdc[2] = CzytajKanalDotyku(TPCHZ1);
	statusDotyku.sAdc[3] = CzytajKanalDotyku(TPCHZ2);

	if (statusDotyku.sAdc[2] > MIN_Z)		//czy siła nacisku jest wystarczająca
	{
		if (statusDotyku.chCzasDotkniecia)
		{
			statusDotyku.chCzasDotkniecia--;
			if (!statusDotyku.chCzasDotkniecia)
			{
				statusDotyku.chFlagi |= DOTYK_DOTKNIETO;

				//oblicz współrzędne ekranowe
				float fTemp;
				fTemp = kalibDotyku.fAx * statusDotyku.sAdc[0] + kalibDotyku.fBx * statusDotyku.sAdc[1] + kalibDotyku.fDeltaX;
				statusDotyku.sX = (uint16_t)fTemp;
				fTemp = kalibDotyku.fAy * statusDotyku.sAdc[0] + kalibDotyku.fBy * statusDotyku.sAdc[1] + kalibDotyku.fDeltaY;
				statusDotyku.sY = (uint16_t)fTemp;
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
	hspi5.Instance->CFG1 = nOld_SPI_CFG1;	//przywróc poprzednie nastawy
}



////////////////////////////////////////////////////////////////////////////////
// Kalibruje ekran rezystancyjny. Wyświetla punkty do naciśnięcia i zbiera pomiary z układu dotykowego
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KalibrujDotyk(void)
{
	uint8_t chErr;
	uint16_t sKolor;

	switch(chEtapKalibr)
	{
	case 0:	//inicjalizacja zmiennych
		//LCD_Orient(POZIOMO);
		LCD_clear();
		TestObliczenKalibracji();		//testowo sprawdzenie poprawności obliczeń
		setColor(WHITE);
		setFont(MidFont);
		sprintf(chNapis, "Dotknij krzyzyk aby");
		print(chNapis, CENTER, 60);
		sprintf(chNapis, "skalibrowac ekran dotykowy");
		print(chNapis, CENTER, 80);
		chEtapKalibr++;
		break;

	case 1:	//wyświetl krzyżyk w prawym górnym rogu
		sXe[0] = DISP_HX_SIZE - BRZEG;
		sYe[0] = BRZEG;
		break;

	case 2:	//wyświetl krzyżyk w lewym górnym rogu
		sXe[1] = BRZEG;
		sYe[1] = BRZEG;
		break;

	case 3:	//wyświetl krzyżyk w lewym dolnym rogu
		sXe[2] = BRZEG;
		sYe[2] = DISP_HY_SIZE - BRZEG;
		break;

	case 4:	//wyświetl krzyżyk w prawym dolnym rogu
		sXe[3] = DISP_HX_SIZE - BRZEG;
		sYe[3] = DISP_HY_SIZE - BRZEG;
		break;

	case 5:	//wyświetl krzyżyk na środku
		sXe[4] = DISP_HX_SIZE/2;
		sYe[4] = DISP_HY_SIZE/2;
		break;
	}


	chErr = ERR_OK;	//domyślny kod błedu

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
				ObliczKalibracjeDotyku();
				//ZapiszPaczke(chPaczka);
				statusDotyku.chFlagi |= DOTYK_ZAPISANO;
				chEtapKalibr = 0;
				LCD_clear();
				return ERR_DONE;	//kod wyjścia z procedury kalibracji
			}
			statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
			//statusDotyku.chFlagi &= ~DOTYK_ZWOLNONO;
		}
	}

	//rysuj krzyżyk kalibracyjny dla etapów większych niż 0
	if (chEtapKalibr)
	{
		drawHLine(sXe[chEtapKalibr-1] - KRZYZ/2, sYe[chEtapKalibr-1], KRZYZ);
		drawVLine(sXe[chEtapKalibr-1], sYe[chEtapKalibr-1] - KRZYZ/2, KRZYZ);
	}

	sKolor = getColor();	//zapamiętaj kolor
	setColor(RED);
	sprintf(chNapis, "Dotyk ADC:");
	print(chNapis, 80, 140);

	setColor(YELLOW);
	sprintf(chNapis, "X=%3d ", statusDotyku.sAdc[0]);
	print(chNapis, 80, 160);
	sprintf(chNapis, "Y=%d  ", statusDotyku.sAdc[1]);
	print(chNapis, 80, 180);
	sprintf(chNapis, "Z1=%d   ", statusDotyku.sAdc[2]);
	print(chNapis, 80, 200);
	sprintf(chNapis, "Z2=%d ", statusDotyku.sAdc[3]);
	print(chNapis, 80, 220);
	setColor(sKolor);	//przywróć kolor
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kalibracja ekranu dotykowego na podstawie dokumentu: Calibration in touch-screen systems https://www.ti.com/lit/pdf/slyt277
// Xe1, Xe2, Xe3 oraz Ye1, Ye2, Ye3 - współrzędne X i Y punktów kalibracyjnych wyświetlanych na ekranie
// Xd1, Xd2, Xd3 oraz Yd1, Yd2, Yd3 - wartości X i Y odczytane z panelu dotykowego
// ax, ay, bx, by - wspólczynniki kalibracyjne
// deltaX, deltaY - przesunięcie między punktami?
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
void ObliczKalibracjeDotyku(void)
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

	ObliczKalibracjeDotyku();

	if ((kalibDotyku.fAx == 0.0623) && (kalibDotyku.fBx == 0.0054) && (kalibDotyku.fDeltaX == 9.9951) && (kalibDotyku.fAy == -0.0163) && (kalibDotyku.fBy == 0.01868) && (kalibDotyku.fDeltaY == -10.1458))
		return ERR_OK;
	else
		return ERR_ZLE_DANE;
}



////////////////////////////////////////////////////////////////////////////////
// Rysuje linie na ekranie
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void TestDotyku(void)
{
	sprintf(chNapis, "Test kalibracji ");
	print(chNapis, CENTER, 60);

	if (statusDotyku.chFlagi & DOTYK_DOTKNIETO)		//jeżeli był dotknięty
	{
		drawHLine(statusDotyku.sX - KRZYZ/2, statusDotyku.sY, KRZYZ);
		drawVLine(statusDotyku.sX, statusDotyku.sY - KRZYZ/2, KRZYZ);
		statusDotyku.chFlagi &= ~DOTYK_DOTKNIETO;
	}
}
