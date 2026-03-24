//////////////////////////////////////////////////////////////////////////////
//
// Zestaw funkcji do obliczeń szybkiej transformaty Fouriera
//
// (c) PitLab 23 maj 2019-2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "fft.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "czas.h"

stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) xXomega[FFT_MAX_ROZMIAR] = {0};		//wynik transformaty
stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) xWnk_tab[FFT_MAX_ROZMIAR] = {0};	//raz wyliczona tablica współczynników, stała dla danego rozmiaru wektora N do potęgi k
stZesp_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) stWejscie[FFT_MAX_ROZMIAR] = {0};		//zmiennna wejściowa
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) sDmaBufDrg[FFT_MAX_ROZMIAR] = {0};		//bufor do przechowywania drgań
float __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) fWynikFFT[FFT_MAX_ROZMIAR / 2] = {0};	//wartość sygnału wyjściowego
stZesp_t xWn2;			//wartość stała dla danego rozmiaru wektora N
stZesp_t xWnk;			//wartość stała dla danego rozmiaru wektora N do potęgi k
uint16_t sPotega;
uint16_t sProbekFFT;;
uint16_t sIndeksProbki;
uint8_t chNoweDane;
uint32_t nCzasFFT;
stFFT_t stKonfigFFT;
extern unia_wymianyCM4_t uDaneCM4;

////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje sprzet do pomiarów
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujFFT(void)
{
	stKonfigFFT.chIndeksZmiennejWe = 0;
	stKonfigFFT.chRodzajOkna = 1;
	stKonfigFFT.sWykladnikPotegi = FFT_WYKLADNIK_MIN;

	sPotega = FFT_WYKLADNIK_MIN;
	sProbekFFT = pow(2, sPotega);


}



////////////////////////////////////////////////////////////////////////////////
// Pobiera nowe dane, okienkuje je i ładuje do bufora.
// Parametry: nic Funkcja bezparametrowa bo jest wywoływana w wątku głównym
// faktyczne parametry przekazywane są w strukturze stFFT wskazując na rodzaj pobieranych danych i okno
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PobierzDaneDoFFT(void)
{
	float fOkno;

	switch(stKonfigFFT.chRodzajOkna)
		{
		case 1: fOkno = 0.53836 - 0.46164 * cos(2 * M_PI * sIndeksProbki / (sProbekFFT - 1));	break;	//Okno_Hamminga
		case 2: fOkno = 0.5 * (1 - cos(2 * M_PI * sIndeksProbki /(sProbekFFT - 1)));		break;		//Okno Hanna
		case 3: fOkno = 0.35875 - 0.48829 * cos(2 * M_PI * sIndeksProbki / (sProbekFFT - 1)) +
								  0.14128 * cos(4 * M_PI * sIndeksProbki / (sProbekFFT - 1)) -
								  0.01168 * cos(6 * M_PI * sIndeksProbki / (sProbekFFT - 1)); 	break;	//Okno Blackmana-Harrisa
		default: fOkno = 1;	break;	//okno prostokątne
		}

	switch(stKonfigFFT.chIndeksZmiennejWe)
	{
	case 0: stWejscie[sIndeksProbki].Re = fOkno * uDaneCM4.dane.fAkcel1[0];	break;	//przemnóż przez okno
	case 1: stWejscie[sIndeksProbki].Re = fOkno * uDaneCM4.dane.fZyroKal1[0];	break;
	}
	stWejscie[sIndeksProbki].Im = 0.0f;

	sIndeksProbki++;
	if (sIndeksProbki == sProbekFFT)
	{
		sIndeksProbki = 0;
		chNoweDane = 1;		//mamy komplet danych, można liczyć FFT
	}


}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje pomiary i pokazuje harmoniczne drgań w osiach X i Z
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void FFT_Drgania(void)
{
	unsigned short s;
	//int n, x, y1, y2;
	float dMod;
	uint32_t nCzasPoczatku;
	//float fWspWyp;	//współczynnik wypełnienie ekranu danymi pomiarowymi
	//float fWypPix;	//wypełnienie piksela danymi
	//float dPixel;	//wypadkowy piksel
	//unsigned short sIndexDanych;	//indeky kolejnych pobieranych danych
	//div_t dres;

	/*if (chRysujRaz)
	{
		BelkaTytulu("");
		//LCD_Orient(POZIOMO);
		//drawBitmap(0, 0, 18, 18, pitlab_logo18);	//logo producenta
		chRysujRaz = 0;
		chMenuSelPos = 0;
		TIM3->CR1 = (0<<0);  				//Bit 0 CEN: Counter enable
		TIM3->ARR = 1000000/(nFFTFreq[fMenu[DF_SET_FREQ].chVal]) - 1;
		TIM3->CR1 |= (1<<0);  				//na sam koniec włacz timer
		DrganiaMenu(RP_DFREQ);
	}*/

	if (chNoweDane)
	{
		chNoweDane = 0;
		nCzasPoczatku = PobierzCzasT6();
		/*for (s=0; s<sProbekFFT; s++)
		{
			//wybierz rodzaj funkcji okienkowanie sygnał wejściowy: https://pl.wikipedia.org/wiki/Okno_czasowe
			//switch (fMenu[DF_WINDOW].chVal)
			switch(1)
			{
			case 0:	dMod = 1;	break;	//okno prostokątne
			case 1: dMod = 0.53836 - 0.46164 * cos(2*M_PI*s/(sProbekFFT-1));	break;	//Okno_Hamminga
			case 2: dMod = 0.5 * (1 - cos(2*M_PI*s/(sProbekFFT-1)));		break;	//Okno Hanna
			case 3: dMod = 0.35875 - 0.48829 * cos(2*M_PI*s/(sProbekFFT-1)) + 0.14128 * cos(4*M_PI*s/(sProbekFFT-1)) - 0.01168 * cos(6*M_PI*s/(sProbekFFT-1)); 	break;	//Okno Blackmana-Harrisa
			default: dMod = 1;	break;	//okno prostokątne
			}

			//switch (fMenu[DF_CHANNEL].chVal)		//wybór kanału do analizy
			switch(0)
			{
			case 0: xXn[s].Re = dMod * (sDmaBufDrg[2*s+1]-2048);	break;	//odejmij offset zera od danych z osi Z i przemnóż przez okno
			case 1: xXn[s].Re = dMod * (sDmaBufDrg[2*s]-2048);		break;	//odejmij offset zera od danych z osi X i przemnóż przez okno
			case 2: xXn[s].Re = dMod * (sqrt(sDmaBufDrg[2*s]*sDmaBufDrg[2*s] + sDmaBufDrg[2*s+1]*sDmaBufDrg[2*s+1])-2896);	break;	// oś X i Z razem
			default: xXn[s].Re = sDmaBufDrg[2*s]-2048;	break;		//oś X brak okna
			}
			xXn[s].Im = 0.0;
		}*/

		FFT(&stKonfigFFT);		//licz szybką transformatę Fouriera

		//oblicz moduł sygnału i logarytmuj
		for (s=0; s<sProbekFFT/2; s++)
		{
			dMod = sqrt(xXomega[s].Re*xXomega[s].Re + xXomega[s].Im*xXomega[s].Im);	//moduł
			//if (fMenu[DF_SET_SCALE].chVal)	//1 = wykres logarytmiczny
			{
				if (dMod > 0)
					fWynikFFT[s] = log10(dMod);
				else
					fWynikFFT[s] = 0;		//nie można liczyć logarytmu z zera i liczb ujemnych
			}
			//else
				//fWynikFFT[s] = dMod / 100000;	//wartość liniowa
		}

		//inicjuj maksima
		stKonfigFFT.fWartoscMax[0] = stKonfigFFT.fWartoscMax[1] = 0.0;
		stKonfigFFT.sPozycjaMax[0] = stKonfigFFT.sPozycjaMax[1] = 0;

		//znajdź maksimum globalne
		for (s=2; s<sProbekFFT/2; s++)
		{
			if (fWynikFFT[s] > stKonfigFFT.fWartoscMax[0])
			{
				stKonfigFFT.fWartoscMax[0] = fWynikFFT[s];
				stKonfigFFT.sPozycjaMax[0] = s;
			}
		}

		//znajdź kolejne maksimum za pierwszym
		for (s = stKonfigFFT.sPozycjaMax[0]+10; s < sProbekFFT/2; s++)
		{
			if (fWynikFFT[s] > stKonfigFFT.fWartoscMax[1])
			{
				stKonfigFFT.fWartoscMax[1] = fWynikFFT[s];
				stKonfigFFT.sPozycjaMax[1] = s;
			}
		}

		nCzasFFT = MinalCzas(nCzasPoczatku);


		/*/zamaż brzegi wykresów tłem aby nie zostawały tam resztki napisów
		setColor(BLACK);
		for (x=0; x<AD_STARTX; x++)
		{
			drawVLine(x, AD_STARTY, AD_Y_SIZE);							//lewy brzed
			drawVLine(x+AD_X_SIZE+AD_STARTX, AD_STARTY, AD_Y_SIZE);		//prawy brzeg
		}

		//rysuj
		fWspWyp = (float)(AD_X_SIZE)/((sProbekFFT/2)-1);	//współczynnik wypełnienia ekranu danymi pix/słowo
		fWypPix = 0;		//sciskanie danych ekranu
		sIndexDanych = 0;
		dPixel = 0;
		y1 = AD_STARTY + AD_Y_SIZE;
		for (s=0; s<AD_X_SIZE; s++)
		{
			x = AD_STARTX + s;
			setColor(BLACK);	//zamaż stary wykres tłem
			drawVLine(x, AD_STARTY, AD_Y_SIZE);

			//linie siatki pionowej
			dres = div(s, AD_X_SIZE/AD_X_DIV);
			if (!dres.rem)
			{
				setColor(GRAY40);	//podziałka w osi x
				for (n=0; n<30; n++)	//rysuj linie przerywaną
					drawPixel(x, n*AD_Y_SIZE/30 + AD_STARTY);
			}

			//linie siatki poziomej, brak reszty z dzielenia oznacza konieczność narysowania linii
			dres = div(s, AD_X_SIZE/50);
			if (!dres.rem)		//brak reszty, czyli rysuj kropkę linii pozionych
			{
				setColor(GRAY40);
				for (n=0; n<=AD_Y_DIV; n++)
					drawPixel(x, n*AD_Y_SIZE/AD_Y_DIV + AD_STARTY);
			}

			//wykres FFT
			if (fWspWyp > 1.0)	//wykresy będą rozciagane do ekranu
			{
				fWypPix -= 1;	//odejmij piksel obrazu
				if (fWypPix < 0)
				{
					//sIndexDanych++;		//pre-inkrementuj aby pominąć dane [0] oraz aby za pętlą mieć dostęp do ostatniej danej
					dPixel = fWynikFFT[sIndexDanych];	//pobierz słowo danych
					sIndexDanych++;
					fWypPix += fWspWyp;
				}
			}
			else	//wykresy będą ściskale po wiecej niż jedną liczbę na piksel
			{
				do	//pobierz dane z wyliczonych indeksów zmiennej
				{
					fWypPix += fWspWyp;	//wypełnienie piksela danymi
					//sIndexDanych++;		//pre-inkrementuj aby pominąć dane [0] oraz aby za pętlą mieć dostęp do ostatniej danej
					//dPixel = (dPixel + 2*fWynikFFT[sIndexDanych])/4;	//filtr uśredniający
					dPixel = fWynikFFT[sIndexDanych];
					sIndexDanych++;
				}
				while (fWypPix < 1);
				fWypPix -= 1;	//odejmij piksel obrazu
			}
			y2 = (AD_STARTY + AD_Y_SIZE) - (dPixel * AD_Y_SIZE) / (AD_Y_DIV * DBDIV);
			if (y2 > AD_STARTY + AD_Y_SIZE)
				y2 = AD_STARTY + AD_Y_SIZE;
			else
			if (y2 < AD_STARTY)
				y2 = AD_STARTY;

			//ustaw kolor wykresu X lub Z
			switch (fMenu[DF_CHANNEL].chVal)
			{
			case 0: setColor(CYAN);		break;	//oś X
			case 1:	setColor(MAGENTA);	break;	//oś Z
			case 2: setColor(YELLOW);	break;	//osie X+Z
			}
			drawLine(x, y1, x, y2);		//wykres liniowy
			y1 = y2;
		}

		//rysuj znaczniki i opis maksimów
		fWypPix = (float)nFFTFreq[fMenu[DF_SET_FREQ].chVal] / sProbekFFT;	//współczynnik  do obliczenia częstotliwości w maksimum
		setColor(WHITE);

		x = AD_STARTX + mLocalMax[0].Pos * fWspWyp;
		y1 = (AD_STARTY + AD_Y_SIZE) - (mLocalMax[0].Val * AD_Y_SIZE) / (AD_Y_DIV * DBDIV);
		drawVLine(x, y1-4, 8);
		drawHLine(x-4, y1, 8);
		if (y1 < AD_STARTY+14)
			y1 = AD_STARTY+14;		//obniż napis jeżeli jest za wysoko
		else
		if (y1 > AD_STARTY+AD_Y_SIZE-28)
			y1 = AD_STARTY+AD_Y_SIZE-28;
		if (fMenu[DF_SET_SCALE].chVal)
			sprintf(chNapis, "%.2f dB", mLocalMax[0].Val);
		else
			sprintf(chNapis, "%.2f", mLocalMax[0].Val);
		print(chNapis, x+10, y1-14, 0);
		sprintf(chNapis, "%.0f Hz", fWypPix*mLocalMax[0].Pos);
		print(chNapis, x+10, y1+2, 0);

		x = AD_STARTX + mLocalMax[1].Pos * fWspWyp;
		y1 = (AD_STARTY + AD_Y_SIZE) - (mLocalMax[1].Val * AD_Y_SIZE) / (AD_Y_DIV * DBDIV);
		drawVLine(x, y1-4, 8);
		drawHLine(x-4, y1, 8);
		if (y1 < AD_STARTY+14)
			y1 = AD_STARTY+14;		//obniż napis jeżeli jest za wysoko
		else
		if (y1 > AD_STARTY+AD_Y_SIZE-28)
			y1 = AD_STARTY+AD_Y_SIZE-28;
		if (fMenu[DF_SET_SCALE].chVal)
			sprintf(chNapis, "%.2f dB", mLocalMax[1].Val);
		else
			sprintf(chNapis, "%.2f", mLocalMax[1].Val);
		print(chNapis, x+10, y1-14, 0);
		sprintf(chNapis, "%.0f Hz", fWypPix*mLocalMax[1].Pos);
		print(chNapis, x+10, y1+2, 0);

		//opis nagłówka
		setFont(MidFont);
		setColor(GREEN);
		setBackColor(MENU_TLO_BAR);
		sprintf(chNapis, "%ddB/div ", DBDIV);
		print(chNapis, 30, 0, 0);

		if (sProbekFFT)	//zabezpieczenie przed dzieleniem przez 0
		{
			sprintf(chNapis, "%dHz/div ", nFFTFreq[fMenu[DF_SET_FREQ].chVal]/20);
			print(chNapis, 140, 0, 0);
		}

		setColor(GREEN);
		sprintf(chNapis, "%dms ", (nTotalTime)/10);
		print(chNapis, 250, 0, 0);
		setBackColor(BLACK);

		chNoweDaneADC = 0;
		DMA1_Channel1->CCR &= ~(1<<0);				//0 EN: Channel disable
		DMA1_Channel1->CNDTR = 2*pow(2,sPotega);	//15:0 NDT[15:0]: Number of data to transfer
		DMA1_Channel1->CCR |= (1<<0);				//0 EN: Channel enable
		ADC1->CR2 |= (1<<20);
	}

	//wyświetla menu kalibracyjne
	if (chPressed & ENCODER)
	{
		chMode ^= 1;
		DrganiaMenu(RP_DFREQ);
		chPressed &= ~ENCODER;
	}


	//obsługa menu
	if (nEnkoder != nLastEnkoder)
	{

		if (chMode)	//tryb zmiany parametrów polecenia
		{
			//obsługa zmiany kalibracji
			n = nEnkoder - nLastEnkoder;
			n += fMenu[chMenuSelPos].chVal;
			if (n < 0)	//mniejsze niż 0
				fMenu[chMenuSelPos].chVal = 0;
			else
			if (n >= fMenu[chMenuSelPos].chMaxVal)
				fMenu[chMenuSelPos].chVal =  fMenu[chMenuSelPos].chMaxVal - 1;
			else
				fMenu[chMenuSelPos].chVal = n;

			switch (chMenuSelPos)
			{
			case DF_SET_FREQ:	//częstotliwość próbkowania
				TIM3->CR1 = (0<<0);  				//Bit 0 CEN: Counter enable
				TIM3->ARR = 1000000/(nFFTFreq[fMenu[chMenuSelPos].chVal]) - 1;
				TIM3->CR1 |= (1<<0);  				//na sam koniec włacz timer
				break;

			case DF_SET_BASE:	//wielkość FFT
				sPotega = FFT_START_BASE + fMenu[DF_SET_BASE].chVal;
				sProbekFFT = pow(2, sPotega);
				break;
			case DF_MOTOR: TIM1->CCR2 = 900 + 5*fMenu[chMenuSelPos].chVal;	break;
			case DF_SET_SCALE: 	break;	//rodzaj podziałki: log/lin
			}
			chBylaZmianaKonf |= KONF_FREQ;
		}
		else	//tryb wyboru polecenia
		{
			chMenuSelPos += nEnkoder - nLastEnkoder;
			if (chMenuSelPos > SIGNED_8BIT)	//mniejsze niż 0
				chMenuSelPos = 0;
			else
			if (chMenuSelPos > MDFPOS-1)
				chMenuSelPos = MDFPOS-1;
		}
		DrganiaMenu(RP_DFREQ);
		nLastEnkoder = nEnkoder;*/
	}
}



////////////////////////////////////////////////////////////////////////////////
// liczy FFT dla pary danych. Dane zostają w tym samym miejscu
// Parametry:
// xWeWyP - struktura danych wejściowych-wyjściowych parzystych
// xWeWyN - struktura danych wejściowych-wyjściowych nieparzystych
// Zwraca: dane w tych samych miejacach
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
void FFT_Motyl(stZesp_t *stWeWyP, stZesp_t *stWeWyN, stZesp_t *stWnk)
{
	stZesp_t stWeP;
	stZesp_t stWeN;
	stZesp_t stTemp;

	//przepisz parzyste dane wejściowe do zmiennej
	stWeP.Re = stWeWyP->Re;
	stWeP.Im = stWeWyP->Im;

	//przepisz nieparzste dane
	stWeN.Re = stWeWyN->Re;
	stWeN.Im = stWeWyN->Im;

	//mnóż Wnk razy nieparzyste
	stTemp = MulComplex(*stWnk, stWeN);

	//górny wynik motylka
	stWeWyP->Re = stWeP.Re + stTemp.Re;
	stWeWyP->Im = stWeP.Im + stTemp.Im;

	//dolny wynik motylka
	stWeWyN->Re = stWeP.Re - stTemp.Re;
	stWeWyN->Im = stWeP.Im - stTemp.Im;
}



////////////////////////////////////////////////////////////////////////////////
// liczy szybką transformatę fouriera
// wylicza połowę współczynników Wnk uwzlędniajac warunek szczególny dla k=1
// wejście: *xXn
// wyjscie: *xXomega
// Czas wykonania: 61,7ms @256, 133,1 ms @512
////////////////////////////////////////////////////////////////////////////////
void FFT(stFFT_t *konfig)
{
	//struct complex xSumP, xSumN;
	//unsigned short k, j, N, x, b;
	uint16_t N, b, j;
	float dTemp;

	//przepisz dane do struktury wyjściowej w kolejności odwróconego bitu
	for (uint16_t k=0; k<konfig->sLiczbaProbek; k++)
	{
		//odwróć bity w słowie o rozmiarze 2^sPotega
		j = 0;
		b = k;
		for (uint16_t x=0; x<konfig->sWykladnikPotegi; x++)
		{
			j <<= 1;
			j |= b & 1;
			b >>= 1;
		}

		xXomega[j].Re = stWejscie[k].Re;
		xXomega[j].Im = stWejscie[k].Im;
	}


	for (uint16_t x=1; x<konfig->sWykladnikPotegi+1; x++)		//iteracje drzewa podziałów x^2=PROBEK_DRGANIA
	{
		N = pow(2, x);	//oblicz liczbę współczynników Wnk = N = 2^x

		//Przygotuj współczynniki Wnk dla pierwszej połowy zakresu (druga będzie identyczna ze znakiem minus)
		for (uint16_t k=0; k<(N/2); k++)
		{
			if (k==0)		//warunek szczególny
			{
				xWnk_tab[k].Re = 1;
				xWnk_tab[k].Im = 0;
			}
			else
			{
				//licz e^(-j2Pik/N)
				dTemp = -2*M_PI*k/N;
				xWnk_tab[k].Re= cos(dTemp);
				xWnk_tab[k].Im= -sin(dTemp);
			}
		}

		//licz operacje na motylach na danym poziomie drzewa
		for (uint16_t j=0; j<N/2; j++)		//indeks motyla w grupie
		{
			for (uint16_t k=0; k<konfig->sLiczbaProbek/N; k++)		//indeks grupy motyli
			{
				FFT_Motyl(&xXomega[k*N + j], &xXomega[k*N + j + N/2], &xWnk_tab[j]);
			}
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// mnożenie liczb zespolonych
// Parametry: stWejscie1, stWejscie2 - struktury obu mnożonych liczb
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t MulComplex(stZesp_t stWejscie1, stZesp_t stWejscie2)
{
	stZesp_t stWynik;

	stWynik.Re = (stWejscie1.Re * stWejscie2.Re) - (stWejscie1.Im * stWejscie2.Im);
	stWynik.Im = (stWejscie1.Im * stWejscie2.Re) + (stWejscie1.Re * stWejscie2.Im);
	return stWynik;
}



////////////////////////////////////////////////////////////////////////////////
// dodawanie liczb zespolonych
// Parametry: stWejscie1, stWejscie2 - struktury obu dodawanych liczb
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t AddComplex(stZesp_t stWejscie1, stZesp_t stWejscie2)
{
	stZesp_t stWynik;

	stWynik.Re = stWejscie1.Re + stWejscie2.Re;
	stWynik.Im = stWejscie1.Im + stWejscie2.Im;
	return stWynik;
}



////////////////////////////////////////////////////////////////////////////////
// podnoszenie liczb zespolonych do potęgi całkowitej na podstawie wzoru de Moivre'a:
// https://www.matemaks.pl/wzor-de-moivre-a-potegowanie-liczb-zespolonych.html
// Parametry: stWejscie - struktura wejściowa liczby zespolonej
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t PowComplex(stZesp_t stPodstawa, float fWykładnik)
{
	stZesp_t stWynik;
	float fModuł, fFi, fTemp;

	fModuł = sqrt(stPodstawa.Re*stPodstawa.Re + stPodstawa.Im*stPodstawa.Im);	//moduł z podstawy

	//oblicz kąt fi  https://pl.wikipedia.org/wiki/Liczby_zespolone
	if (stPodstawa.Im >= 0)
		fFi = acos(stPodstawa.Re / fModuł);
	else
		fFi = -1 * acos(stPodstawa.Re / fModuł);

	fTemp  = pow(fModuł, fWykładnik);	//moduł do potęgi

	stWynik.Re = fTemp * cosf(fWykładnik * fFi);
	stWynik.Im = fTemp * sinf(fWykładnik * fFi);
	return stWynik;
}



////////////////////////////////////////////////////////////////////////////////
// podnoszenie liczby e do potęgi zespolonej
// korzystając ze wzoru: e^ix = cos(x) + isin(x) patrz  https://pl.wikipedia.org/wiki/Liczby_zespolone
// Parametry: stWejscie - struktura wejściowa liczby zespolonej
// Zwraca: struktura wyjściowa liczby zespolonej
////////////////////////////////////////////////////////////////////////////////
stZesp_t PowEComplex(stZesp_t stWejscie)
{
	stZesp_t stWynik;

	stWynik.Re = cos(stWejscie.Im);
	stWynik.Im = sin(stWejscie.Im);
	return stWynik;
}



