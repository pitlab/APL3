//////////////////////////////////////////////////////////////////////////////
//
// Zestaw narzędzi do obróbki obrazu
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "analiza_obrazu.h"
#include "stdlib.h"
#include <cmsis_gcc.h>	//
#include "czas.h"
#include "jpeg.h"

static uint32_t nSumaR[ROZMIAR_HIST_KOLOR];
static uint32_t nSumaG[ROZMIAR_HIST_KOLOR];
static uint32_t nSumaB[ROZMIAR_HIST_KOLOR];




////////////////////////////////////////////////////////////////////////////////
// Konwertuje dane RGB w formacie 565 znajdujace się w buforze obrazRGB565 do danych czarno-białych o głębi 7-bitowej
// Zapisuje kopię obrazu czarnobiałego w formacie wyświetlacza RGB565
// Parametry:
// [we] obrazRGB565* - wskaźnik na bufor[2*rozmiar] z obrazem kolorowym
// [wy] obrazCB* - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [we] rozmiar - rozmiar bufora
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwersjaRGB565doCB7(uint16_t *obrazRGB565, uint8_t *obrazCB, uint32_t rozmiar)
{
	uint16_t pix;
	for (uint32_t n=0; n<rozmiar; n++)
	{
		pix = *(obrazRGB565 + n);
		//bity czerwony i niebieski mają skalę 5-bitową (32), zielony 6-bitową (64).
		*(obrazCB + n) = (pix >> 11) + ((pix & 0x07E0) >> 5) + (pix & 0x1F);		//B32 + G64 + R32
	}
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje dane czarno-białe o głębi 7-bitowej na RGB565
// Parametry:
// [we] obrazCB* - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] obrazCB565* - wskaźnik na bufor[2*rozmiar] z obrazem czarno-białym
// rozmiar - rozmiar bufora
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwersjaCB7doRGB5666(uint8_t *obrazCB, uint16_t *obrazCB565, uint32_t rozmiar)
{
	uint16_t pixCB, pixRB, pixG;
	for (uint32_t n=0; n<rozmiar; n++)
	{
		pixCB = *(obrazCB+n);
		pixRB = pixCB >> 2;	//składowe: czerwona i niebieska
		pixG  = pixCB >> 1;	//składowa zielona
		*(obrazCB565 + n) = (pixRB << 11) + (pixG << 5) + pixRB;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje dane czarno-białe z kamery o głębi 8-bitowej na RGB666
// Parametry:
// [we] obrazCB* - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] obrazCB666* - wskaźnik na bufor[3*rozmiar] z obrazem czarno-białym
// rozmiar - rozmiar bufora
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwersjaCB8doRGB666(uint8_t *obrazCB, uint8_t *obrazCB666, uint32_t rozmiar)
{
	uint16_t pixCB, pixRGB;
	for (uint32_t n=0; n<rozmiar; n++)
	{
		pixCB = *(obrazCB+n);
		pixRGB = pixCB;
		*(obrazCB666 + 3*n + 0) = pixRGB;
		*(obrazCB666 + 3*n + 1) = pixRGB;
		*(obrazCB666 + 3*n + 2) = pixRGB;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje 16-bitowy obraz RGB565 na 24-bitowy RGB666
// Parametry:
// [we] obrazCB* - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] obrazCB565* - wskaźnik na bufor[2*rozmiar] z obrazem czarno-białym
// rozmiar - rozmiar bufora
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwersjaRGB565doRGB666(uint16_t *obrazRG565, uint8_t *obrazRGB666, uint32_t rozmiar)
{
	uint16_t sPix565;
	for (uint32_t n=0; n<rozmiar; n++)
	{
		sPix565 = *(obrazRG565 + n);
		*(obrazRGB666 + 3*n + 0) = ((sPix565 & 0xF800) >> 11) << 3;	//R
		*(obrazRGB666 + 3*n + 1) = ((sPix565 & 0x07E0) >> 5) << 2;	//G
		*(obrazRGB666 + 3*n + 2) =  (sPix565 & 0x001F) << 3;		//B
	}
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje wiersz bloków 8x8 obrazu w formacie RGB888 znajdujace się w buforze
// na kolejne bloki MCU dla obrazu YCbCr422
// Parametry:
// [we] *obrazRGB888 - wskaźnik na bufor[3*rozmiar] z obrazem wejściowym
// [wy] *obrazYCbCr - wskaźnik na bufor[2*rozmiar] z obrazem wyjściowym
// [we] sSzerokosc - szerokość obrazu w pikselach. Musi być podzielna przez 8
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KonwersjaRGB888doYCbCr422(uint8_t *obrazRGB888, uint8_t *obrazYCbCr, uint16_t sSzerokosc)
{
	uint32_t nOffsetWiersza, nOffsetBloku;
	uint32_t nOfsetWe, nOffsetWyjscia;
	uint16_t chLiczbaParBlokow = sSzerokosc / 16;
	uint8_t chR1, chG1, chB1, chR2, chG2, chB2;
	int16_t sY1, sY2, sCb, sCr;

	for (uint8_t b=0; b<chLiczbaParBlokow; b++)	//petla po połowie bloków na szerokosci obrazu
	{
		nOffsetBloku = 2 * b * 24;
		for (uint8_t y=0; y<8; y++)				//pętla po wierszach
		{
			nOffsetWiersza = y * sSzerokosc * 3;
			for (uint8_t x=0; x<8; x++)			//pętla po  kolumnach
			{
				nOfsetWe = nOffsetWiersza + nOffsetBloku + 3 * x;
				chR1 = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku lewego
				chG1 = *(obrazRGB888 + nOfsetWe + 1);
				chB1 = *(obrazRGB888 + nOfsetWe + 2);

				nOfsetWe += 24;
				chR2 = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku prawego
				chG2 = *(obrazRGB888 + nOfsetWe + 1);
				chB2 = *(obrazRGB888 + nOfsetWe + 2);

				//Formowanie MCU
				nOffsetWyjscia = b * ROZMIAR_MCU422 + y * 8 + x;
				sY1 = ((uint16_t)chR1 * 77 + (uint16_t)chG1 * 150 + (uint16_t)chB1 * 29) >> 8;	//255 cykli
				*(obrazYCbCr + nOffsetWyjscia + 0 * ROZMIAR_BLOKU) = (uint8_t)sY1;

				sY2 = ((uint16_t)chR2 * 77 + (uint16_t)chG2 * 150 + (uint16_t)chB2 * 29) >> 8;
				*(obrazYCbCr + nOffsetWyjscia + 1 * ROZMIAR_BLOKU) = (uint8_t)sY2;

				sCb = (  ((int32_t)(chR1+chR2) * (-43)) + ((int32_t)(chG1+chG2) * (-84)) + ((int32_t)(chB1+chB2) * (127)) + 256) >> 9;
				*(obrazYCbCr + nOffsetWyjscia + 2 * ROZMIAR_BLOKU) = (uint8_t)sCb;

				sCr = ( ((int32_t)(chR1+chR2) * (127)) + ((int32_t)(chG1+chG2) * (-106)) + ((int32_t)(chB1+chB2) * (-21)) + 256) >> 9;	//284 cykle
				*(obrazYCbCr + nOffsetWyjscia + 3 * ROZMIAR_BLOKU) = (uint8_t)sCr;
			}
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje splot Robertsa do detekcji krawędzi na obrazie monochromatycznym
// Są dwie macierze splotu robiace detekcję w poziomie i pionie
// [1  0] [ 0 1]
// [0 -1] [-1 0]
// Parametry:
// [we] obrazWe* - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] obrazWy* - wskaźnik na bufor[rozmiar] z obrazem  po detekcji (może być ten sam obszar co wejściowy)
// [we] szerokosc - szerokość obrazu (ilość pikseli w wierszu)
// [we] wysokosc - wysokość obrazu (ilość wierszy)
// [we] prog - próg odcięcia, wartość powyżej ktorej cokolwiek znajdzie sie w obrazie wyjściowym
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void DetekcjaKrawedziRoberts(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog)
{
	uint16_t x, y;
	uint32_t yszer;
	int pixWe[4], pixWy;
	uint8_t *wiersz;

	*obrazWy = 0;	//inicjuj pierwszy piksel, bo pętla go nie ruszy
	for (y=0; y<wysokosc-1; y++)
	{
		yszer = y*szerokosc;
		for (x=1; x<szerokosc-1; x++)
		{
			wiersz = obrazWe + yszer + x;
			pixWe[0] = *(wiersz);
			pixWe[1] = *(wiersz + 1);

			wiersz = obrazWe + ((y+1)*szerokosc) + x;
			pixWe[2] = *(wiersz);
			pixWe[3] = *(wiersz + 1);

			pixWy = abs(pixWe[0] - pixWe[3]) + abs(pixWe[2] - pixWe[1]);
			if (pixWy <= prog)
				pixWy = 0;
			*(obrazWy + yszer + x) = pixWy;
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Wykonuje splot Sobela do detekcji krawędzi na obrazie monochromatycznym
// Są dwie macierze splotu robiace detekcję w poziomie i pionie
// [P0, P1, P2]   [-1 0 1]   [ 1  2  1]
// [P3, P4, P5] x [-2 0 2] x [ 0  0  0]
// [P6, P7, P8]   [-1 0 1]   [-1 -2 -1]
// Parametry:
// [we] obrazWe* - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] obrazWy* - wskaźnik na bufor[rozmiar] z obrazem po detekcji
// [we] szerokosc - szerokość obrazu (ilość pikseli w wierszu)
// [we] wysokosc - wysokość obrazu (ilość wierszy)
// [we] prog - wartość progująca
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void DetekcjaKrawedziSobel(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog)
{
	uint16_t x, y;
	uint8_t *wiersz;
	uint8_t pix, pixWe[9];//, pixWy[9];
	uint32_t yszer, ym1szer, yp1szer;	//Y*szerokosc, Y minus 1 * szerokosc, Y plus 1 * szerokosc

	for (y=1; y<wysokosc-1; y++)
	{
		yszer = y * szerokosc;
		ym1szer = (y-1) * szerokosc;
		yp1szer = (y+1) * szerokosc;
		for (x=1; x<szerokosc-1; x++)
		{
			wiersz = obrazWe + ym1szer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);

			wiersz = obrazWe + yszer + x;
			pixWe[3] = *(wiersz - 1);
			pixWe[4] = *(wiersz);
			pixWe[5] = *(wiersz + 1);

			wiersz = obrazWe + yp1szer + x;
			pixWe[6] = *(wiersz - 1);
			pixWe[7] = *(wiersz);
			pixWe[8] = *(wiersz + 1);

			pix = abs(pixWe[2] - pixWe[0] + pixWe[8] - pixWe[6]  + (2 * (pixWe[5] - pixWe[3]))) +
			      abs(pixWe[0] - pixWe[6] + pixWe[2] - pixWe[8]  + (2 * (pixWe[1] - pixWe[7])));
			pix >>= 2;			//normalizacja dzielac przez 4
			if (pix <= prog)	//progowanie
				pix = 0;
			*(obrazWy + x + yszer) = pix;
		}
	}

	//inicjuj pierwszy wiersz, bo algorytm tam nie sięgnie
	for (x=0; x<szerokosc; x++)
		*(obrazWy + x) = 0;

	//wypełnienie ostatniego wiersza
	yszer = (wysokosc-1)*szerokosc;
	for (x=0; x<szerokosc; x++)
		*(obrazWy + yszer + x) = 0;
}



////////////////////////////////////////////////////////////////////////////////
// Liczy histogram z 7-bitowego obrazu czarno-białego
// Parametry:
// [we] *chObraz - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] *chHist - 129 elementowy, 8-bitowy histogram. Ostatni element jest dla liczb spoza zakresu
// [we] nRozmiar - ilość pikseli do analizy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HistogramCB7(uint8_t *chObraz, uint32_t nRozmiar, uint8_t *chHist)
{
	uint32_t nHistogram[ROZMIAR_HIST_CB7 + 1], temp;
	uint8_t pix;

	for (uint8_t n=0; n<ROZMIAR_HIST_CB7 + 1; n++)
		nHistogram[n] = 0;

	for (uint32_t n=0; n<nRozmiar; n++)
	{
		pix = *(chObraz + n);
		if (pix > ROZMIAR_HIST_CB7)
			pix = ROZMIAR_HIST_CB7;	//ostatnia pozycja zbiera śmieci spoza zakresu 7-bit
		nHistogram[pix]++;
	}

	for (uint8_t n=0; n<ROZMIAR_HIST_CB7 + 1; n++)
	{
		temp = (uint8_t)(nHistogram[n] / DZIELNIK_HIST_CB);
		if (temp > 255)
			temp = 255;
		*(chHist + n) = temp;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Liczy histogram z 8-bitowego obrazu czarno-białego
// Parametry:
// [we] *chObraz - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] *chHist - 256 elementowy, 8-bitowy histogram.
// [we] nRozmiar - ilość pikseli do analizy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HistogramCB8(uint8_t *chObraz, uint32_t nRozmiar, uint8_t *chHist)
{
	uint32_t nHistogram[ROZMIAR_HIST_CB8], temp;
	uint8_t pix;

	for (uint16_t n=0; n<ROZMIAR_HIST_CB8; n++)
		nHistogram[n] = 0;

	for (uint32_t n=0; n<nRozmiar; n++)
	{
		pix = *(chObraz + n);
		nHistogram[pix]++;
	}

	for (uint16_t n=0; n<ROZMIAR_HIST_CB8; n++)
	{
		temp = (uint8_t)(nHistogram[n] / DZIELNIK_HIST_CB);
		if (temp > 255)
			temp = 255;
		*(chHist + n) = temp;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Liczy histogram z obrazu RGB565.
// Bity czerwony i niebieski mają skalę 5-bitową, zielony 6-bitową, więc żeby można było porównywać wyrównuję histogram do 5 bitów.
// Parametry:
// [we] obraz* - wskaźnik na bufor[2*rozmiar] z obrazem RGB565
// [wy] histR - 32 elementowy, 16-bitowy histogram
// [wy] histG - 32 elementowy, 16-bitowy histogram
// [wy] histB - 32 elementowy, 16-bitowy histogram
// [we] rozmiar - ilość pikseli do analizy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HistogramRGB565(uint16_t *obrazRGB565, uint32_t rozmiar, uint8_t *histR, uint8_t *histG, uint8_t *histB)
{
	uint16_t sPixel;
	uint32_t temp;

	//inicjuj sumę pokseli w danym kolorze, później będzie inkrementowana, wiec musi zaczynać się od zera
	for (uint8_t n=0; n<ROZMIAR_HIST_KOLOR; n++)
	{
		nSumaR[n] = 0;
		nSumaG[n] = 0;
		nSumaG[n+ROZMIAR_HIST_KOLOR] = 0;
		nSumaB[n] = 0;
	}

	for (uint32_t n=0; n<rozmiar; n++)
	{
		sPixel = *(obrazRGB565 + n);
		nSumaR[(sPixel & 0xF800) >> 11]++;
		nSumaG[(sPixel & 0x07C0) >> 6]++;	//ogranicz rozdzielczość zielonego do 5 bitów
		nSumaB[(sPixel & 0x001F)]++;
	}

	for (uint8_t n=0; n<ROZMIAR_HIST_KOLOR; n++)
	{
		temp = nSumaR[n] / DZIELNIK_HIST_RGB;
		*(histR+n) = temp & 0xFF;

		temp = nSumaG[n] / DZIELNIK_HIST_RGB;
		*(histG+n) = temp & 0xFF;

		temp = nSumaB[n] / DZIELNIK_HIST_RGB;
		*(histB+n) = temp & 0xFF;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Progowanie obrazu mnochromatycznego
// Parametry:
// [we/wy] obraz* - wskaźnik na bufor[rozmiar] z obrazem
// [we] prog - wartość progu uznania za białe
// [we] rozmiar - ilość pikseli do analizy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Progowanie(uint8_t *obraz, uint8_t prog, uint32_t rozmiar)
{
	for (uint32_t n=0; n<rozmiar; n++)
	{
		if (*(obraz + n) > prog)
			*(obraz + n) = 0x7F;
		else
			*(obraz + n) = 0x00;
	}
}



////////////////////////////////////////////////////////////////////////////////
// Dylatacja morfologiczna. Jeżeli chociaż jeden z piskeli sąsiadujacych ma wartość 1 to punkt centralny też.
// Parametry:
// [we] obrazWe* - wskaźnik na bufor[rozmiar] z obrazem wejściowym
// [wy] obrazWy* - wskaźnik na bufor[rozmiar] z obrazem wyjściowym
// [we] szerokosc - szerokość obrazu (ilość pikseli w wierszu)
// [we] wysokosc - wysokość obrazu (ilość wierszy)
// [we] prog - wartość progująca akcję przyrostu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Dylatacja(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog)
{
	uint16_t x, y; //, n;
	uint8_t *wiersz;
	uint8_t pixWe[3];
	uint32_t yszer, ym1szer, yp1szer;	//Y*szerokosc, Y minus 1 * szerokosc, Y plus 1 * szerokosc

	for (y=1; y<wysokosc-1; y++)
	{
		yszer = y * szerokosc;
		ym1szer = (y-1) * szerokosc;
		yp1szer = (y+1) * szerokosc;
		for (x=1; x<szerokosc-1; x++)
		{
			wiersz = obrazWe + ym1szer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] > prog) || (pixWe[1] > prog)  || (pixWe[2] > prog))
			{
				*(obrazWy + yszer + x) = 0x7F;
				continue;
			}

			wiersz = obrazWe + yszer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] > prog) || (pixWe[1] > prog)  || (pixWe[2] > prog))
			{
				*(obrazWy + yszer + x) = 0x7F;
				continue;
			}

			wiersz = obrazWe + yp1szer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] > prog) || (pixWe[1] > prog)  || (pixWe[2] > prog))
			{
				*(obrazWy + yszer + x) = 0x7F;
				continue;
			}
			*(obrazWy + yszer + x) = 0;
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Erozja morfologiczna. Jeżeli chociaż jeden z piskeli sąsiadujacych ma wartość 0 to punkt centralny też.
// Parametry:
// [we] obrazWe* - wskaźnik na bufor[rozmiar] z obrazem wejściowym
// [wy] obrazWy* - wskaźnik na bufor[rozmiar] z obrazem wyjściowym
// [we] szerokosc - szerokość obrazu (ilość pikseli w wierszu)
// [we] wysokosc - wysokość obrazu (ilość wierszy)
// [we] prog - wartość progująca akcję przyrostu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Erozja(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog)
{
	uint16_t x, y;
	uint8_t *wiersz;
	uint8_t pixWe[3];
	uint32_t yszer, ym1szer, yp1szer;	//Y*szerokosc, Y minus 1 * szerokosc, Y plus 1 * szerokosc

	for (y=1; y<wysokosc-1; y++)
	{
		yszer = y * szerokosc;
		ym1szer = (y-1) * szerokosc;
		yp1szer = (y+1) * szerokosc;
		for (x=1; x<szerokosc-1; x++)
		{
			wiersz = obrazWe + ym1szer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] <= prog) || (pixWe[1] <= prog)  || (pixWe[2] <= prog))
			{
				*(obrazWy + yszer + x) = 0x00;
				continue;
			}

			wiersz = obrazWe + yszer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] <= prog) || (pixWe[1] <= prog)  || (pixWe[2] <= prog))
			{
				*(obrazWy + yszer + x) = 0x00;
				continue;
			}

			wiersz = obrazWe + yp1szer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] <= prog) || (pixWe[1] <= prog)  || (pixWe[2] <= prog))
			{
				*(obrazWy + yszer + x) = 0x00;
				continue;
			}
			*(obrazWy + yszer + x) = 0x7F;
		}
	}
}



////////////////////////////////////////////////////////////////////////////////
// Odszumianie jako wariant erozji. Usuń piksel jeżeli nie ma żadnego sąsiada.
// Parametry:
// [we] obrazWe* - wskaźnik na bufor[rozmiar] z obrazem wejściowym
// [wy] obrazWy* - wskaźnik na bufor[rozmiar] z obrazem wyjściowym
// [we] szerokosc - szerokość obrazu (ilość pikseli w wierszu)
// [we] wysokosc - wysokość obrazu (ilość wierszy)
// [we] prog - wartość progująca akcję przyrostu
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void Odszumianie(uint8_t *obrazWe, uint8_t *obrazWy, uint16_t szerokosc, uint16_t wysokosc, uint8_t prog)
{
	uint16_t x, y;
	uint8_t *wiersz;
	uint8_t pix, pixWe[3];
	uint32_t yszer, ym1szer, yp1szer;	//Y*szerokosc, Y minus 1 * szerokosc, Y plus 1 * szerokosc

	for (y=1; y<wysokosc-1; y++)
	{
		yszer = y * szerokosc;
		ym1szer = (y-1) * szerokosc;
		yp1szer = (y+1) * szerokosc;
		for (x=1; x<szerokosc-1; x++)
		{
			//zaczynam od środkowego wiersza aby przechwycić wartość piksela centralnego
			wiersz = obrazWe + yszer + x;
			pixWe[0] = *(wiersz - 1);
			pix = pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] > prog) || (pixWe[1] > prog)  || (pixWe[2] > prog))
			{
				*(obrazWy + yszer + x) = pix;
				continue;
			}

			wiersz = obrazWe + ym1szer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] > prog) || (pixWe[1] > prog)  || (pixWe[2] > prog))
			{
				*(obrazWy + yszer + x) = pix;
				continue;
			}

			wiersz = obrazWe + yp1szer + x;
			pixWe[0] = *(wiersz - 1);
			pixWe[1] = *(wiersz);
			pixWe[2] = *(wiersz + 1);
			if ((pixWe[0] > prog) || (pixWe[1] > prog)  || (pixWe[2] > prog))
			{
				*(obrazWy + yszer + x) = pix;
				continue;
			}
			*(obrazWy + yszer + x) = 0x00;	//jeżeli nie było sąsiadów to usuń go
		}
	}
}


