//////////////////////////////////////////////////////////////////////////////
//
// Zestaw narzędzi do obróbki obrazu
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "analiza_obrazu.h"
#include "stdlib.h"


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
void KonwersjaCB7doRGB565(uint8_t *obrazCB, uint16_t *obrazCB565, uint32_t rozmiar)
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
// [we] obraz* - wskaźnik na bufor[rozmiar] z obrazem czarno-białym
// [wy] hist - 129 elementowy, 8-bitowy histogram. Ostatni element jest dla liczb spoza zakresu
// [we] rozmiar - ilość pikseli do analizy
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HistogramCB7(uint8_t *obraz, uint8_t *hist, uint32_t rozmiar)
{
	uint32_t histogram[129], temp;
	uint8_t pix;

	for (uint8_t n=0; n<129; n++)
		histogram[n] = 0;

	for (uint32_t n=0; n<rozmiar; n++)
	{
		pix = *(obraz+n);
		if (pix > 128)
			pix = 128;	//ostatnia pozycja zbiera śmieci spoza zakresu 7-bit
		histogram[pix]++;
	}

	for (uint8_t n=0; n<129; n++)
	{
		temp = (uint8_t)(histogram[n] / DZIELNIK_HIST_CB);
		if (temp > 255)
			temp = 255;
		*(hist+n) = temp;
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
	uint32_t nSumaR[32], nSumaG[32], nSumaB[32], temp;

	//inicjuj sumę pokseli w danym kolorze, później będzie inkrementowana, wiec musi zaczynać się od zera
	for (uint8_t n=0; n<32; n++)
	{
		nSumaR[n] = 0;
		nSumaG[n] = 0;
		nSumaG[n+32] = 0;
		nSumaB[n] = 0;
	}

	for (uint32_t n=0; n<rozmiar; n++)
	{
		sPixel = *(obrazRGB565 + n);
		nSumaR[(sPixel & 0xF800) >> 11]++;
		nSumaG[(sPixel & 0x07C0) >> 6]++;	//ogranicz rozdzielczość zielonego do 5 bitów
		nSumaB[(sPixel & 0x001F)]++;
	}

	for (uint8_t n=0; n<32; n++)
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

