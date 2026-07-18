//////////////////////////////////////////////////////////////////////////////
//
// Obsługa zapisu obrazu w formacie bmp
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Bmp.h>
#include <Czas.h>
#include <Kamera.h>
#include <Rejestrator.h>
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include "Ekran.h"

FIL SDBmpFile;       //struktura pliku z obrazem
extern RTC_TimeTypeDef stTime;
extern RTC_DateTypeDef stDate;
extern char __attribute__ ((aligned (32))) cBufPodreczny[40];
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) sBuforKamery[SZER_ZDJECIA * WYS_ZDJECIA];
extern stKonfKam_t stKonfKam;
extern uint8_t cNazwaPlikuObrazu[DLG_NAZWY_PLIKU_OBR];	//początek nazwy pliku z obrazem, po tym jest data i czas
extern volatile uint8_t cStatusRejestratora;	//zestaw flag informujących o stanie rejestratora



////////////////////////////////////////////////////////////////////////////////
// zapisuje obraz z kamery do pliku bmp. Ponieważ jest wywoływana w wątku rejestratora, jest bezparametrowa
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaZapisuBmp(void)
{
	if (stKonfKam.chFormatObrazu == OBR_Y8)
	{
		ZapiszPlikBmp((uint8_t*)sBuforKamery, BMP_KOLOR_8, (uint16_t)stKonfKam.chSzerWy * KROK_ROZDZ_KAM, (uint16_t)stKonfKam.chWysWy * KROK_ROZDZ_KAM);	//monochromatyczny
	}
	else
	{
		ZapiszPlikBmp((uint8_t*)sBuforKamery, BMP_KOLOR_24, (uint16_t)stKonfKam.chSzerWy * KROK_ROZDZ_KAM, (uint16_t)stKonfKam.chWysWy * KROK_ROZDZ_KAM);	//kolorowy
	}
	cStatusRejestratora &= ~STATREJ_ZAPISZ_BMP;	//wyłącz zapis
}



////////////////////////////////////////////////////////////////////////////////
// zapisuje monochromatyczny obraz do pliku bmp
// Parametry: *chObrazWe - wskaźnik na dane obrazu
//	chFormatKoloru - określa ilość bitów koloru piksela: 8 lub 24
//	sSzerokosc, sWysokosc - rozmiary obrazu będące wielokrotnością 4
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszPlikBmp(uint8_t *chObrazWe, uint8_t chFormatKoloru, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	FRESULT fres = 0;
	UINT nZapisanoBajtow;
	uint8_t cNaglowek[ROZMIAR_NAGLOWKA_BMP] = {'B','M'};
	uint32_t nRozmiarDanych;
	uint32_t nRozmiarPliku;
	uint32_t nOffsetWiersza, nOffsetPiksela;
	//uint16_t s3x;
	uint8_t r,g,b;

	if (chFormatKoloru == BMP_KOLOR_8)
	{
		nRozmiarDanych = sSzerokosc * sWysokosc;
		nRozmiarPliku = ROZMIAR_NAGLOWKA_BMP + nRozmiarDanych + ROZMIAR_PALETY_BMP;
	}
	else
	{
		nRozmiarDanych = sSzerokosc * sWysokosc * BAJTOW_KOLORU;
		nRozmiarPliku = ROZMIAR_NAGLOWKA_BMP + nRozmiarDanych;
	}

	cNaglowek[2] = (uint8_t)(nRozmiarPliku);
	cNaglowek[3] = (uint8_t)(nRozmiarPliku >> 8);
	cNaglowek[4] = (uint8_t)(nRozmiarPliku >> 16);
	cNaglowek[5] = (uint8_t)(nRozmiarPliku >> 24);
	cNaglowek[6] = 0;	//reserved
	cNaglowek[7] = 0;
	cNaglowek[8] = 0;	//reserved
	cNaglowek[9] = 0;

	//ustal rozmiar offsety danych. Dla trybu 8 bitowego za nagłówkiem jest paleta kolorów
	if (chFormatKoloru == BMP_KOLOR_8)
		nOffsetWiersza = ROZMIAR_NAGLOWKA_BMP + ROZMIAR_PALETY_BMP;	//tryb z paletą 8-bitową
	else
		nOffsetWiersza = ROZMIAR_NAGLOWKA_BMP;						//tryby z kolorem 24-bitowym

	cNaglowek[10] = (uint8_t)(nOffsetWiersza);
	cNaglowek[11] = (uint8_t)(nOffsetWiersza >> 8);
	cNaglowek[12] = (uint8_t)(nOffsetWiersza >> 16);
	cNaglowek[13] = (uint8_t)(nOffsetWiersza >> 24);

	// Nagłówek DIB (BITMAPINFOHEADER)
	cNaglowek[14] = 50; 							// rozmiar nagłówka, offset na początek obrazu lub palety
	cNaglowek[15] = 0;
	cNaglowek[16] = 0;
	cNaglowek[17] = 0;
	cNaglowek[18] = (uint8_t)(sSzerokosc);
	cNaglowek[19] = (uint8_t)(sSzerokosc >> 8);
	cNaglowek[20] = (uint8_t)(sSzerokosc >> 16);
	cNaglowek[21] = (uint8_t)(sSzerokosc >> 24);
	cNaglowek[22] = (uint8_t)(sWysokosc);
	cNaglowek[23] = (uint8_t)(sWysokosc >> 8);
	cNaglowek[24] = (uint8_t)(sWysokosc >> 16);
	cNaglowek[25] = (uint8_t)(sWysokosc >> 24);
	cNaglowek[26] = 1; 							// liczba płaszczyzn
	cNaglowek[27] = 0;
	cNaglowek[28] = chFormatKoloru; 				//liczba bitów na piksel: 8 lub 24 BGR
	cNaglowek[29] = 0;								//algorytm kompresji: 0=brak
	cNaglowek[30] = 0;
	cNaglowek[31] = 0;
	cNaglowek[32] = 0;
	cNaglowek[34] = (uint8_t)(nRozmiarDanych);		//rozmiar rysunku
	cNaglowek[35] = (uint8_t)(nRozmiarDanych >> 8);
	cNaglowek[36] = (uint8_t)(nRozmiarDanych >> 16);
	cNaglowek[37] = (uint8_t)(nRozmiarDanych >> 24);
	//biXPelsPerMeter - rozdzielczość w pixelach na metr dla osi OX;
	//biYPelsPerMeter - analogicznie j.w. dla osi OY,
	//biClrUsed - ilość kolorów która faktycznie została użyta,
	//biClrImportant - ilość kolorów znaczących.
	//biClrRotation - flaga rotacji palety
	for (uint8_t n=38; n<ROZMIAR_NAGLOWKA_BMP; n++)
		cNaglowek[n] = 0;

	cNaglowek[56] = 'A';
	cNaglowek[57] = 'P';
	cNaglowek[58] = 'L';
	cNaglowek[59] = 'v';
	cNaglowek[60] = '0' + WER_GLOWNA;
	cNaglowek[61] = '.';
	cNaglowek[62] = '0' + WER_PODRZ;
	cNaglowek[63] = 0;

	PobierzDateCzas(&stDate, &stTime);
	sprintf(cBufPodreczny, "%s_%04d%02d%02d_%02d%02d%02d.bmp", cNazwaPlikuObrazu, stDate.Year+2000, stDate.Month, stDate.Date, stTime.Hours, stTime.Minutes, stTime.Seconds);

	fres = f_open(&SDBmpFile, cBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
	if (fres != FR_OK)
		return (uint8_t)fres;

	//zapisz nagłówek do pliku
	fres = f_write(&SDBmpFile, cNaglowek, ROZMIAR_NAGLOWKA_BMP, &nZapisanoBajtow);
	if (fres != FR_OK)
	{
		f_close(&SDBmpFile);
		return (uint8_t)fres;
	}

	if (chFormatKoloru == BMP_KOLOR_8)
	{
		//zapisz paletę (256 kolorów od czarnego do białego). Aby nie powiekszać stosu tworząc dużą strukturę palety kolorów (256*4 bajty), zapisuję ją sekwencyjnie 32x32 bajty
		for (uint8_t m=0; m<32; m++)
		{
			for (uint8_t n=0; n<8; n++)
			{
				cNaglowek[n * 4 + 0] = n + m * 8; // Blue
				cNaglowek[n * 4 + 1] = n + m * 8; // Green
				cNaglowek[n * 4 + 2] = n + m * 8; // Red
				cNaglowek[n * 4 + 3] = 0; // Alfa
			}
			fres = f_write(&SDBmpFile, cNaglowek, 8*4, &nZapisanoBajtow);
			if (fres != FR_OK)
			{
				f_close(&SDBmpFile);
				return (uint8_t)fres;
			}
		}
	}

	__set_PRIMASK(1);   // wyłącza wszystkie przerwania
	uint8_t *chBuforWiersza;		//tworzę zmienną dynamiczną pobierającą pamieć ze sterty zamiast ze stosu
	if (chFormatKoloru == BMP_KOLOR_8)
		chBuforWiersza = (uint8_t*)malloc(sSzerokosc * BAJTOW_SZAROSCI * WIERSZY_BMP);
	else
		chBuforWiersza = (uint8_t*)malloc(sSzerokosc * BAJTOW_KOLORU * WIERSZY_BMP);
	__set_PRIMASK(0);   // włącza wszystkie przerwania

	//teraz przepisz obraz wierszami od dołu do góry
    for (int32_t y=(sWysokosc/WIERSZY_BMP)-1; y>=0; y--)	//przy liczeniu w dół poniżej 0 zmienna musi być ze znakiem!
    {
    	if (chFormatKoloru == BMP_KOLOR_8)
    	{
    		fres = f_write(&SDBmpFile, &chObrazWe[y * sSzerokosc * WIERSZY_BMP], sSzerokosc * WIERSZY_BMP, &nZapisanoBajtow);	//zapisz prosto z obrazu
    		if ((fres != FR_OK) || (nZapisanoBajtow != sSzerokosc * WIERSZY_BMP))
			{
    			__set_PRIMASK(1);   // wyłącza wszystkie przerwania
				free(chBuforWiersza);
				__set_PRIMASK(0);   // włącza wszystkie przerwania
				f_close(&SDBmpFile);
				return (uint8_t)fres;
			}
    	}

		if (chFormatKoloru == BMP_KOLOR_24)	//obraz kolorowy
    	{
			nOffsetWiersza = y * sSzerokosc * BAJTOW_KOLORU * WIERSZY_BMP;
			//for (int16_t w = WIERSZY_BMP-1; w>=0; w--)		//przy liczeniu w dół poniżej 0 zmienna musi być ze znakiem!
			//for (int16_t w=0; w<WIERSZY_BMP; w++)
			//{

				for (uint32_t x = 0; x < sSzerokosc; x++)
				{
					if (x == 119)
						fres = 0;

					nOffsetPiksela = BAJTOW_KOLORU * x;
					// Konwersja RGB → BGR (BMP wymaga odwrotnej kolejności)
					/*chBuforWiersza[w * sSzerokosc + nOffsetPiksela + 0] = chObrazWe[(WIERSZY_BMP - 1 - w) * sSzerokosc + nOffsetWiersza + nOffsetPiksela + 2]; // Blue
					chBuforWiersza[w * sSzerokosc + nOffsetPiksela + 1] = chObrazWe[(WIERSZY_BMP - 1 - w) * sSzerokosc + nOffsetWiersza + nOffsetPiksela + 1]; // Green
					chBuforWiersza[w * sSzerokosc + nOffsetPiksela + 2] = chObrazWe[(WIERSZY_BMP - 1 - w) * sSzerokosc + nOffsetWiersza + nOffsetPiksela + 0]; // Red*/
					/*chBuforWiersza[nOffsetPiksela + 0] = chObrazWe[nOffsetWiersza + nOffsetPiksela + 2]; // Blue
					chBuforWiersza[nOffsetPiksela + 1] = chObrazWe[nOffsetWiersza + nOffsetPiksela + 1]; // Green
					chBuforWiersza[nOffsetPiksela + 2] = chObrazWe[nOffsetWiersza + nOffsetPiksela + 0]; // Red*/
					r = chObrazWe[nOffsetWiersza + nOffsetPiksela + 0];
					g = chObrazWe[nOffsetWiersza + nOffsetPiksela + 1];
					b = chObrazWe[nOffsetWiersza + nOffsetPiksela + 2];
					chBuforWiersza[nOffsetPiksela + 0] = b;
					chBuforWiersza[nOffsetPiksela + 1] = g;
					chBuforWiersza[nOffsetPiksela + 2] = r;
				}
			//}
			//liczba pikseli w wierszu musi być podzielna przez 4 - ze wzgledu na szerokość obrazu podzielną przez 16 jest to zawsze spełnione
			fres = f_write(&SDBmpFile, chBuforWiersza, sSzerokosc * 3 * WIERSZY_BMP, &nZapisanoBajtow);
			if ((fres != FR_OK) || (nZapisanoBajtow != sSzerokosc * 3 * WIERSZY_BMP))
			{
				__set_PRIMASK(1);   // wyłącza wszystkie przerwania
				free(chBuforWiersza);
				__set_PRIMASK(0);   // włącza wszystkie przerwania
				f_close(&SDBmpFile);
				return (uint8_t)fres;
			}
    	}
		/*else
		if (chFormatKoloru == OBR_YUV444)
		{
			// Konwersja YUV → BGR
			//uint32_t nY, nU, nV, nR, nG, nB;
			for (uint16_t x = 0; x < sSzerokosc; x++)
			{
				s3x = 3 * x;
				nY = (int32_t)chObrazWe[nOffsetWiersza + s3x + 0];
				nU = (int32_t)chObrazWe[nOffsetWiersza + s3x + 1] - 128;
				nV = (int32_t)chObrazWe[nOffsetWiersza + s3x + 2] - 128;

				// Konwersja wg ITU-R BT.601
				//nR = nY + (int32_t)(1.402f * nV);
				//nG = nY - (int32_t)(0.344136f * nU + 0.714136f * nV);
				//nB = nY + (int32_t)(1.772f * nU);
				//optymalizacja bez używania float
				nR = nY + ((1436 * nV) >> 10);
				nG = nY - ((352 * nU + 731 * nV) >> 10);
				nB = nY + ((1815 * nU) >> 10);

				// Ogranicz zakres do [0..255]
				if (nR < 0) nR = 0; else if (nR > 255) nR = 255;
				if (nG < 0) nG = 0; else if (nG > 255) nG = 255;
				if (nB < 0) nB = 0; else if (nB > 255) nB = 255;

				chBuforWiersza[s3x + 0] = (uint8_t)nB; // Blue
				chBuforWiersza[s3x + 1] = (uint8_t)nG; // Green
				chBuforWiersza[s3x + 2] = (uint8_t)nR; // Red
			}
			fres = f_write(&SDBmpFile, chBuforWiersza, sSzerokosc, &nZapisanoBajtow);
		}*/
    }
    __set_PRIMASK(1);   // wyłącza wszystkie przerwania
    free(chBuforWiersza);
    __set_PRIMASK(0);   // włącza wszystkie przerwania
	fres = f_close(&SDBmpFile);
	return (uint8_t)fres;
}



////////////////////////////////////////////////////////////////////////////////
// zapisuje surowe dane do pliku bin
// Parametry: *chDaneWe - wskaźnik na dane obrazu
//	sRozmiar- rozmiar danych
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszPlikBin(uint8_t *chDaneWe, uint32_t nRozmiar)
{
	FRESULT fres = 0;
	UINT nZapisanoBajtow;

	PobierzDateCzas(&stDate, &stTime);
	sprintf(cBufPodreczny, "dane_%04d%02d%02d_%02d%02d%02d.bin", stDate.Year+2000, stDate.Month, stDate.Date, stTime.Hours, stTime.Minutes, stTime.Seconds);

	fres = f_open(&SDBmpFile, cBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
	if (fres != FR_OK)
		return (uint8_t)fres;

	//teraz przepisz dane
	fres = f_write(&SDBmpFile, chDaneWe, nRozmiar, &nZapisanoBajtow);
	if ((fres != FR_OK) || (nZapisanoBajtow != nRozmiar))
	{
		f_close(&SDBmpFile);
		return (uint8_t)fres;
	}

	fres = f_close(&SDBmpFile);
	return (uint8_t)fres;
}
