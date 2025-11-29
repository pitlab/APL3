//////////////////////////////////////////////////////////////////////////////
//
// Obsługa zapisu obrazu w formacie bmp
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "bmp.h"
#include "kamera.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include "display.h"
#include "rejestrator.h"

FIL SDBmpFile;       //struktura pliku z obrazem
//extern char chNapis[100];
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern char __attribute__ ((aligned (32))) chBufPodreczny[40];
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[];
extern stKonfKam_t stKonfKam;
extern uint8_t chNazwaPlikuObrazu[DLG_NAZWY_PLIKU_OBR];	//początek nazwy pliku z obrazem, po tym jest data i czas
extern volatile uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
//uint8_t chBuforWiersza[DISP_X_SIZE * 3 * 2];	//bufor na 2 wiersze

////////////////////////////////////////////////////////////////////////////////
// zapisuje obraz z kamery do pliku bmp. Ponieważ jest wywoływana w wątku rejestratora, jest bezparametrowa
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaZapisuBmp(void)
{
	if (stKonfKam.chFormatObrazu == OBR_Y8)
	{
		ZapiszPlikBmp((uint8_t*)sBuforKamerySRAM, BMP_KOLOR_8, (uint16_t)stKonfKam.chSzerWy * KROK_ROZDZ_KAM, (uint16_t)stKonfKam.chWysWy * KROK_ROZDZ_KAM);	//monochromatyczny
	}
	else
	{
		ZapiszPlikBmp((uint8_t*)sBuforKamerySRAM, BMP_KOLOR_24, (uint16_t)stKonfKam.chSzerWy * KROK_ROZDZ_KAM, (uint16_t)stKonfKam.chWysWy * KROK_ROZDZ_KAM);	//kolorowy
	}
	chStatusRejestratora &= ~STATREJ_ZAPISZ_BMP;	//wyłącz zapis
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
	uint8_t chNaglowek[ROZMIAR_NAGLOWKA_BMP] = {'B','M'};
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

	chNaglowek[2] = (uint8_t)(nRozmiarPliku);
	chNaglowek[3] = (uint8_t)(nRozmiarPliku >> 8);
	chNaglowek[4] = (uint8_t)(nRozmiarPliku >> 16);
	chNaglowek[5] = (uint8_t)(nRozmiarPliku >> 24);
	chNaglowek[6] = 0;	//reserved
	chNaglowek[7] = 0;
	chNaglowek[8] = 0;	//reserved
	chNaglowek[9] = 0;

	//ustal rozmiar offsety danych. Dla trybu 8 bitowego za nagłówkiem jest paleta kolorów
	if (chFormatKoloru == BMP_KOLOR_8)
		nOffsetWiersza = ROZMIAR_NAGLOWKA_BMP + ROZMIAR_PALETY_BMP;	//tryb z paletą 8-bitową
	else
		nOffsetWiersza = ROZMIAR_NAGLOWKA_BMP;						//tryby z kolorem 24-bitowym

	chNaglowek[10] = (uint8_t)(nOffsetWiersza);
	chNaglowek[11] = (uint8_t)(nOffsetWiersza >> 8);
	chNaglowek[12] = (uint8_t)(nOffsetWiersza >> 16);
	chNaglowek[13] = (uint8_t)(nOffsetWiersza >> 24);

	// Nagłówek DIB (BITMAPINFOHEADER)
	chNaglowek[14] = 50; 							// rozmiar nagłówka, offset na początek obrazu lub palety
	chNaglowek[15] = 0;
	chNaglowek[16] = 0;
	chNaglowek[17] = 0;
	chNaglowek[18] = (uint8_t)(sSzerokosc);
	chNaglowek[19] = (uint8_t)(sSzerokosc >> 8);
	chNaglowek[20] = (uint8_t)(sSzerokosc >> 16);
	chNaglowek[21] = (uint8_t)(sSzerokosc >> 24);
	chNaglowek[22] = (uint8_t)(sWysokosc);
	chNaglowek[23] = (uint8_t)(sWysokosc >> 8);
	chNaglowek[24] = (uint8_t)(sWysokosc >> 16);
	chNaglowek[25] = (uint8_t)(sWysokosc >> 24);
	chNaglowek[26] = 1; 							// liczba płaszczyzn
	chNaglowek[27] = 0;
	chNaglowek[28] = chFormatKoloru; 				//liczba bitów na piksel: 8 lub 24 BGR
	chNaglowek[29] = 0;								//algorytm kompresji: 0=brak
	chNaglowek[30] = 0;
	chNaglowek[31] = 0;
	chNaglowek[32] = 0;
	chNaglowek[34] = (uint8_t)(nRozmiarDanych);		//rozmiar rysunku
	chNaglowek[35] = (uint8_t)(nRozmiarDanych >> 8);
	chNaglowek[36] = (uint8_t)(nRozmiarDanych >> 16);
	chNaglowek[37] = (uint8_t)(nRozmiarDanych >> 24);
	//biXPelsPerMeter - rozdzielczość w pixelach na metr dla osi OX;
	//biYPelsPerMeter - analogicznie j.w. dla osi OY,
	//biClrUsed - ilość kolorów która faktycznie została użyta,
	//biClrImportant - ilość kolorów znaczących.
	//biClrRotation - flaga rotacji palety
	for (uint8_t n=38; n<ROZMIAR_NAGLOWKA_BMP; n++)
		chNaglowek[n] = 0;

	chNaglowek[56] = 'A';
	chNaglowek[57] = 'P';
	chNaglowek[58] = 'L';
	chNaglowek[59] = 'v';
	chNaglowek[60] = '0' + WER_GLOWNA;
	chNaglowek[61] = '.';
	chNaglowek[62] = '0' + WER_PODRZ;
	chNaglowek[63] = 0;

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(chBufPodreczny, "%s_%04d%02d%02d_%02d%02d%02d.bmp", chNazwaPlikuObrazu, sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

	fres = f_open(&SDBmpFile, chBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
	if (fres != FR_OK)
		return (uint8_t)fres;

	//zapisz nagłówek do pliku
	fres = f_write(&SDBmpFile, chNaglowek, ROZMIAR_NAGLOWKA_BMP, &nZapisanoBajtow);
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
				chNaglowek[n * 4 + 0] = n + m * 8; // Blue
				chNaglowek[n * 4 + 1] = n + m * 8; // Green
				chNaglowek[n * 4 + 2] = n + m * 8; // Red
				chNaglowek[n * 4 + 3] = 0; // Alfa
			}
			fres = f_write(&SDBmpFile, chNaglowek, 8*4, &nZapisanoBajtow);
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

	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(chBufPodreczny, "dane_%04d%02d%02d_%02d%02d%02d.bin", sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

	fres = f_open(&SDBmpFile, chBufPodreczny, FA_CREATE_ALWAYS | FA_WRITE);
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
