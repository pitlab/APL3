//////////////////////////////////////////////////////////////////////////////
//
// Obsługa zapisu obrazu w formacie bmp
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include"bmp.h"
#include "kamera.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>

FIL SDBmpFile;       //struktura pliku z obrazem
//extern char chNapis[100];
extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern char __attribute__ ((aligned (32))) chBufPodreczny[40];
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[];
extern struct st_KonfKam stKonfKam;



////////////////////////////////////////////////////////////////////////////////
// zapisuje obraz z kamery do pliku bmp. Ponieważ jest wywoływana w wątku rejestratora, jest bezparametrowa
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void ObslugaZapisuBmp(void)
{
	uint8_t chFormatKoloru;

	if (stKonfKam.chFormatObrazu == OBR_Y8)
		chFormatKoloru = BMP_KOLOR_Y8;
	else
	if (stKonfKam.chFormatObrazu == OBR_RGB565)
		chFormatKoloru = BMP_KOLOR_RGB24;
	else
		return;	//nieobsługiwany format koloru

	ZapiszPlikBmp((uint8_t*)sBuforKamerySRAM, chFormatKoloru, (uint16_t)stKonfKam.chSzerWy * KROK_ROZDZ_KAM, (uint16_t)stKonfKam.chWysWy * KROK_ROZDZ_KAM);
}



////////////////////////////////////////////////////////////////////////////////
// zapisuje monochromatyczny obraz do pliku bmp
// Parametry: *chObrazWe - wskaźnik na dane obrazu
//  sSzerokosc, sWysokosc - rozmiary obrazu będące wielokrotnością 4
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t ZapiszPlikBmp(uint8_t *chObrazWe, uint8_t chFormatKoloru, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	FRESULT fres = 0;
	UINT nZapisanoBajtow;
	uint8_t chNaglowek[ROZMIAR_NAGLOWKA_BMP] = {'B','M'};
	uint32_t nRozmiarDanych = sSzerokosc * sWysokosc;
	uint32_t nRozmiarPliku = ROZMIAR_NAGLOWKA_BMP + nRozmiarDanych;
	uint32_t nOffsetWiersza;
	uint16_t s3x;

	chNaglowek[2] = (uint8_t)(nRozmiarPliku);
	chNaglowek[3] = (uint8_t)(nRozmiarPliku >> 8);
	chNaglowek[4] = (uint8_t)(nRozmiarPliku >> 16);
	chNaglowek[5] = (uint8_t)(nRozmiarPliku >> 24);

	chNaglowek[10] = (uint8_t)(OFFSET_DANYCH_BMP);
	chNaglowek[11] = (uint8_t)(OFFSET_DANYCH_BMP >> 8);
	chNaglowek[12] = (uint8_t)(OFFSET_DANYCH_BMP >> 16);
	chNaglowek[13] = (uint8_t)(OFFSET_DANYCH_BMP >> 24);

	// Nagłówek DIB (BITMAPINFOHEADER)
	chNaglowek[14] = 40; // rozmiar DIB header
	chNaglowek[18] = (uint8_t)(sSzerokosc);
	chNaglowek[19] = (uint8_t)(sSzerokosc >> 8);
	chNaglowek[20] = (uint8_t)(sSzerokosc >> 16);
	chNaglowek[21] = (uint8_t)(sSzerokosc >> 24);
	chNaglowek[22] = (uint8_t)(sWysokosc);
	chNaglowek[23] = (uint8_t)(sWysokosc >> 8);
	chNaglowek[24] = (uint8_t)(sWysokosc >> 16);
	chNaglowek[25] = (uint8_t)(sWysokosc >> 24);
	chNaglowek[26] = 1; // liczba płaszczyzn
	chNaglowek[28] = chFormatKoloru; // 8-bitowy obraz (256 kolorów) lub 24-bity BGR
	chNaglowek[34] = (uint8_t)(nRozmiarDanych);
	chNaglowek[35] = (uint8_t)(nRozmiarDanych >> 8);
	chNaglowek[36] = (uint8_t)(nRozmiarDanych >> 16);
	chNaglowek[37] = (uint8_t)(nRozmiarDanych >> 24);


	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	sprintf(chBufPodreczny, "zdj_%04d%02d%02d_%02d%02d%02d.bmp", sDate.Year+2000, sDate.Month, sDate.Date, sTime.Hours, sTime.Minutes, sTime.Seconds);

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

	if (chFormatKoloru == BMP_KOLOR_Y8)
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

	//teraz przepisz obraz wierszami od dołu do góry
	//uint8_t chBuforWiersza[sSzerokosc * 3];	//rozmiar potrzebny do obsługi koloru
	uint8_t *chBuforWiersza = NULL;
		chBuforWiersza = (uint8_t*)malloc(sSzerokosc * 3);	//tworzę zmienną dynamiczną pobierającą pamieć ze sterty zamiast ze stosu

    for (int16_t y = sWysokosc - 1; y >= 0; y--)	//zmienna musi być ze znakiem!
    {
    	nOffsetWiersza = y * sSzerokosc * 3;
    	if (chFormatKoloru == BMP_KOLOR_Y8)
    	{
    		//memcpy(chBuforWiersza, &chObrazWe[y * sSzerokosc], sSzerokosc);
    		//fres = f_write(&SDBmpFile, chBuforWiersza, sSzerokosc, &nZapisanoBajtow);

    		fres = f_write(&SDBmpFile, &chObrazWe[y * sSzerokosc], sSzerokosc, &nZapisanoBajtow);	//zapisz prosto z obrazu
    	}
    	else	//obraz kolorowy
    	{
    		// Konwersja RGB → BGR (BMP wymaga odwrotnej kolejności)
			for (uint16_t x = 0; x < sSzerokosc; x++)
			{
				s3x = 3 * x;
				chBuforWiersza[s3x + 0] = chObrazWe[nOffsetWiersza + s3x + 2]; // Blue
				chBuforWiersza[s3x + 1] = chObrazWe[nOffsetWiersza + s3x + 1]; // Green
				chBuforWiersza[s3x + 2] = chObrazWe[nOffsetWiersza + s3x + 0]; // Red
			}

			fres = f_write(&SDBmpFile, chBuforWiersza, sSzerokosc, &nZapisanoBajtow);
    	}
		if (fres != FR_OK || nZapisanoBajtow != sSzerokosc)
		{
			if (fres != FR_OK)
			{
				free(chBuforWiersza);
				f_close(&SDBmpFile);
				return (uint8_t)fres;
			}
		}
    }
    free(chBuforWiersza);
	fres = f_close(&SDBmpFile);
	return (uint8_t)fres;
}
