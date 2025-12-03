//////////////////////////////////////////////////////////////////////////////
//
// Obsługa bloku danych informacyjnych EXIF w pliku JPEG
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "exif.h"
#include "napisy.h"

uint8_t chNaglJpegExif[ROZMIAR_EXIF];
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern const char *chNapisLcd[MAX_NAPISOW];


////////////////////////////////////////////////////////////////////////////////
// Buduje strukturę tagu w Exif. Rozmiar TAG-a to 12 bajtów
// Pierwsze 2 bajty to identyfikator, koleje 2 to typ danych, kolejne 4 bajty to rozmiar, kolejne 4 to offset danych lub bezpośrednio dane
// Jeżeli podany rozmiar jest zerem to zamiast offsetu podaj dane
// Parametry:
// [wy] *nTag - wskaźnik wskaźnik na budowany TAG
// [we] sID - identyfikator TAG-u
// [we] sTyp - typ danych
// [we] *chDane wskaźnik na dane
// [we] chRozmiar - rozmiar danych wyrażony w bajtach
// [we] nOffset - offset do danych począwszy od nagłówka TIFF
// Zwraca: rozmiar struktury
////////////////////////////////////////////////////////////////////////////////
void PrzygotujTag(uint8_t **chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint32_t nRozmiar, uint8_t **chWskDanych, uint8_t *chPoczatekTIFF)
{
	uint32_t nOffset = *chWskDanych - chPoczatekTIFF;
	uint8_t chRozmiarTagu;

	//oblicz jednostkowy rozmiar tagu dla różnych typów danych
	switch (sTyp)
	{
	case EXIF_TYPE_BYTE:
	case EXIF_TYPE_ASCII:		chRozmiarTagu = nRozmiar;		break;	//ASCII
	case EXIF_TYPE_SHORT:		chRozmiarTagu = nRozmiar / 2;	break;	//SHORT
	case EXIF_TYPE_LONG:				//LONG 32-bit
	case EXIF_TYPE_SLONG:		chRozmiarTagu = nRozmiar / 4;	break;	//Signed LONG 32-bit
	case EXIF_TYPE_RATIONAL:											//RATIONAL: 2x LONG. Pierwszy numerator, drugi denominator
	case EXIF_TYPE_SRATIONAL: 	chRozmiarTagu = nRozmiar / 8;	break;	//Signed RATIONAL: 2x SLONG
	}

	//dla tagów o podanej zerowej długosci przyjmij rozmiar 1
	if (chRozmiarTagu == 0)
		chRozmiarTagu = 1;

	*(*chWskTaga +  0) = (uint8_t)(sTagID);
	*(*chWskTaga +  1) = (uint8_t)(sTagID >> 8);
	*(*chWskTaga +  2) = (uint8_t)(sTyp);
	*(*chWskTaga +  3) = (uint8_t)(sTyp >> 8);
	*(*chWskTaga +  4) = chRozmiarTagu;
	*(*chWskTaga +  5) = 0;
	*(*chWskTaga +  6) = 0;
	*(*chWskTaga +  7) = 0;
	if (nRozmiar > 4)	//jeżeli rozmiar nie zmieści się w strukturze to wstaw offset do segmentu danych, jeżeli zmieści, to wstaw 4 bajty danych zamiast offsetu
	{
		*(*chWskTaga +  8) = (uint8_t)(nOffset);			//Offset od początku nagłówka TIFF do miejsca gdzie dane są zapisane
		*(*chWskTaga +  9) = (uint8_t)(nOffset >>  8);
		*(*chWskTaga + 10) = (uint8_t)(nOffset >> 16);
		*(*chWskTaga + 11) = (uint8_t)(nOffset >> 24);

		for (uint32_t n=0; n<nRozmiar; n++)
			*(*chWskDanych + n) = *(chDane + n);
		*(*chWskDanych + nRozmiar) = 0;
		nRozmiar++;		//powiększ rozmiar o zero terminujące wartość
		*chWskDanych += nRozmiar;
	}
	else	//jeżeli rozmiar zmieści sie w 4 bajtach to go wstaw zamiast offsetu
	{
		for (uint8_t n=0; n<nRozmiar; n++)
			*(*chWskTaga + n + 8) = *(chDane + n);
	}
	*chWskTaga += ROZMIAR_TAGU;	//wskaż na adres następnego tagu
}



////////////////////////////////////////////////////////////////////////////////
// Buduje strukturę bloku Exif do umieszczenia w pliku jpeg
// Pierwszych 18 bajtów jest już ustawionych, trzeba wstawić kolejne tagi
// Parametry:
// [we] *stKonf - wskaźnik na konfigurację kamery
// [we] *stDane - wskaźnik na dane autopilota
// Zwraca: rozmiar struktury
////////////////////////////////////////////////////////////////////////////////
uint32_t PrzygotujExif(stKonfKam_t *stKonf, volatile stWymianyCM4_t *stDane, RTC_DateTypeDef stData, RTC_TimeTypeDef stCzas)
{
	uint32_t nRozmiar, nOffset;
	uint8_t chBufor[25];
	uint8_t *wskchAdresTAG;
	uint8_t *wskchAdresDanych;
	uint8_t *wskchAdresExif, *wskchAdresGPS;
	uint8_t *wskchPoczatekTIFF = &chNaglJpegExif[12];	//początek TIFF
	float fTemp1, fTemp2;

	chNaglJpegExif[0] = 0xFF;	//SOI
	chNaglJpegExif[1] = 0xD8;
	chNaglJpegExif[2] = 0xFF;	//APP1
	chNaglJpegExif[3] = 0xE1;
	chNaglJpegExif[4] = 0x00;	// Rozmiar (192 bajty)
	chNaglJpegExif[5] = 0xC0;
	chNaglJpegExif[6] = 'E';
	chNaglJpegExif[7] = 'x';
	chNaglJpegExif[8] = 'i';
	chNaglJpegExif[9] = 'f';
	chNaglJpegExif[10] = 0x00;
	chNaglJpegExif[11] = 0x00;

	//TIFF header (little endian)
	chNaglJpegExif[12] = 'I';		//"II" = Intel - określa porządek bajtów
	chNaglJpegExif[13] = 'I';
	chNaglJpegExif[14] = 0x2A;		//0x2A00 = magic
	chNaglJpegExif[15] = 0x00;
	chNaglJpegExif[16] = 0x08;		// offset do IFD0
	chNaglJpegExif[17] = 0x00;
	chNaglJpegExif[18] = 0x00;
	chNaglJpegExif[19] = 0x00;

	//IFD0: Liczba tagów - Interoperability number
	chNaglJpegExif[20] = (uint8_t)LICZBA_TAGOW_IFD0;
	chNaglJpegExif[21] = (uint8_t)(LICZBA_TAGOW_IFD0 >> 8);

	wskchAdresTAG = (uint8_t*)chNaglJpegExif + 22;	//adres miejsca gdzie zapisać pierwszy TAG
	wskchAdresDanych = wskchAdresTAG + (LICZBA_TAGOW_IFD0 * ROZMIAR_TAGU) + 4;	//adres za grupą tagów gdzie zapisać dane + wskaźnik 4 bajty do IFD1

	//TAG-i IFD0 ***********************************************************************************************************************************
	switch (stKonf->chFormatObrazu)
	{
	case OBR_RGB565:nRozmiar = sprintf((char*)chBufor, "%s %s YUV422 ", chNapisLcd[STR_EXIF], chNapisLcd[STR_JPEG]);	break;	//obraz kolorowy
	case OBR_Y8:	nRozmiar = sprintf((char*)chBufor, "%s %s Y8 ", chNapisLcd[STR_EXIF], chNapisLcd[STR_JPEG]);	break;		//obraz monochromatyczny
	default:		nRozmiar = sprintf((char*)chBufor, "nieznany %s ", chNapisLcd[STR_JPEG]);	break;
	}
	PrzygotujTag(&wskchAdresTAG, EXTAG_IMAGE_DESCRIPTION, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "%s ", chNapisLcd[STR_PITLAB]);
	PrzygotujTag(&wskchAdresTAG, EXTAG_IMAGE_INPUT_MAKE, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "APL hv%d.%d ",WER_GLOWNA,  WER_PODRZ);
	PrzygotujTag(&wskchAdresTAG, EXTAG_EQUIPMENT_MODEL, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "APL sv%d ", WER_REPO);
	PrzygotujTag(&wskchAdresTAG, EXTAG_SOFTWARE, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "Piotr Laskowski ");	//dodać: artystę zapisać gdzieś w osobnej strukturze
	PrzygotujTag(&wskchAdresTAG, EXTAG_ARTIST, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	chBufor[0] = 1;	//punkt (0,0) jest: 1=LGR, 2PGR, 3=DGR, 4=DLR,
	chBufor[1] = 0;	//obrócone rząd z kolumną: 5=LGR, 6=PGR, 7=DPR, 8=DLR
	//chBufor[2] = 0;
	//chBufor[3] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_ORIENTATION, EXIF_TYPE_SHORT, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);	//wstaw dane zamiast offsetu

	nRozmiar = sprintf((char*)chBufor, "%4d:%02d:%02d %02d:%02d:%02d", stData.Year + 2000, stData.Month, stData.Date, stCzas.Hours, stCzas.Minutes, stCzas.Seconds);
	PrzygotujTag(&wskchAdresTAG, EXTAG_DATE_TIME, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "%c %s 2025 ", 0xA9, chNapisLcd[STR_PITLAB]);
	PrzygotujTag(&wskchAdresTAG, EXTAG_COPYRIGT, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	wskchAdresExif = wskchAdresDanych + ROZMIAR_INTEROPER;
	wskchAdresExif = (uint8_t*)(((uint32_t)wskchAdresExif + 1) & 0xFFFFFFFE);	//wyrównanie do 2
	nOffset = (uint32_t)wskchAdresExif - (uint32_t)wskchPoczatekTIFF;	//offset do Exif IFD, czyli różnica adresów IFD0 i Exif_IFD
	chBufor[0] = (uint8_t)(nOffset);	//młodszy przodem
	chBufor[1] = (uint8_t)(nOffset >> 8);
	chBufor[2] = (uint8_t)(nOffset >> 16);
	chBufor[3] = (uint8_t)(nOffset >> 24);
	PrzygotujTag(&wskchAdresTAG, EXTAG_EXIF_IFD, EXIF_TYPE_LONG, chBufor, 4, &wskchAdresDanych, wskchPoczatekTIFF);	//rozmiar=0, dane umieść w TAGu

	wskchAdresGPS = wskchAdresExif + LICZBA_TAGOW_EXIF * (ROZMIAR_TAGU + 8) + ROZMIAR_INTEROPER;	//liczbę nadmiarowych danych tagów Exif - przyjmuję jako 8 na tag, bo to głównie Rational
	wskchAdresGPS = (uint8_t*)(((uint32_t)wskchAdresGPS + 1) & 0xFFFFFFFE);	//wyrównanie do 2
	nOffset = (uint32_t)wskchAdresGPS - (uint32_t)wskchPoczatekTIFF;	//offset do Exif IFD, czyli różnica adresów IFD0 i GPS_IFD
	chBufor[0] = (uint8_t)(nOffset);	//młodszy przodem
	chBufor[1] = (uint8_t)(nOffset >> 8);
	chBufor[2] = (uint8_t)(nOffset >> 16);
	chBufor[3] = (uint8_t)(nOffset >> 24);
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_IFD, EXIF_TYPE_LONG, chBufor, 4, &wskchAdresDanych, wskchPoczatekTIFF);		//rozmiar=0, dane umieść w TAGu

	//wskaźnik do IFD1: 0 = brak
	*(wskchAdresTAG + 0) = 0;	//młodszy przodem
	*(wskchAdresTAG + 1) = 0;
	*(wskchAdresTAG + 2) = 0;
	*(wskchAdresTAG + 3) = 0;
	//Tutaj lądują dane do których wskaźniki są podane w tagach IFD0.....

	//TAG-i Exif IFD ***********************************************************************************************************************************
	*(wskchAdresExif + 0) = (uint8_t)LICZBA_TAGOW_EXIF;			//Liczba tagów - Interoperability number
	*(wskchAdresExif + 1) = (uint8_t)(LICZBA_TAGOW_EXIF >> 8);

	wskchAdresTAG = wskchAdresExif + ROZMIAR_INTEROPER;	//adres miejsca gdzie zapisać pierwszy TAG w grupie Exif
	wskchAdresDanych = wskchAdresTAG + (LICZBA_TAGOW_EXIF * ROZMIAR_TAGU);	//adres za grupą tagów gdzie zapisać dane

	fTemp1 = stDane->fTemper[0] - 273.15f;	//Dodać temperaturę otoczenia, na razie jest temperatura IMU [°C]
	fTemp2 = floorf(fTemp1);	//pełne dziesiate części stopni
	chBufor[0] = (int8_t)fTemp2;		//liczba ze znakiem
	chBufor[1] = (int8_t)((int16_t)fTemp2 >> 8);
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_TEMPERATURE, EXIF_TYPE_SRATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//SRATIONAL x1

	fTemp2 = floorf(stDane->fCisnieBzw[0] / 100);	//ciśnienie w hPa
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = (uint8_t)((uint16_t)fTemp2 >> 8);
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// ciśnienie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_PRESSURE, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	fTemp1 = stDane->fKatIMU1[1] * 180.0f / M_PI;	//kąt pochylenie IMU w dziesiatych częściach stopnia docelowo dodać IMU kamery
	fTemp2 = floorf(fTemp1);	//pełne dziesiate części stopni
	chBufor[0] = (int8_t)fTemp2;		//liczba ze znakiem
	chBufor[1] = (int8_t)((int16_t)fTemp2 >> 8);
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_CAM_ELEVATION, EXIF_TYPE_SRATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//SRATIONAL x1

	chBufor[0] = (uint8_t)(stKonf->chSzerWe / stKonf->chSzerWy);	//zoom cyfrowy po szerokości
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// zoom / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_DIGITAL_ZOOM, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	//TAG-i GPS IFD ***********************************************************************************************************************************
	*(wskchAdresGPS + 0) = (uint8_t)LICZBA_TAGOW_GPS;			//Liczba tagów - Interoperability number
	*(wskchAdresGPS + 1) = (uint8_t)(LICZBA_TAGOW_GPS >> 8);

	wskchAdresTAG = wskchAdresGPS + ROZMIAR_INTEROPER;	//adres miejsca gdzie zapisać pierwszy TAG w grupie GPS
	wskchAdresDanych = wskchAdresTAG + (LICZBA_TAGOW_GPS * ROZMIAR_TAGU);	//adres za grupą tagów gdzie zapisać dane

	chBufor[0] = 2;
	chBufor[1] = 3;
	chBufor[2] = 0;
	chBufor[3] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_TAG_VERSION, EXIF_TYPE_BYTE, chBufor, 4, &wskchAdresDanych, wskchPoczatekTIFF);	//BYTE x4, ale nie wstawiaj ich do danych

	fTemp1 = stDane->stGnss1.dSzerokoscGeo;
	if (fTemp1 < 0)
	{
		chBufor[0] = 'S';
		fTemp1 *= -1.0f;	//zmień znak na dodatni
	}
	else
		chBufor[0] = 'N';
	chBufor[1] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_NS_LATI_REF, EXIF_TYPE_ASCII, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);	//ASCII x2
	fTemp2 = (uint8_t)floorf(fTemp1);		//pełne stopnie;
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;

	fTemp2 = (fTemp1 - fTemp2) * 60.0f;	//ułamkowa część stopni -> minuty
	fTemp1 = floorf(fTemp2);			//pełne minuty
	chBufor[8] = (uint8_t)fTemp1;
	chBufor[9] = 0;
	chBufor[10] = 0;
	chBufor[11] = 0;
	chBufor[12] = 1;		// minuty / 1
	chBufor[13] = 0;
	chBufor[14] = 0;
	chBufor[15] = 0;

	fTemp2 = (fTemp2 - fTemp1) * 6000.0f;	//ułamkowa część minut -> //sekundy * 100
	fTemp1 = floorf(fTemp2);			//pełne setki sekund
	chBufor[17] = (uint8_t)fTemp1;
	chBufor[16] = (uint8_t)((uint16_t)fTemp1 >> 8);
	chBufor[18] = 0;
	chBufor[19] = 0;
	chBufor[20] = 100;		// sekundy / 100
	chBufor[21] = 0;
	chBufor[22] = 0;
	chBufor[23] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_LATITUDE, EXIF_TYPE_RATIONAL, chBufor, 24, &wskchAdresDanych, wskchPoczatekTIFF);

	fTemp1 = stDane->stGnss1.dDlugoscGeo;
	if (fTemp1 < 0)
	{
		chBufor[0] = 'W';
		fTemp1 *= -1.0f;	//zmień znak na dodatni
	}
	else
		chBufor[0] = 'E';
	chBufor[1] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_EW_LONGI_REF, EXIF_TYPE_ASCII, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);	//ASCII x2

	//fTemp2 = fTemp1 * 180 / M_PI;	//radiany -> stopnie
	fTemp2 = floorf(fTemp1);			//pełne stopnie
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;

	fTemp2 = (fTemp1 - fTemp2) * 60.0f;	//ułamkowa część stopni -> minuty
	fTemp1 = floorf(fTemp2);			//pełne minuty
	chBufor[8] = (uint8_t)fTemp1;
	chBufor[9] = 0;
	chBufor[10] = 0;
	chBufor[11] = 0;
	chBufor[12] = 1;		// minuty / 1
	chBufor[13] = 0;
	chBufor[14] = 0;
	chBufor[15] = 0;

	fTemp2 = (fTemp2 - fTemp1) * 6000.0f;		//ułamkowa część minut
	fTemp1 = floorf(fTemp2);	//pełne setki sekund
	chBufor[17] = (uint8_t)fTemp1;
	chBufor[16] = (uint8_t)((uint16_t)fTemp1 >> 8);
	chBufor[18] = 0;
	chBufor[19] = 0;
	chBufor[20] = 100;		// sekundy / 100
	chBufor[21] = 0;
	chBufor[22] = 0;
	chBufor[23] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_LONGITUDE, EXIF_TYPE_RATIONAL, chBufor, 24, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x3

	fTemp1 = stDane->stGnss1.fWysokoscMSL;
	if (fTemp1 < 0)
	{
		chBufor[0] = 1;		//1 = poniżej poziomu morza
		fTemp1 *= -1.0f;	//zamień na dodatnie
	}
	else
		chBufor[0] = 0;		//0 = powyżej poziomu morza
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_ALTITUDE_REF, EXIF_TYPE_BYTE, chBufor, 1, &wskchAdresDanych, wskchPoczatekTIFF);	//BYTE x1

	fTemp1 *= 10.0f;			//dziesiąte części metra
	fTemp2 = floorf(fTemp1);		//pełne dziesiątki
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = (uint8_t)((uint32_t)fTemp2 >> 8);
	chBufor[2] = (uint8_t)((uint32_t)fTemp2 >> 16);
	chBufor[3] = (uint8_t)((uint32_t)fTemp2 >> 24);
	chBufor[4] = 10;		// metrów / 10
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_ALTITUDE, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	//timestamp GPS
	chBufor[0] = stDane->stGnss1.chGodz;
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// godz / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	chBufor[8] = stDane->stGnss1.chMin;
	chBufor[9] = 0;
	chBufor[10] = 0;
	chBufor[11] = 0;
	chBufor[12] = 1;	// minuty / 1
	chBufor[13] = 0;
	chBufor[14] = 0;
	chBufor[15] = 0;
	chBufor[16] = stDane->stGnss1.chSek;
	chBufor[17] = 0;
	chBufor[18] = 0;
	chBufor[19] = 0;
	chBufor[20] = 1;	// sekundy / 1
	chBufor[21] = 0;
	chBufor[22] = 0;
	chBufor[23] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_TIME_STAMP, EXIF_TYPE_RATIONAL, chBufor, 24, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x3

	chBufor[0] = 'K';	//km/h
	chBufor[1] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_SPEED_REF, EXIF_TYPE_ASCII, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);		//ASCII,

	fTemp1 = stDane->stGnss1.fPredkoscWzglZiemi;	//prędkość w m/s
	fTemp2 = fTemp1 * 10.0f / 3.6f;					//prędkość w 10*km/h
	fTemp2 = floorf(fTemp1);						//pełne dziesiątki
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = (uint8_t)((uint32_t)fTemp2 >> 8);
	chBufor[2] = (uint8_t)((uint32_t)fTemp2 >> 16);
	chBufor[3] = (uint8_t)((uint32_t)fTemp2 >> 24);
	chBufor[4] = 10;		// km/h / 10
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_SPEED, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	//aktualizuj rozmiar struktury Exif
	nRozmiar = wskchAdresDanych - (uint8_t*)chNaglJpegExif;	//rozmiar to różnica wskaźników początku i końca danych
	chNaglJpegExif[4] = (uint8_t)(nRozmiar >> 8);   // Rozmiar (big endian)
	chNaglJpegExif[5] = (uint8_t)nRozmiar;
	return nRozmiar;
}



