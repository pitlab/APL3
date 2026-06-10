//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi odbiornika RC Crossfire na UART8
//
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "Crossfire.h"
#include "WeWyRC.h"
#include <GNSS.h>
#include "Uarty.h"
#include "Czas.h"

extern UART_HandleTypeDef huart8;
extern stRC_t stRC1, stRC2;	//struktura danych odbiorników RC1 i RC2
extern unia_wymianyCM4_t uDaneCM4;
extern unia_wymianyCM7_t uDaneCM7;
extern uint8_t chBuforOdbioruUart8[ROZMIAR_BUF_ODB_UART8];
uint16_t sPoprzedniKanal;

uint8_t chBuforAnalizyCrossfire[ROZMIAR_BUF_ANA_CRSF];
volatile uint8_t chWskNapBufAnaCRSF, chWskOprBufAnaCRSF; 	//wskaźniki napełniania i opróżniania kołowego bufora odbiorczego analizy danych Crossfire
uint8_t chRamkaCRSF[ROZMIAR_RAMKI_CRSF];
uint8_t cEtapOdbioruRamki;	//wskazuje która część ramki jest odbierana
uint8_t cLicznikDanych;	//zlicza dane w polu payload
//union {
//	stSpakowaneKanalyCRSF_t stSpakowaneKanalyCRSF;
//	uint8_t cRamkaCRSF[22];
//} uSpakowaneKanalyCRSF;

uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};



////////////////////////////////////////////////////////////////////////////////
// Inicjuje pracę odbiornika RC w gnieździe GNSS (z powodu braku innego)
// Parametry: brak
// Zwraca: kod zakończenia inicjalizacji: ERR_DONE = zakończono, BLAD_OK - w trakcie
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujCrossfire(void)
{
	uint8_t cBłąd = BLAD_OK;

	chWskNapBufAnaCRSF = chWskOprBufAnaCRSF = 0;	//inicjuj wskaźniki napełniania i opróżniania buforma kołowego analizy danych z UART8
	cEtapOdbioruRamki = EORC_ADRES;
	huart8.Init.BaudRate = 420000; 		//według dokumentacji TBS iNAV
	cBłąd = UART_SetConfig(&huart8);
	HAL_UART_Receive_DMA(&huart8, chBuforOdbioruUart8, ROZMIAR_BUF_ODB_UART8);	//Upewnij się że nie jest uruchamiana funckja inicjalizacji GNSS
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Odbiór danych Crossfire z bufora kołowego i formowanie ich w ramkę
// Parametry:
// [we] *chRamka - wskaźnik na dane formowanej ramki
// [we/wy] *cEtapOdbioru - wskazuje na bieżacy etap odbioru części ramki
// [we] *chBuforAnalizy - wskaźnik na bufor z danymi wejsciowymi
// [we] chWskNapBuf - wskaźnik napełniania bufora wejsciowego
// [wy] *chWskOprBuf - wskaźnik na wskaźnik opróżniania bufora wejściowego
// Zwraca: kod wykonania operacji: BLAD_GOTOWE gdy jest kompletna ramka, BLAD_OK gdy skończyły się dane ale nie skompetowano jeszcze ramki
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t OdbiórRamkiCrossfire(uint8_t *chRamka, uint8_t *cEtapOdbioru, uint8_t *cLicznikDanych, uint8_t *chBufor, uint8_t chWskNapBuf, uint8_t *chWskOprBuf)
{
	uint8_t cDane;
	uint8_t cBłąd = BLAD_OK;

	while (chWskNapBuf != *chWskOprBuf)
	{
		cDane = chBufor[*chWskOprBuf];
		(*chWskOprBuf)++;
		if (*chWskOprBuf >= ROZMIAR_BUF_ANA_CRSF)
			*chWskOprBuf = 0;

		switch (*cEtapOdbioru)
		{
		case EORC_ADRES:	//odbierany jest SyncByte będący adresem urządzenia. Reaguję na CRSF_ADR_FLIGHT_CTRL oraz na CRSF_ADR_BROADCAST
			//if ((cDane == CRSF_ADR_FLIGHT_CTRL) || (cDane == CRSF_ADR_BROADCAST))
			if (cDane == CRSF_ADR_FLIGHT_CTRL)
			{
				chRamka[0] = cDane;
				(*cEtapOdbioru)++;
			}
			break;

		case EORC_DLUGOSC:
			if (cDane)	//jeżeli długość jest niezerowa
			{
				chRamka[EORC_DLUGOSC] = cDane;
				(*cEtapOdbioru)++;
			}
			else
				*cEtapOdbioru = EORC_ADRES;
			break;

		case EORC_TYP:
			chRamka[EORC_TYP] = cDane;
			*cLicznikDanych = 0;
			(*cEtapOdbioru)++;
			break;

		case EORC_DANE:
			chRamka[EORC_DANE + *cLicznikDanych] = cDane;
			(*cLicznikDanych)++;
			if (*cLicznikDanych == chRamka[EORC_DLUGOSC] - 2)	// pole długość zawiera liczbę danych + Typ + CRC
				*cEtapOdbioru = EORC_CRC;
			break;

		case EORC_CRC:
			*cEtapOdbioru = EORC_ADRES;
			chRamka[EORC_DANE + *cLicznikDanych] = cDane;
			uint8_t cCrc = crc8(&chRamka[EORC_TYP], chRamka[EORC_DLUGOSC] - 1);	//CRC nie jest liczone z 2 pierwszych bajtów nagłówka i długości oraz z CRC
			if (cDane == cCrc)
			{
				cBłąd = BLAD_GOTOWE;

			}
			else
			{
				cBłąd = BLAD_CRC;
				//HAL_GPIO_TogglePin(GPIOI, GPIO_PIN_10);			//kanał serw 7 skonfigurowany jako IO
			}
			break;

			//for (uint8_t n=EORC_TYP; n<; n++)
		default:	cBłąd = BLAD_ZLE_DANE;
			*cEtapOdbioru = EORC_ADRES;	//napraw automat stanu jeżeli wypadł z synchronizmu
			break;
		}
	}
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja licząca CRC ramek Crossfire wrac z tablicą wyników
// Parametry:
// [we] *ptr - wskaźnik na dane wejściowe
// [we] len - rozmiar danych
// Zwraca: wynik obliczeń CRC
////////////////////////////////////////////////////////////////////////////////
uint8_t crc8(const uint8_t * ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}



////////////////////////////////////////////////////////////////////////////////
// Analizuje dane odebrane z odbiornika, dekoduje je i wstawia do struktury danych
// Parametry:
// [we] *chRamka - wskaźnik na odebraną ramkę Crossfire
// [wy] *stRC - wskaźnik na strukturę danych odbiorników RC zawierajacą stan kanałów
// Zwraca: kod zakończenia inicjalizacji: ERR_DONE = zakończono, BLAD_OK - w trakcie
////////////////////////////////////////////////////////////////////////////////
uint8_t AnalizujCrossfire(uint8_t *cRamka, stRC_t *stRC)
{
	uint8_t cBłąd = BLAD_OK;
	uint16_t sWartoscKanalu;

	switch(cRamka[EORC_TYP])
	{
	case TYPCRSF_LINK_STAT:		 	//0x14 Link Statistics
		stRC->cRSSI_Up1 = cRamka[EORC_DANE + 0];		// Uplink RSSI Antenna 1 (dBm * -1)
		stRC->cRSSI_Up2 = cRamka[EORC_DANE + 1];		// Uplink RSSI Antenna 2 (dBm * -1)
		stRC->cJakoscUpLinku = cRamka[EORC_DANE + 2];	// Uplink Package success rate / Link quality (%)
		stRC->cSNR_UpLinku = cRamka[EORC_DANE + 3];		// Uplink SNR (dB)
														// number of currently best antenna
														// enum {4fps = 0 , 50fps, 150fps}
		stRC->cMocRF = cRamka[EORC_DANE + 6];			// enum {0mW = 0, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW, 250mW, 50mW}
		stRC->cRSSI_Down = cRamka[EORC_DANE + 7];		// Downlink RSSI (dBm * -1)
		stRC->cJakoscDnLinku = cRamka[EORC_DANE + 8];	// Downlink Package success rate / Link quality (%)
		stRC->cSNR_DnLinku = cRamka[EORC_DANE + 9];		// Downlink SNR (dB)
		break;

	case TYPCRSF_CHAN_PACKED:	 	//0x16 RC Channels Packed Payload
		sWartoscKanalu = ((uint16_t)cRamka[EORC_DANE + 0] | (((uint16_t)cRamka[EORC_DANE + 1]  << 8) & 0x7E0));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[0] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0001;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 1] >> 3) | (((uint16_t)cRamka[EORC_DANE + 2] << 5) & 0x7E0));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[1] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0002;
			if (sWartoscKanalu != sPoprzedniKanal)
				//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);			//kanał serw 1 skonfigurowany jako IO
			sPoprzedniKanal = sWartoscKanalu;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 2] >> 6) | (((uint16_t)cRamka[EORC_DANE + 3] << 2) & 0x3FC) | (((uint16_t)cRamka[EORC_DANE + 4] << 10) & 0x400));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[2] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0004;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 4] >> 1) | (((uint16_t)cRamka[EORC_DANE + 5] << 7) & 0x780));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[3] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0008;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 5] >> 4) | (((uint16_t)cRamka[EORC_DANE + 6] << 4) & 0x7F0));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[4] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0010;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 6] >> 7) | (((uint16_t)cRamka[EORC_DANE + 7] << 1) & 0x1FE) | (((uint16_t)cRamka[EORC_DANE + 8] << 9) & 0x600));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[5] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0020;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 8] >> 2) | (((uint16_t)cRamka[EORC_DANE + 9] << 6) & 0x7C0));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[6] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0040;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 9] >> 5) | (((uint16_t)cRamka[EORC_DANE + 10] << 3) & 0x7F8));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[7] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0080;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 11] >> 0) | (((uint16_t)cRamka[EORC_DANE + 12] << 8) & 0x700));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[8] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0100;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 12] >> 3) | (((uint16_t)cRamka[EORC_DANE + 13] << 5) & 0x7E0));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[9] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0200;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 13] >> 6) | (((uint16_t)cRamka[EORC_DANE + 14] << 2) & 0x3FC) | (((uint16_t)cRamka[EORC_DANE + 15] << 10) & 0x400));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[10] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0400;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 15] >> 1) | (((uint16_t)cRamka[EORC_DANE + 16] << 7) & 0x780));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[11] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x0800;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 16] >> 4) | (((uint16_t)cRamka[EORC_DANE + 17] << 4) & 0x7F0));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[12] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x1000;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 17] >> 7) | (((uint16_t)cRamka[EORC_DANE + 18] << 1) & 0x1FE) | (((uint16_t)cRamka[EORC_DANE + 19] << 9) & 0x600));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[13] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x2000;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 19] >> 2) | (((uint16_t)cRamka[EORC_DANE + 20] << 6) & 0x7C0));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[14] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x4000;
		}
		else
			cBłąd = BLAD_ZLE_DANE;

		sWartoscKanalu = (((uint16_t)cRamka[EORC_DANE + 20] >> 5) | (((uint16_t)cRamka[EORC_DANE + 21] << 3) & 0x7F8));
		if (sWartoscKanalu < WE_RC_MAX)
		{
			stRC->sKanaly[15] = sWartoscKanalu + OFFSET_ZERA_CROSSFIRE;
			stRC->sZdekodowaneKanaly |= 0x8000;
		}
		else
			cBłąd = BLAD_ZLE_DANE;
		break;

	default:	cBłąd = BLAD_ZLE_DANE;	break;
	}

	if (cBłąd == BLAD_OK)
		stRC->nCzasOdOstatniejRamki = PobierzCzasT7();
	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja nadrzędna wywoływana z pętli głównej, skupiająca w sobie funkcje potrzebne do obsługi ramki Crossfire
// Parametry: brak
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaRamkiCrossfire(void)
{
	uint8_t cBłąd;

	cBłąd = OdbiórRamkiCrossfire(chRamkaCRSF, &cEtapOdbioruRamki, &cLicznikDanych, chBuforAnalizyCrossfire, chWskNapBufAnaCRSF, (uint8_t*)&chWskOprBufAnaCRSF);
	if (cBłąd == BLAD_GOTOWE)
		cBłąd = AnalizujCrossfire(chRamkaCRSF, &stRC2);

	return cBłąd;
}
