//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika ciśnienia MS5611 na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "MS5611.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"

extern SPI_HandleTypeDef hspi2;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint16_t sKonfig[5];  //współczynniki kalibracyjne
static uint8_t chBuf5611[4];
uint8_t chErr;
int32_t ndT;	//różnica między temepraturą bieżącą a referencyjną. Potrzebna do obliczeń ciśnienia
float fTemperatura;
uint8_t chProporcjaPomiarow;

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z PROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMS5611(void)
{
	uint32_t nCzasStart;

    nCzasStart = PobierzCzas();
    do
    {
    	chBuf5611[0] = PMS_RESET;
    	chErr = HAL_SPI_Transmit(&hspi2, chBuf5611, 1, 5);	//typowy czas wykonania operacji to 2,8ms
    	if (chErr)
    		return chErr;

    	for (uint16_t n=0; n<6; n++)
    		sKonfig[n] = CzytajKonfiguracjeMS5611(PMS_PROM_READ_C1 + 2*n);

        if (MinalCzas(nCzasStart) > 2400)   //czekaj maksymalnie 1200us
            return ERR_TIMEOUT;
    }
    while (!sKonfig[0] | !sKonfig[1] | !sKonfig[2] | !sKonfig[3] | !sKonfig[4] | !sKonfig[5]);
    uDaneCM4.dane.nZainicjowano |= INIT_MS5611;
    return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje wspólczynnik konfiguracyjny
// Parametry: chAdres - adres współczynnika kalibracyjnego
// Zwraca: wartość współczynnika kalibracyjnego
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint16_t CzytajKonfiguracjeMS5611(uint8_t chAdres)
{
	uint16_t sWspolczynnik;

	chBuf5611[0] = chAdres;
	chErr |= HAL_SPI_TransmitReceive(&hspi2, chBuf5611, chBuf5611, 3, 5);
	sWspolczynnik = ((uint16_t)chBuf5611[1] <<8) + chBuf5611[2];
	return sWspolczynnik;
}



////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna konwersje temperatury lub ciśnienia
// Parametry: chTyp - rodzaj konwersji do wykonania
// Zwraca: kod  błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t StartKonwersjiMS5611(uint8_t chTyp)
{
	chBuf5611[0] = chTyp;
	return HAL_SPI_Transmit(&hspi2, chBuf5611, 1, 5);
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje wynik konwersji
// Parametry: nic
// Zwraca: wynik konwersji
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint32_t CzytajWynikKonwersjiMS5611(void)
{
	uint32_t nWynik;

	chBuf5611[0] = PMS_ADC_READ;
	chErr |= HAL_SPI_TransmitReceive(&hspi2, chBuf5611, chBuf5611, 4, 5);
	nWynik = ((uint32_t)chBuf5611[1] <<16) + ((uint32_t)chBuf5611[2] <<8) + chBuf5611[3];
	return nWynik;
}


////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość konwersji temperatury
// dT = D2 - TREF = D2 - C5 * 2^8
// TEMP = 20°C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
// Parametry: nKonwersja - wynik konwersji
// Zwraca: temepratura
// Czas wykonania: 
////////////////////////////////////////////////////////////////////////////////
float MS5611_LiczTemperature(uint32_t nKonwersja)
{
    float fTemp;


    //fdT = nKonwersja - ((float)sC5 * 0x100);
    ndT = nKonwersja - (int32_t)sKonfig[4] * 0x100;
    fTemp = 2000.0 + (ndT * sKonfig[5])/8388608;
    return fTemp/100;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość konwersji temperatury ciśnienia
// OFF = OFFT1 + TCO * dT = C2 * 2^16 + (C4 * dT ) / 2^7
// SENS = SENST1 + TCS * dT = C1 * 2^15 + (C3 * dT ) / 2^8
// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15
// Parametry: nic
// Zwraca: ci�nienie w kPa
// Czas wykonania: 
////////////////////////////////////////////////////////////////////////////////
float MS5611_LiczCisnienie(uint32_t nKonwersja)
{
    int64_t llOffset, llSens;
	int32_t nCisnienie;
    //double dTemp, dOffset, dSens;

    llOffset = ((int64_t)sKonfig[1] * 65536) + (((int64_t)sKonfig[3] * ndT) / 128);
    llSens =   ((int64_t)sKonfig[0] * 32768) + (((int64_t)sKonfig[2] * ndT) / 256);

    nCisnienie = ((((int64_t)nKonwersja * llSens / 2097152) - llOffset) / 32768);

    //dOffset = (fdT * sC4)/128 + ((double)sC2*65536);
    //dSens = (fdT * sC3)/256 + (double)sC1*32768;
    //dTemp = ((lD1 * dSens)/2097152 - dOffset)/32768;
    return (float)nCisnienie/1000; //wynik w kPa
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Wykonuje w pętli 1 pomiar temepratury i 7 pomiarów ciśnienia
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaMS5611(void)
{
	uint32_t nKonwersja;

	if ((uDaneCM4.dane.nZainicjowano & INIT_MS5611) != INIT_MS5611)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujMS5611();
		if (!chErr)
			StartKonwersjiMS5611(PMS_CONV_D2_OSR1024);	//uruchom konwersję temperatury nie trwajacą dłużej niż obieg pętli
	}
	else
	{
		switch (chProporcjaPomiarow)
		{
		case 0:
			nKonwersja = CzytajWynikKonwersjiMS5611();
			fTemperatura = MS5611_LiczTemperature(nKonwersja);
			StartKonwersjiMS5611(PMS_CONV_D1_OSR1024);		//uruchom konwersję ciśnienia
			break;

		case 7:
			nKonwersja = CzytajWynikKonwersjiMS5611();
			uDaneCM4.dane.fWysokosc[0] = MS5611_LiczCisnienie(nKonwersja);	//!!!!! potrzebna konwersja z ciśnienia na wysokość
			StartKonwersjiMS5611(PMS_CONV_D2_OSR1024);		//uruchom konwersję temperatury
			break;

		default:
			nKonwersja = CzytajWynikKonwersjiMS5611();
			uDaneCM4.dane.fWysokosc[0] = MS5611_LiczCisnienie(nKonwersja);	//!!!!! potrzebna konwersja z ciśnienia na wysokość
			StartKonwersjiMS5611(PMS_CONV_D1_OSR1024);		//uruchom konwersję ciśnienia
			break;
		}
		chProporcjaPomiarow++;
		chProporcjaPomiarow &= 0x07;
	}
	return chErr;
}
