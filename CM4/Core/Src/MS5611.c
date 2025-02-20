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
#include "main.h"
#include "spi.h"
#include "modul_IiP.h"

extern SPI_HandleTypeDef hspi2;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint16_t sKonfig[6];  //współczynniki kalibracyjne
static uint8_t chBuf5611[4];
static uint8_t chProporcjaPomiarow;
float fP0_MS5611 = 0.0f;	//ciśnienie zerowe do obliczeń wysokości [Pa]
static uint16_t sLicznikUsrednianiaP0 = 0;			//licznik uśredniania ciśnienia zerowego do obliczeń wysokości

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z PROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMS5611(void)
{
	uint32_t nCzasStart;
	uint8_t chErr;

    nCzasStart = PobierzCzas();
    do
    {
    	chBuf5611[0] = PMS_RESET;
    	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
    	chErr = HAL_SPI_Transmit(&hspi2, chBuf5611, 1, 5);	//typowy czas wykonania operacji to 2,8ms
    	HAL_Delay(3);
    	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
    	if (chErr)
    		return chErr;

    	for (uint16_t n=0; n<6; n++)
    		sKonfig[n] = CzytajSPIu16mp(PMS_PROM_READ_C1 + 2*n);

        if (MinalCzas(nCzasStart) > 5000)   //czekaj maksymalnie 5000us
            return ERR_TIMEOUT;

        //sprawdź czy odczytana konfiguracja nie jest samymi zerami ani jedynkami
        for (uint16_t n=0; n<6; n++)
        {
        	if ((sKonfig[n] == 0) || (sKonfig[n] == 0xFFFF))
        		chErr = ERR_ZLE_DANE;
        }
    }
    while (chErr);
    uDaneCM4.dane.nZainicjowano |= INIT_MS5611;
    sLicznikUsrednianiaP0 = LICZBA_PROBEK_USREDNIANIA;	//rozpocznij przygotowanie P0
    return chErr;
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
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	HAL_SPI_TransmitReceive(&hspi2, chBuf5611, chBuf5611, 4, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	nWynik = ((uint32_t)chBuf5611[1] <<16) + ((uint32_t)chBuf5611[2] <<8) + chBuf5611[3];
	return nWynik;
}


////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość konwersji temperatury
// dT = D2 - TREF = D2 - C5 * 2^8
// TEMP = 20°C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
// Parametry: nKonwersja - wynik konwersji
// Zwraca: temepratura [°C]
// Czas wykonania: 
////////////////////////////////////////////////////////////////////////////////
float MS5611_LiczTemperature(uint32_t nKonwersja, int32_t* ndTemp)
{
    int32_t nTemp;
    int64_t lTemp2 = 0;

    *ndTemp = nKonwersja - (int32_t)sKonfig[4] * 0x100;
    nTemp = 2000.0 + ((float)*ndTemp * sKonfig[5]) / 8388608;

    if (nTemp < 2000)	//jeżeli temepratura < 20°C
    	lTemp2 = (*ndTemp * *ndTemp) / 32768;

    return (float)(nTemp - lTemp2) / 100;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość konwersji temperatury ciśnienia
// OFF = OFFT1 + TCO * dT = C2 * 2^16 + (C4 * dT ) / 2^7
// SENS = SENST1 + TCS * dT = C1 * 2^15 + (C3 * dT ) / 2^8
// P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15
// Parametry: nic
// Zwraca: ciśnienie w Pa
// Czas wykonania: 
////////////////////////////////////////////////////////////////////////////////
float MS5611_LiczCisnienie(uint32_t nKonwersja, int32_t ndTemp)
{
    int64_t llOffset, llOffset2 = 0;
    int64_t llSens, llSens2 = 0;
    int32_t nTemp;
	float fCisnienie;
	uint64_t lTempKwadrat;

    llOffset = ((int64_t)sKonfig[1] * 65536) + (((int64_t)sKonfig[3] * ndTemp) / 128);
    llSens =   ((int64_t)sKonfig[0] * 32768) + (((int64_t)sKonfig[2] * ndTemp) / 256);
    nTemp = 2000.0 + ((float)ndTemp * sKonfig[5]) / 8388608;

    if (nTemp < 2000)	//jeżeli temepratura < 20°C
	{
    	lTempKwadrat = (nTemp - 2000) * (nTemp - 2000);		//kwadrat temperatury
    	llOffset2 = 5 * lTempKwadrat / 2;
    	llSens2 = 5 * lTempKwadrat / 4;
	}

    if (nTemp < -1500)		//jeżeli temepratura < -15°C
	{
    	lTempKwadrat = (nTemp + 1500) * (nTemp + 1500);		//kwadrat temperatury
		llOffset2 += 7 * lTempKwadrat;
		llSens2 += 11 * lTempKwadrat / 2;
	}

    llOffset -= llOffset2;
    llSens -= llSens2;
    fCisnienie = (float)(((int64_t)nKonwersja * llSens / 2097152) - llOffset) / 32768.0f;
    return fCisnienie; //wynik w Pa
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
	uint8_t chErr;
	float fCisnienie = 0;
	static int32_t ndT;	//różnica między temepraturą bieżącą a referencyjną. Potrzebna do obliczeń ciśnienia. Zmienna statyczna aby istniała poza czasem życia funkcji

	if ((uDaneCM4.dane.nZainicjowano & INIT_MS5611) != INIT_MS5611)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujMS5611();
		if (chErr)
			return chErr;
		else
		{
			chBuf5611[0] = PMS_CONV_D2_OSR2048;
			ZapiszSPIu8(chBuf5611, 1);	//uruchom konwersję temperatury nie trwajacą dłużej niż obieg pętli, czymi max 2048
		}
	}
	else
	{
		switch (chProporcjaPomiarow)
		{
		case 0:
			nKonwersja = CzytajWynikKonwersjiMS5611();
			uDaneCM4.dane.fTemper[0] = (7 * uDaneCM4.dane.fTemper[0] + MS5611_LiczTemperature(nKonwersja, &ndT)) / 8;	//filtruj temepraturę
			chBuf5611[0] = PMS_CONV_D1_OSR2048;
			ZapiszSPIu8(chBuf5611, 1);		//uruchom konwersję ciśnienia
			break;

		case 7:
			nKonwersja = CzytajWynikKonwersjiMS5611();
			fCisnienie = MS5611_LiczCisnienie(nKonwersja, ndT);
			uDaneCM4.dane.fCisnie[0] = (7 * uDaneCM4.dane.fCisnie[0] + fCisnienie) / 8;

			chBuf5611[0] = PMS_CONV_D2_OSR256;
			ZapiszSPIu8(chBuf5611, 1);		//uruchom konwersję temperatury
			break;

		default:
			nKonwersja = CzytajWynikKonwersjiMS5611();
			fCisnienie = MS5611_LiczCisnienie(nKonwersja, ndT);
			uDaneCM4.dane.fCisnie[0] = (7 * uDaneCM4.dane.fCisnie[0] + fCisnienie) / 8;
			chBuf5611[0] = PMS_CONV_D1_OSR2048;
			ZapiszSPIu8(chBuf5611, 1);		//uruchom konwersję ciśnienia
			break;
		}
		chProporcjaPomiarow++;
		chProporcjaPomiarow &= 0x07;

		//przygotuj P0
		if (fCisnienie > 0)		//wykonaj tylko dla cykli pomiaru ciśnienia, pomiń cykle pomiary tempertury
		{
			if (sLicznikUsrednianiaP0)	//czy przygotowanie ciśnienia P0 jeszcze trwa
			{
				uDaneCM4.dane.fWysoko[1] = 0.0f;
				fP0_MS5611 = (127 * fP0_MS5611 + fCisnienie) / 128;
				sLicznikUsrednianiaP0--;
				if (sLicznikUsrednianiaP0 == 0)
					uDaneCM4.dane.nZainicjowano |= INIT_P0_MS4525;
			}
			else
				uDaneCM4.dane.fWysoko[1] = WysokoscBarometryczna(uDaneCM4.dane.fCisnie[0], fP0_MS5611, uDaneCM4.dane.fTemper[0]);	//P0 gotowe więc oblicz wysokość
		}
	}
	return chErr;
}
