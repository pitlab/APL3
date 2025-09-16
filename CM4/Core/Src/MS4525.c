//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika ciśnienia różnicowego MS4525 5AI na magistrali I2C
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "MS4525.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"
#include "spi.h"
#include "modul_IiP.h"

//Zakres pomiarowy to +-1psi co odpowiada +-6894.76 [Pa] co odpowiada prędkosci 108,5 m/s (390,6 km/h)


extern I2C_HandleTypeDef hi2c3;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chProporcjaPomiarow;
extern uint8_t chCzujnikOdczytywanyNaI2CExt;	//identyfikator czujnika odczytywanego na zewntrznym I2C. Potrzebny do tego aby powiązać odczytane dane z rodzajem obróbki
uint8_t chDaneMS4525[5];
float fCiśnienieZerowaniaMS4525;	//ciśnienie zmierzone podczas kalibracji czujnika. Należy odjać je od bieżących wskazań
uint16_t sLicznikZerowaniaMS4525 = MAX_PROB_INICJALIZACJI;	//odlicza czas uśredniania danych z czujnika. Podczas inicjalizacji zlicza próby nieudanej inicjalizacji

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika.
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMS4525(void)
{
	uint8_t chErr;

	//wyślij adres i sprawdź czy odpowie ACK-iem
	chErr = HAL_I2C_Master_Transmit(&hi2c3, MS2545_I2C_ADR, chDaneMS4525, 2, I2C_TIMOUT);
	if (chErr == BLAD_OK)
	{
		uDaneCM4.dane.nZainicjowano |= INIT_MS4525;
		KalibrujZeroMS4525();
	}
	else	//wystapił błąd inicjalizacji. Po wystapieniu MAX_PROB_INICJAIZACJI wyłącz obsługę czujnika
	{
		sLicznikZerowaniaMS4525--;
		if (!sLicznikZerowaniaMS4525)
			uDaneCM4.dane.nBrakCzujnika |= INIT_MS4525;
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Wykonuje w pętli 1 pomiar temperatury i 157 pomiarów ciśnienia
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaMS4525(void)
{
	uint8_t chErr;

	//po MAX_PROB_INICJALIZACJI ustawiany jest bit braku czujnika. Taki czujnik nie jest dłużej obsługiwany
	if (uDaneCM4.dane.nBrakCzujnika & INIT_MS4525)
		return ERR_BRAK_MS4525;

	if ((uDaneCM4.dane.nZainicjowano & INIT_MS4525) != INIT_MS4525)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujMS4525();
		if (chErr)
			return chErr;
	}
	else	//czujnik jest zainicjowany
	{
		switch (chProporcjaPomiarow)
		{
		case 0:
			chCzujnikOdczytywanyNaI2CExt = CISN_ROZN_MS2545;	//odczytaj ciśnienie różnicowe i temepraturę
			chErr = HAL_I2C_Master_Receive_DMA(&hi2c3, MS2545_I2C_ADR, chDaneMS4525, 4);
			break;

		default:
			chCzujnikOdczytywanyNaI2CExt = CISN_TEMP_MS2545;	//odczytaj ciśnienie różnicowe
			chErr = HAL_I2C_Master_Receive_DMA(&hi2c3, MS2545_I2C_ADR, chDaneMS4525, 2);
			break;
		}
		chProporcjaPomiarow++;
		chProporcjaPomiarow &= 0x0F;
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza ciśnienie na podstawie odczytanych danych
// Wzór: Output = 80%*16383 / (2*ZAKRES) * (Cisnienie - (-ZAKRES)) + 10% * 16383
// Po przekształceniu: Cisnienie =  (Output - 1638.3f) * 2*ZAKRES / (0.8*16383) - ZAKRES
// Parametry: *dane - wskaźnik na odczytane dane
// Zwraca: obliczone ciśnienie różnicowe w Pa
////////////////////////////////////////////////////////////////////////////////
float CisnienieMS2545(uint8_t * dane)
{
	float fCisnienie;
	int16_t sCisnienie;

	sCisnienie = (uint16_t)(0x100 * *(dane+0) + *(dane+1));
	fCisnienie =  ((float)sCisnienie - 1638.3f) * (2*ZAKRES_POMIAROWY_CISNIENIA) / (0.8f * 16383) - ZAKRES_POMIAROWY_CISNIENIA;	//wynik w [psi]
	fCisnienie *= PASKALI_NA_PSI;	//wynik w [Pa]

	//zerowanie wskazań czujnika
	if (sLicznikZerowaniaMS4525)
	{
		sLicznikZerowaniaMS4525--;
		fCiśnienieZerowaniaMS4525 = (127 * fCiśnienieZerowaniaMS4525 + fCisnienie) / 128;
		if (sLicznikZerowaniaMS4525 == 0)
			uDaneCM4.dane.nZainicjowano |= INIT_P0_MS4525;
	}
	else
		fCisnienie -= fCiśnienieZerowaniaMS4525;

	return fCisnienie;
}


////////////////////////////////////////////////////////////////////////////////
// Oblicza temperaturę na podstawie odczytanych danych
// Wzór: Output = (Temperatura - (-50°C)) * 2047 / (150°C - (-50°C)) = (Temperatura + 50°C)) * 2047 / 200°C
// Po przekształceniu: Temperatura = Output * 200 / 2047 - 50°C
// Parametry: *dane - wskaźnik na odczytane dane
// Zwraca: obliczona temepratura
////////////////////////////////////////////////////////////////////////////////
float TemperaturaMS2545(uint8_t * dane)
{
	uint16_t sTemperatura;

	sTemperatura = (int16_t)(0x100 * *(dane+2) + *(dane+3));
	return (float)(sTemperatura>>5) * 200.f / 2047.f - 50.f;
}



////////////////////////////////////////////////////////////////////////////////
// Włącza uśrednianie wartości zwracanego ciśnienia aby uznać je za zerowe i móc odejmować od kolejnych pomiarów
// Parametry: *fZCisnienieZero - wskaźnik na zmienną z cśnieniem zerowym
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
void KalibrujZeroMS4525(void)
{
	sLicznikZerowaniaMS4525 = LICZBA_PROBEK_USREDNIANIA_MS4525;
}



