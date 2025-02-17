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

extern I2C_HandleTypeDef hi2c3;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chProporcjaPomiarow;
extern uint8_t chCzujnikOdczytywanyNaI2CExt;	//identyfikator czujnika odczytywanego na zewntrznym I2C. Potrzebny do tego aby powiązać odczytane dane z rodzajem obróbki
uint8_t chDaneMS4525[5];



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
	if (chErr == ERR_OK)
		uDaneCM4.dane.nZainicjowano |= INIT_MS4525;
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
	float fPsi;
	int16_t sCisnienie;

	sCisnienie = (int16_t)(0x100 * *(dane+0) + *(dane+1));
	fPsi =  ((float)sCisnienie - 1638.3f) * (-2*ZAKRES_POMIAROWY_CISNIENIA) / (0.8*16383) - ZAKRES_POMIAROWY_CISNIENIA;
	return fPsi * PASKALI_NA_PSI;
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




