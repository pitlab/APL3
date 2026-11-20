//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi protokołu lidarów Benewake na przykładzie TF02-PRO
// Komunikacja po I2C
//
// (c) PitLab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "BenewakeI2C.h"
#include "Uarty.h"
#include "WymianaCM4.h"

uint8_t cBuforLidaru[ROZMIAR_BUF_LIDARU];
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t cLicznikPróbInicjalizacji = 0;
extern I2C_HandleTypeDef hi2c3;
extern volatile uint16_t sCzujnikZapisywanyNaI2CExt, sCzujnikOdczytywanyNaI2CExt;
static uint8_t cSekwencjaPomiaruBenewake;


////////////////////////////////////////////////////////////////////////////////
// Funkcja inicjalizuje lidar sprawdzając jego obecność
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLidarI2C(void)
{
	uint8_t cBłąd = BLAD_OK;

	//odpytaj wersję czujnika
	cBuforLidaru[0] = NAGLOWEK_BENEWAKE;
	cBuforLidaru[1] = 0x04;			//rozmiar ramki wychodzącej
	cBuforLidaru[2] = LB_WERSJA;
	cBuforLidaru[3] = 0x5F;			//suma kontrolna
	cBłąd |= HAL_I2C_Master_Transmit(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 4, TIMEOUT_POLECENIA);
	cBłąd |= HAL_I2C_Master_Receive(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 7, TIMEOUT_POLECENIA);
	if ((cBłąd) || (cBuforLidaru[3] != 3) || (cBuforLidaru[4] != 4) || (cBuforLidaru[5] != 3))		//wersja 3.4.3
		cBłąd = BLAD_BRAK_CZUJNIKA;

	return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową lidaru
// Parametry: nic
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ObsługaI2C_LidaruBenewake(void)
{
	uint8_t cBłąd = BLAD_OK;

	//po MAX_PROB_INICJALIZACJI ustawiany jest bit braku czujnika. Taki czujnik nie jest dłużej obsługiwany
	if (uDaneCM4.dane.nBrakCzujnika & INIT_LIDAR_BENEWAKE)
		return BLAD_OK;		//czujnik jest opcjonalny, więc nie raportuj jego braku jako błąd

	if ((uDaneCM4.dane.nZainicjowano & INIT_LIDAR_BENEWAKE) != INIT_LIDAR_BENEWAKE)	//jeżeli czujnik nie jest zainicjowany
	{
		cLicznikPróbInicjalizacji++;
		cBłąd = InicjujLidarI2C();
		if (cBłąd)
		{
			if (cLicznikPróbInicjalizacji > MAX_PROB_INICJALIZACJI)
				uDaneCM4.dane.nBrakCzujnika |= INIT_LIDAR_BENEWAKE;
			return cBłąd;
		}
		uDaneCM4.dane.nZainicjowano |= INIT_LIDAR_BENEWAKE;
	}
	else	//czujnik jest zainicjowany
	{
		//dokumentacja sugeruje przerwę 100 ms między wysłaniem polecenia pomiaru a jego odczytem więc stosuję sekwencję: polecenie - odczyt z przerwą
		switch(cSekwencjaPomiaruBenewake)
		{
		case SB_WYSLIJ_POLECENIE:	//Pierwsza część: wysłanie polecenia odczytu.
			cBuforLidaru[0] = NAGLOWEK_BENEWAKE;
			cBuforLidaru[1] = 0x05;	//rozamiar ramki wychodzącej
			cBuforLidaru[2] = 0x00;
			cBuforLidaru[3] = 0x01;
			cBuforLidaru[4] = 0x60;	//suma kontrolna
			cBłąd = HAL_I2C_Master_Transmit_IT(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 5);	//wyślij polecenie odczytu pomiarów nie kończąc transferu STOP-em
			sCzujnikZapisywanyNaI2CExt = LIDAR_BENEWAKE;
			break;

		case SB_CZYTAJ_DANE:
			sCzujnikOdczytywanyNaI2CExt = LIDAR_BENEWAKE;
			cBłąd = HAL_I2C_Master_Receive_IT(&hi2c3, ADRES_I2C_BENEWAKE, cBuforLidaru, 9);
			break;
		}
		cSekwencjaPomiaruBenewake++;
		cSekwencjaPomiaruBenewake &= 0x03;
	}
	return cBłąd;
}

