//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika ciśnienia BMP581 na magistrali SPI
// Z góry zakładam czas konwersji z uśrednianiem 4 pomiarów ciśnienia wynoszący maksymalnie 2,9 + 5% = 3,045 ms.
// Jeżeli polecenie wykonania pomiaru uruchamiane jest szybciej niż mogla zakonczyć sie
// konwersja, to takie polecenie wychodzi z kodem błędu BLAD_ZA_KROTKI_CZAS
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "BMP581.h"
#include "WymianaCM4.h"
#include "PetlaGlowna.h"
#include "spi.h"
#include "main.h"
#include "Modul_I2P.h"
#include "Czas.h"

// Dopuszczalna prędkość magistrali 1..12MHz
extern SPI_HandleTypeDef hspi2;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chProporcjaPomiarow;
static float fP0_BMP581 = 0.0f;	//ciśnienie P0 do obliczeń wysokości [Pa]
static uint8_t chBufBMP581[4];
static uint16_t sLicznikUsrednianiaP0 = 0;			//licznik uśredniania ciśnienia zerowego do obliczeń wysokości
static float fWysokoscUsredniona;		//średnia z ostatnich pomiarów wysokości potrzebna do liczenia wariometru
static uint32_t nCzasOstatniejKonwersjiBMP581;



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujBMP581(void)
{
	uint8_t chDane;

	/*HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
	chBufBMP581[0] = PBMP5_CHIP_ID | READ_SPI;
	//chBufBMP581[0] = PBMP5_REV_ID | READ_SPI;
	chBufBMP581[1] = 0;
	HAL_SPI_TransmitReceive(&hspi2, chBufBMP581, chBufBMP581, 2, 5);
	HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1
	chDane = chBufBMP581[1];*/

	chDane = CzytajSPIu8(PBMP5_CHIP_ID);	//sprawdź obecność układu
	if (chDane != 0x50)
		return BLAD_BRAK_CZUJNIKA;

	chBufBMP581[0] = PBMP5_DRIVE_CONFIG;
	chBufBMP581[1] = 0x00;
	ZapiszSPIu8(chBufBMP581, 2);

	chDane = CzytajSPIu8(PBMP5_CHIP_STATUS);	//sprawdź status konfiguracji magistrali
	if (chDane != 0x03)							//SPI MODE0 lub MODE3
		return BLAD_BRAK_CZUJNIKA;

	chBufBMP581[0] = PBMP5_OVERSAMLING_RATE;
	chBufBMP581[1] = (3 << 0)|	//oversampling temperatury: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
					 (2 << 3)|	//oversampling ciśnienia: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
					 (1 << 6);	//enable pressure sensor measurements
	ZapiszSPIu8(chBufBMP581, 2);

	chBufBMP581[0] = PBMP5_OUTPUT_DATA_RATE;
	chBufBMP581[1] = (1 << 0)|	//pwr_mode: 0=standby, 1=normal mode in configured ODR grid, 2=forced one time mode measurement, 3=non stop mode, measurement wothout further duty cycling
					 (1 << 2)|	//ODR: 0=240Hz, 1=218,5Hz, 2=199,11Hz, 3=179,2Hz, 4=160Hz,, A=100,3Hz
					 (1 << 7);	//deep_dis - disable deep standby
	ZapiszSPIu8(chBufBMP581, 2);

	sLicznikUsrednianiaP0 = LICZBA_PROBEK_USREDNIANIA;	//rozpocznij filtrowanie P0
	uDaneCM4.dane.nZainicjowano |= INIT_BMP581;
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Wykonuje w pętli 1 pomiar temepratury i 15 pomiarów ciśnienia
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaBMP581(void)
{
	uint8_t cBłąd = BLAD_OK;
	float fCisnienie = 0;

	if ((uDaneCM4.dane.nZainicjowano & INIT_BMP581) != INIT_BMP581)	//jeżeli czujnik nie jest zainicjowany
	{
		cBłąd = InicjujBMP581();
		if (cBłąd)
			return cBłąd;
		nCzasOstatniejKonwersjiBMP581 = PobierzCzasT7();
	}
	else	//czujnik jest zainicjowany
	{

		//sprawdź ile czasu upłyneło od ostatniego pomiaru. Jeżeli było to mniej niż czas potrzebny na konwersję to pomiń to uruchomienie
		uint32_t nCzas = MinalCzasT7(nCzasOstatniejKonwersjiBMP581);
		if (nCzas < 3045)
			return BLAD_ZA_KROTKI_CZAS;

		//konwersja miała szansę się zakonczyć, więc oczytaj pomiar i uruchom następny
		nCzasOstatniejKonwersjiBMP581 = PobierzCzasT7();
		switch (chProporcjaPomiarow)
		{
		case 0:	uDaneCM4.dane.fTemper[TEMP_BARO2] = (7 * uDaneCM4.dane.fTemper[TEMP_BARO2] + (float)(CzytajSPIs24mp(PBMP5_TEMP_DATA_XLSB) / 65536) + KELVIN) / 8;	break;
		default:
			fCisnienie = (float)(CzytajSPIs24mp(PBMP5_PRESS_DATA_XLSB) / 64);
			uDaneCM4.dane.fCisnieBzw[1] = (7 * uDaneCM4.dane.fCisnieBzw[1] + fCisnienie) / 8;
			break;
		}
		chProporcjaPomiarow++;
		chProporcjaPomiarow &= 0x0F;

		//przygotuj P0
		if (fCisnienie > 0)		//wykonaj tylko dla cykli pomiaru ciśnienia, pomiń cykle pomiary tempertury
		{
			if (sLicznikUsrednianiaP0)	//czy przygotowanie ciśnienia P0 jeszcze trwa
			{
				uDaneCM4.dane.fWysokoMSL[1] = 0.0f;
				fP0_BMP581 = (127 * fP0_BMP581 + fCisnienie) / 128;
				sLicznikUsrednianiaP0--;
				if (sLicznikUsrednianiaP0 == 0)
					uDaneCM4.dane.nZainicjowano |= INIT_P0_BMP851;
			}
			else
			{
				uDaneCM4.dane.fWysokoMSL[1] = WysokoscBarometryczna(uDaneCM4.dane.fCisnieBzw[1], fP0_BMP581, uDaneCM4.dane.fTemper[TEMP_BARO2]);	//P0 gotowe więc oblicz wysokość
				fWysokoscUsredniona = (1023 * fWysokoscUsredniona + uDaneCM4.dane.fWysokoMSL[1]) / 1024;
				float fWariometr = (fWysokoscUsredniona - uDaneCM4.dane.fWysokoMSL[1]) * 1000 / uDaneCM4.dane.ndT;	//dH [m] * 1e3 / t [1e-6 s]
				uDaneCM4.dane.fWariometr[1] = (999 * uDaneCM4.dane.fWariometr[1] + fWariometr) / 1000;				//dH / 1e-3
			}
		}
	}
	return cBłąd;
}
