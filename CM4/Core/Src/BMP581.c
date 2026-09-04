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
//static uint8_t chProporcjaPomiarow;
static float fP0_BMP581 = 0.0f;	//ciśnienie P0 do obliczeń wysokości [Pa]
static uint8_t cBufBMP581[4];
static uint16_t sLicznikUsrednianiaP0 = 0;			//licznik uśredniania ciśnienia zerowego do obliczeń wysokości
static float fWysokośćUśredniona;		//średnia z ostatnich pomiarów wysokości potrzebna do liczenia wariometru
static uint32_t nCzasOstatniejKonwersjiBMP581;



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujBMP581(void)
{
	HAL_Delay(2);		//Power-up time 2ms - czas jaki musi upłynąć od pojawienia się VCC do pierwszej komunikacji
	cBufBMP581[0] = CzytajSPIu8(BMP5_REG_CHIP_ID);	//sprawdź obecność układu
	if (cBufBMP581[0] != 0x50)
		return BLAD_BRAK_CZUJNIKA;

	cBufBMP581[0] = BMP5_REG_DRIVE_CONFIG;
	cBufBMP581[1] = 0x00;
	ZapiszSPIu8(cBufBMP581, 2);

	cBufBMP581[0] = CzytajSPIu8(BMP5_REG_CHIP_STATUS);	//sprawdź status konfiguracji magistrali
	if (cBufBMP581[0] != 0x03)							//SPI MODE0 lub MODE3
		return BLAD_BRAK_CZUJNIKA;

	cBufBMP581[0] = BMP5_REG_ODR_CONFIG;
	cBufBMP581[1] = (0 << 0);	//pwr_mode: 0=standby,
	ZapiszSPIu8(cBufBMP581, 2);

	cBufBMP581[0] = BMP5_REG_OSR_CONFIG;
	/*cBufBMP581[1] = (1 << 0)|	//oversampling temperatury: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
			 	 	 (2 << 3)|	//oversampling ciśnienia: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
					 (1 << 6);	//enable pressure sensor measurements*/
	cBufBMP581[1] = (2 << 0)|	//oversampling temperatury: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
				 	 (3 << 3)|	//oversampling ciśnienia: 0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32, 6=x64, 7=x128
					 (1 << 6);	//enable pressure sensor measurements
	ZapiszSPIu8(cBufBMP581, 2);

	cBufBMP581[0] = BMP5_REG_ODR_CONFIG;
	cBufBMP581[1] = (1 << 0)|	//pwr_mode: 0=standby, 1=normal mode in configured ODR grid, 2=forced one time mode measurement, 3=non stop mode, measurement without further duty cycling
					 (15 << 2)|	//ODR: 0=240Hz, 1=218,5Hz, 2=199,11Hz, 3=179,2Hz, 4=160Hz,, A=100,3Hz; B=89,6; C=80; D=70; E=60; F=50
					 (1 << 7);	//deep_dis - disable deep standby
	ZapiszSPIu8(cBufBMP581, 2);

	cBufBMP581[0] = BMP5_REG_DSP_CONFIG;
	cBufBMP581[1] = (3 << 0)|	//kompensacja ciśnienia i temepratury: 0=bez kompensacji ciśnienia i temepratury; 1=komp. temepratury; 2=3=kompensacja ciśnienia i temperatury
			 	 	 (1 << 2)|	//IIR flush in FORCED mode
					 (1 << 3)|	//Temperature Data Registers IIR selection temeprature data: 0=before IIR filter; 1=after IIR filter
					 (1 << 4)|	//FIFO IIR selection temperature data: 0=before IIR filter; 1=after IIR filter
					 (1 << 5)|	//Shadow Registers IIR selection pressure data: 0=before IIR filter; 1=after IIR filter
					 (1 << 6)|	//FIFO IIR selection pressure data: 0=before IIR filter; 1=after IIR filter
					 (1 << 7);	//Out Of Range IIR selection: 0=before IIR filter; 1=after IIR filter
	ZapiszSPIu8(cBufBMP581, 2);


	cBufBMP581[0] = BMP5_REG_DSP_IIR;
	cBufBMP581[1] = (3 << 0)|	//Pressure IIR band filter selection: 0=Baypass; 1=1; 2=3; 3=7; 4=15; 5=31; 6=63; 7=127
			 	 	 (2 << 3)|	//Temperature IIR band filter selection:
					 (0 << 6);	//reserved
	ZapiszSPIu8(cBufBMP581, 2);

	//ustaw źródło przerwania
	cBufBMP581[0] = BMP5_REG_INT_SOURCE;
	cBufBMP581[1] = (1 << 0)|	//data ready
					 (0 << 1)|	//fifo full
					 (0 << 2)|	//fifo threshold
					 (0 << 3);	//pressure data out of range
	ZapiszSPIu8(cBufBMP581, 2);

	/*cBufBMP581[0] = BMP5_REG_FIFO_SEL;
	cBufBMP581[1] = (3 << 0)|	//FIFO frame sel: 0=FIFO not enabled; 1=Temperature data; 2=Pressure data; 3 pressure and temperature data
					 (0 << 2);	//FIFO decimation level
	ZapiszSPIu8(cBufBMP581, 2);*/


	HAL_Delay(4);		//Re-configuration time 4ms

	sLicznikUsrednianiaP0 = LICZBA_PROBEK_USREDNIANIA;	//rozpocznij filtrowanie P0
	uDaneCM4.dane.nZainicjowano |= INIT_BMP581;
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaBMP581(void)
{
	uint8_t cBłąd = BLAD_OK;
	int32_t nWartosc[2];
	uint32_t nCzas, nDeltaCzasu;

	if ((uDaneCM4.dane.nZainicjowano & INIT_BMP581) != INIT_BMP581)	//jeżeli czujnik nie jest zainicjowany
	{
		cBłąd = InicjujBMP581();
		if (cBłąd)
			return cBłąd;
		nCzasOstatniejKonwersjiBMP581 = PobierzCzasT7();
	}
	else	//czujnik jest zainicjowany
	{
		uint8_t cDane = CzytajSPIu8(BMP5_REG_INT_STATUS);	//sprawdź status pomiaru
		if ((cDane & 0x01) != 0x01)		//drdy_data_reg = Data Ready
			return BLAD_ZA_KROTKI_CZAS;

		//konwersja miała szansę się zakonczyć, więc oczytaj pomiar i uruchom następny
		nCzas = PobierzCzasT7();
		nDeltaCzasu = MinalCzas2T7(nCzasOstatniejKonwersjiBMP581, nCzas);
		nCzasOstatniejKonwersjiBMP581 = nCzas;

		CzytajBuforSPIsmp(BMP5_REG_TEMP_DATA_XLSB, nWartosc, 2);	//odczyt z rejestrów
		//CzytajBuforSPIsmp(BMP5_REG_FIFO_DATA, nWartosc, 2);			//odczyt z FIFO
		uDaneCM4.dane.fTemper[TEMP_BARO2] = (7 * uDaneCM4.dane.fTemper[TEMP_BARO2] + ((float)nWartosc[0] / 65536.0f) + KELVIN) / 8;
		uDaneCM4.dane.fCisnieBzw[1] = (float)nWartosc[1] / 64.0f;

		uDaneCM4.dane.fWysokoMSL[1] = WysokoscBarometryczna(uDaneCM4.dane.fCisnieBzw[1], CISNIENIE_QNE, uDaneCM4.dane.fTemper[TEMP_BARO2]);	//wartość bwzezględna, nie wymaga uśredniania P0
		uDaneCM4.dane.cNowyPomiar |= NP_WYS2;
		fWysokośćUśredniona = ((PODSTAWA_FILTRA_IIR_WARIOMETRU - 1) * fWysokośćUśredniona + uDaneCM4.dane.fWysokoMSL[1]) / PODSTAWA_FILTRA_IIR_WARIOMETRU;

		//przygotuj P0
		if (sLicznikUsrednianiaP0)	//czy przygotowanie ciśnienia P0 jeszcze trwa
		{
			fP0_BMP581 = ((PODSTAWA_FILTRA_IIR_P0 - 1) * fP0_BMP581 + uDaneCM4.dane.fCisnieBzw[1]) / PODSTAWA_FILTRA_IIR_P0;
			sLicznikUsrednianiaP0--;
			if (sLicznikUsrednianiaP0 == 0)
			{
				uDaneCM4.dane.nZainicjowano |= INIT_P0_BMP851;
			}
		}
		else
		{
			uDaneCM4.dane.fWysokoAGL[1] = WysokoscBarometryczna(uDaneCM4.dane.fCisnieBzw[1], fP0_BMP581, uDaneCM4.dane.fTemper[TEMP_BARO2]);	//P0 gotowe więc oblicz wysokość
			if ((nDeltaCzasu > 0) && (nDeltaCzasu < 10000))	//nie licz dla zera i długich przestoi, bo to generuje dużą szpilkę danych
				uDaneCM4.dane.fWariometr[1] = (uDaneCM4.dane.fWysokoMSL[1] - fWysokośćUśredniona) * 1000 * KOREKTA_SKALI_FILTRA_WARIOMETRU_BMP581 / nDeltaCzasu;	//dH [m] * 1e3 / t [1e-6 s]
		}

		cBufBMP581[0] = CzytajSPIu8(BMP5_REG_OSR_EFF);

		cBufBMP581[1] = CzytajSPIu8(0x20);
		cBufBMP581[2] = CzytajSPIu8(0x21);
		cBufBMP581[3] = CzytajSPIu8(0x22);

	}
	return cBłąd;
}
