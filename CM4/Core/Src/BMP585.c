//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika ciśnienia BMP585 na magistrali SPI
// Z góry zakładam czas konwersji z uśrednianiem 4 pomiarów ciśnienia wynoszący maksymalnie 2,9 + 5% = 3,045 ms.
// Jeżeli polecenie wykonania pomiaru uruchamiane jest szybciej niż mogla zakonczyć sie
// konwersja, to takie polecenie wychodzi z kodem błędu BLAD_ZA_KROTKI_CZAS
//
// (c) Pit Lab 2026
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "BMP585.h"
#include "WymianaCM4.h"
#include "PetlaGlowna.h"
#include "spi.h"
#include "main.h"
#include "Modul_I2P.h"
#include "Czas.h"

// Dopuszczalna prędkość magistrali 1..12MHz
extern SPI_HandleTypeDef hspi2;
extern volatile unia_wymianyCM4_t uDaneCM4;
static uint8_t chBufBMP585[4];
static uint16_t sLicznikUsrednianiaP0 = 0;			//licznik uśredniania ciśnienia zerowego do obliczeń wysokości
static float fWysokośćUśredniona;		//średnia z ostatnich pomiarów wysokości potrzebna do liczenia wariometru
static uint32_t nCzasOstatniejKonwersjiBMP585;
static float fP0_BMP585 = 0.0f;	//ciśnienie P0 do obliczeń wysokości [Pa]



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujBMP585(void)
{
	uint8_t chDane;

	HAL_Delay(2);		//Power-up time 2ms - czas jaki musi upłynąć od pojawienia się VCC do pierwszej komunikacji
	chDane = CzytajSPIu8(BMP5_REG_CHIP_ID);	//sprawdź obecność układu
	if (chDane != 0x51)
		return BLAD_BRAK_CZUJNIKA;

	chDane = CzytajSPIu8(BMP5_REG_CHIP_STATUS);	//sprawdź status konfiguracji magistrali. Musi być ustawiona na SPI
	if (chDane != 0x03)							//SPI MODE0 lub MODE3
		return BLAD_BRAK_CZUJNIKA;

	//ustaw standby bo w takim trybie powinna być robiona konfiguracja
	chBufBMP585[0] = BMP5_REG_ODR_CONFIG;
	chBufBMP585[1] = (0 << 0)|	//pwr_mode: 0=standby, 1=normal mode in configured ODR grid, 2=forced one time mode measurement, 3=non stop mode, measurement without further duty cycling
					 (0 << 2)|	//ODR: 0=240Hz, 1=218,5Hz, 2=199,11Hz, 3=179,2Hz, 4=160Hz,, A=100,3Hz
					 (1 << 7);	//deep_dis - disable deep standby
	ZapiszSPIu8(chBufBMP585, 2);

	chBufBMP585[0] = BMP5_REG_OSR_CONFIG;
	chBufBMP585[1] = (4 << 0)|	//osr_t oversampling rate: 0=1x, 1=2x, 2=4x, 3=8x, 4=16x, 5=32x, 6=64x, 7=128x
					 (4 << 3)|	//osr_p overdampling dla ciśnienia - tak samo jak dla temperatury
					 (1 << 6);	//press_en
	ZapiszSPIu8(chBufBMP585, 2);

	chBufBMP585[0] = BMP5_REG_DSP_CONFIG;
	chBufBMP585[1] = (1 << 0)|	//reserved 01
			 	 	 (1 << 2)|	//IIR flush in FORCED mode
					 (0 << 3)|	//Temperature Data Registers IIR selection temeprature data: 0=before IIR filter; 1=after IIR filter
					 (0 << 4)|	//FIFO IIR selection temperature data: 0=before IIR filter; 1=after IIR filter
					 (0 << 5)|	//Shadow Registers IIR selection pressure data: 0=before IIR filter; 1=after IIR filter
					 (0 << 6)|	//FIFO IIR selection pressure data: 0=before IIR filter; 1=after IIR filter
					 (0 << 7);	//Out Of Range IIR selection: 0=before IIR filter; 1=after IIR filter
	ZapiszSPIu8(chBufBMP585, 2);

	//ustaw finalny tryb pracy
	chBufBMP585[0] = BMP5_REG_ODR_CONFIG;
	chBufBMP585[1] = (1 << 0)|	//pwr_mode: 0=standby, 1=normal mode in configured ODR grid, 2=forced one time mode measurement, 3=non stop mode, measurement without further duty cycling
					 (0 << 2)|	//ODR: 0=240Hz, 1=218,5Hz, 2=199,11Hz, 3=179,2Hz, 4=160Hz,, A=100,3Hz
					 (1 << 7);	//deep_dis - disable deep standby
	ZapiszSPIu8(chBufBMP585, 2);

	//ustaw źródło przerwania
	chBufBMP585[0] = BMP5_REG_INT_SOURCE;
	chBufBMP585[1] = (1 << 0)|	//data ready
					 (0 << 1)|	//fifo full
					 (0 << 2)|	//fifo threshold
					 (0 << 3);	//pressure data out of range
	ZapiszSPIu8(chBufBMP585, 2);

	HAL_Delay(4);		//Re-configuration time 4ms

	sLicznikUsrednianiaP0 = LICZBA_PROBEK_USREDNIANIA;	//rozpocznij filtrowanie P0
	return BLAD_OK;
}




uint8_t ObslugaBMP585(void)
{
	uint8_t cBłąd = BLAD_OK;
	int32_t nWartosc[2];

	//Ponieważ zegar SPI = 40MHz a układy mogą pracować z prędkością max 10MHz, przy każdym dostępie przestaw dzielnik zegara na 4
//	nZastanaKonfiguracja_SPI_CFG1 = hspi2.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI
	hspi2.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;		//maska preskalera
	hspi2.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_4;


	if ((uDaneCM4.dane.nZainicjowano & INIT_BMP585) != INIT_BMP585)	//jeżeli czujnik nie jest zainicjowany
	{
		cBłąd = InicjujBMP585();
		if (cBłąd)
			return cBłąd;
		uDaneCM4.dane.nZainicjowano |= INIT_BMP585;
		nCzasOstatniejKonwersjiBMP585 = PobierzCzasT7();
	}
	else
	{
		//sprawdź ile czasu upłyneło od ostatniego pomiaru. Jeżeli było to mniej niż czas potrzebny na konwersję to pomiń to uruchomienie
		//uint32_t nCzas = MinalCzasT7(nCzasOstatniejKonwersjiBMP585);
		//if (nCzas < 4375)	//ODR = 240Hz -> 4,16ms +5%  = 4,375
			//return BLAD_ZA_KROTKI_CZAS;

		uint8_t cDane = CzytajSPIu8(BMP5_REG_INT_STATUS);	//sprawdź status pomiaru
		if ((cDane & 0x01) == 0x00)		//drdy_data_reg = Data Ready
			return BLAD_ZA_KROTKI_CZAS;


		//konwersja miała szansę się zakonczyć, więc oczytaj pomiar i uruchom następny
		nCzasOstatniejKonwersjiBMP585 = PobierzCzasT7();

		CzytajBuforSPIsmp(BMP5_REG_TEMP_DATA_XLSB, nWartosc, 2);	//odczyt rejestrów temperatury i ciśnienia

		uDaneCM4.dane.fTemper[TEMP_BARO2] = (7 * uDaneCM4.dane.fTemper[TEMP_BARO2] + ((float)nWartosc[0] / 65536.0f) + KELVIN) / 8;
		uDaneCM4.dane.fCisnieBzw[1] = (float)nWartosc[1] / 64.0f;

		uDaneCM4.dane.fWysokoMSL[1] = WysokoscBarometryczna(uDaneCM4.dane.fCisnieBzw[1], CISNIENIE_QNE, uDaneCM4.dane.fTemper[TEMP_BARO2]);	//wartość bwzezględna, nie wymaga uśredniania P0
		uDaneCM4.dane.cNowyPomiar |= NP_WYS2;
		fWysokośćUśredniona = ((PODSTAWA_FILTRA_IIR_WARIOMETRU - 1) * fWysokośćUśredniona + uDaneCM4.dane.fWysokoMSL[1]) / PODSTAWA_FILTRA_IIR_WARIOMETRU;

		//przygotuj P0
		if (sLicznikUsrednianiaP0)	//czy przygotowanie ciśnienia P0 jeszcze trwa
		{
			fP0_BMP585 = ((PODSTAWA_FILTRA_IIR_P0 - 1) * fP0_BMP585 + uDaneCM4.dane.fCisnieBzw[1]) / PODSTAWA_FILTRA_IIR_P0;
			sLicznikUsrednianiaP0--;
			if (sLicznikUsrednianiaP0 == 0)
			{
				uDaneCM4.dane.nZainicjowano |= INIT_P0_BMP855;
			}
		}
		else
		{
			uDaneCM4.dane.fWysokoAGL[1] = WysokoscBarometryczna(uDaneCM4.dane.fCisnieBzw[1], fP0_BMP585, uDaneCM4.dane.fTemper[TEMP_BARO2]);	//P0 gotowe więc oblicz wysokość
			if ((uDaneCM4.dane.ndT > 0) && (uDaneCM4.dane.ndT < 10000))	//nie licz dla zera i długich przestoi, bo to generuje dużą szpilkę danych
				uDaneCM4.dane.fWariometr[1] = (uDaneCM4.dane.fWysokoMSL[1] - fWysokośćUśredniona) * 1000 * KOREKTA_SKALI_FILTRA_WARIOMETRU / uDaneCM4.dane.ndT;	//dH [m] * 1e3 / t [1e-6 s]
		}

	}
	return cBłąd;
}
