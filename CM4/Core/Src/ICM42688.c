//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika żyroskopów i akcelerometrów ICM-42688-V na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "ICM42688.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"
#include "petla_glowna.h"
#include "modul_IiP.h"
#include "spi.h"

extern volatile unia_wymianyCM4_t uDaneCM4;
extern volatile unia_wymianyCM7_t uDaneCM7;
extern SPI_HandleTypeDef hspi2;
const int8_t chZnakZyro[3] = {-1, 1, -1};	//korekcja znaku prędkości żyroskopów
const int8_t chZnakAkcel[3] = {-1, 1, 1};	//korekcja znaku akcelerometrów
extern WspRownProstej3_t stWspKalTempZyro1;		//współczynniki równania prostych do estymacji przesunęcia zera w funkcji temperatury
extern float fSkaloZyro1[3];

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujICM42688(void)
{
	uint8_t chDane[2];

	chDane[0] = CzytajSPIu8(PICM4268_WHO_I_AM);		//sprawdź obecność układu
	if (chDane[0] != 0xDB)
		return ERR_BRAK_ICM42688;


	//włącz żyroskopu i akcelerometry w tryb Low Noise
	chDane[0] = PICM4268_PWR_MGMT0;
	chDane[1] = (0 << 5) |	//TEMP_DIS. 0: Temperature sensor is enabled (default), 1: Temperature sensor is disabled
				(0 << 4) |	//IDLE: If this bit is set to 1, the RC oscillator is powered on even if Accel and Gyro are powered off. Nominally this bit is set to 0, so when Accel and Gyro are powered off, the chip will go to OFF state, since the RC oscillator will also be powered off
				(3 << 2) |	//GYRO_MODE: 00: Turns gyroscope off (default), 01: Places gyroscope in Standby Mode, 10: Reserved, 11: Places gyroscope in Low Noise (LN) Mode
				(3 << 0);	//ACCEL_MODE: 00: Turns accelerometer off (default), 01: Turns accelerometer off, 10: Places accelerometer in Low Power (LP) Mode, 11: Places accelerometer in Low Noise (LN) Mode
	ZapiszSPIu8(chDane, 2);

	chDane[0] = PICM4268_GYRO_CONFIG0;
	chDane[1] = (1 << 5) |	//GYRO_FS_SEL: 000: ±2000 dps (default), 001: ±1000 dps, 010: ±500 dps, 011: ±250 dps, 100: ±125 dps, 101: ±62.5 dps, 110: ±31.25 dps, 111: ±15.625 dps
				(7 << 0);	//GYRO_ODR: 0000: Reserved, 0001: 32 kHz, 0010: 16 kHz, 0011: 8 kHz, 0100: 4 kHz, 0101: 2 kHz, 0110: 1 kHz (default), 0111: 200 Hz, 1000: 100 Hz, 1001: 50 Hz, 1010: 25 Hz, 1011: 12.5 Hz, 1100: Reserved, 1101: Reserved, 1110: Reserved, 1111: 500 Hz
	ZapiszSPIu8(chDane, 2);

	chDane[0] = PICM4268_ACCEL_CONGIG0;
	chDane[1] = (1 << 5) |	//ACCEL_FS_SEL: 000: ±16g (default), 001: ±8g, 010: ±4g, 011: ±2g, 100: Reserved, 101: Reserved, 110: Reserved, 111: Reserved
				(7 << 0);	//ACCEL_ODR: 0000: Reserved, 0001: 32 kHz (LN mode), 0010: 16 kHz (LN mode), 0011: 8 kHz (LN mode), 0100: 4 kHz (LN mode), 0101: 2 kHz (LN mode), 0110: 1 kHz (LN mode) (default), 0111: 200 Hz (LP or LN mode)
							//			 1000: 100 Hz (LP or LN mode), 1001: 50 Hz (LP or LN mode), 1010: 25 Hz (LP or LN mode), 1011: 12.5 Hz (LP or LN mode), 1100: 6.25 Hz (LP mode), 1101: 3.125 Hz (LP mode), 1110: 1.5625 Hz (LP mode), 1111: 500 Hz (LP or LN mode
	ZapiszSPIu8(chDane, 2);

	//przełącz na bank 1
	chDane[0] = PICM4268_BANK_SEL;
	chDane[0] = 1;
	ZapiszSPIu8(chDane, 2);

	//wyłącz filtry: nothch i AAF na ścieżce żyroskopów
	chDane[0] = PICM4268_B1_GYRO_CONFIG_STATIC2;
	chDane[1] = (1 << 1) |	//GYRO_AAF_DIS: 0: Enable gyroscope anti-aliasing filter (default), 1: Disable gyroscope anti-aliasing filter
				(1 << 0);	//GYRO_NF_DIS: 0: Enable Notch Filter (default), 1: Disable Notch Filter
	ZapiszSPIu8(chDane, 2);

	//przełącz na bank 2
	chDane[0] = PICM4268_BANK_SEL;
	chDane[0] = 2;
	ZapiszSPIu8(chDane, 2);

	//wyłącz filtr AAF na ścieżce skcelerometrów
	chDane[0] = PICM4268_B2_ACCEL_CONFIG_STATIC2;
	chDane[1] = (0 << 1) |	//ACCEL_AAF_DELT: Controls bandwidth of the accelerometer anti-alias filter
				(1 << 0);	//ACCEL_AAF_DIS: 0: Enable accelerometer anti-aliasing filter (default), 1: Disable accelerometer anti-aliasing filter
	ZapiszSPIu8(chDane, 2);
	//chDane[0] = CzytajSPIu8(PICM4268_B2_ACCEL_CONFIG_STATIC2);

	//przełącz na bank 0
	chDane[0] = PICM4268_BANK_SEL;
	chDane[0] = 0;
	ZapiszSPIu8(chDane, 2);

	uDaneCM4.dane.nZainicjowano |= INIT_ICM42688;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania: 24us @200MHz SPI@10MHz
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaICM42688(void)
{
	uint8_t chErr = ERR_OK;
	uint8_t chDane[15];
	float fPrzesuniecieZyro[3];

	if ((uDaneCM4.dane.nZainicjowano & INIT_ICM42688) != INIT_ICM42688)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujICM42688();
	}
	else	//odczytaj hurtem wszystkie 7 16-bitowych rejestrów
	{
		chDane[0] = PICM4268_TEMP_DATA1 | READ_SPI;
		HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
		HAL_SPI_TransmitReceive(&hspi2, chDane, chDane, 15, 5);
		HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

		uDaneCM4.dane.fTemper[TEMP_IMU1] = (float)((int16_t)((chDane[1] <<8) + chDane[2]) / 132.48) + 25.0 + KELVIN;	//temperatura w K
		ObliczWspTemperaturowy3(stWspKalTempZyro1, uDaneCM4.dane.fTemper[TEMP_IMU1], fPrzesuniecieZyro);			//oblicz przesuniecie zera dla bieżącej temperatury

		for (uint16_t n=0; n<3; n++)
		{
			uDaneCM4.dane.fAkcel1[n] = (float)((int16_t)(chDane[2*n+3] <<8) + chDane[2*n+4]) * (8.0 * AKCEL1G / 32768.0) * chZnakAkcel[n];			//+-8g -> [m/s^2]
			uDaneCM4.dane.fZyroSur1[n] = (float)((int16_t)(chDane[2*n+9] <<8) + chDane[2*n+10]) * (1000.0 * DEG2RAD / 32768.0) * chZnakZyro[n];	//+-1000°/s -> [rad/s]

			//w czasie kalibracji wzmocnienia nie uwzgledniej wzmocnienia, jedynie przesunięcie
			if ((uDaneCM7.dane.chWykonajPolecenie >= POL_CALKUJ_PRED_KAT)  && (uDaneCM7.dane.chWykonajPolecenie <= POL_KALIBRUJ_ZYRO_WZMP))
				uDaneCM4.dane.fZyroKal1[n] = uDaneCM4.dane.fZyroSur1[n] - fPrzesuniecieZyro[n];		//żyro po kalibracji przesuniecia
			else
				uDaneCM4.dane.fZyroKal1[n] = uDaneCM4.dane.fZyroSur1[n] * fSkaloZyro1[n] - fPrzesuniecieZyro[n];		//żyro po kalibracji przesuniecia i skalowania
		}
	}
	return chErr;
}
