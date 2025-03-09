//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa czujnika żyroskopów i akcelerometrów LSM6DSV na magistrali SPI
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "LSM6DSV.h"
#include "wymiana_CM4.h"
#include "petla_glowna.h"
#include "main.h"
#include "modul_IiP.h"
#include "spi.h"

extern volatile unia_wymianyCM4_t uDaneCM4;
extern SPI_HandleTypeDef hspi2;
extern float fOffsetZyro2[3];
const int8_t chZnakZyro2[3] = {-1, 1, -1};	//korekcja znaku prędkości żyroskopów
extern WspRownProstej_t stWspKalOffsetuZyro2;		//współczynniki równania prostych do estymacji offsetu
//float fZyroSur2[3];		//surowe nieskalibrowane prędkosci odczytane z żyroskopu 2

////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizację czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujLSM6DSV(void)
{
	uint8_t chDane[2];

	chDane[0] = CzytajSPIu8(PLSC6DSV_WHO_AM_I);		//sprawdź obecność układu
	if (chDane[0] != 0x70)
		return ERR_BRAK_LSM6DSV;


	chDane[0] = PLSC6DSV_CTRL1;
	chDane[1] = (1 << 4) |	//OP_MODE_XL_[2:0]: Accelerometer operating mode selection: (000: high-performance mode (default); 001: high-accuracy ODR mode; 010: reserved; 	011: ODR-triggered mode; 100: low-power mode 1 (2 mean); 101: low-power mode 2 (4 mean); 110: low-power mode 3 (8 mean); 111: normal mode)
				(7 << 0);	//ODR_XL_[3:0]: 0=Power-down (default); 1=1.875 Hz (low-power mode); 2=7.5 Hz (high-performance, normal mode), 3=15 Hz (low-power, high-performance, normal mode), 4=30 Hz (low-power, high-performance, normal mode)
							//5=60 Hz (low-power, high-performance, normal mode); 6=120 Hz (low-power, high-performance, normal mode), 7=240 Hz (low-power, high-performance, normal mode), 8=480 Hz (high-performance, normal mode)
							//9=960 Hz (high-performance, normal mode), 10=1.92 kHz (high-performance, normal mode), 11=3.84 kHz (high-performance mode, 12=7.68 kHz (high-performance mode)
	ZapiszSPIu8(chDane, 2);

	chDane[0] = PLSC6DSV_CTRL2;
	chDane[1] = (1 << 4) |	//OP_MODE_G_[2:0]: Gyroscope operating mode selection: (000: high-performance mode (default); 001: high-accuracy ODR mode; 010: reserved; 	011: ODR-triggered mode; 100: sleep mode; 101: low-power mode; 110-111: reserved)
				(7 << 0);	//ODR_G_[3:0]: 0=Power-down (default); 2=7.5 Hz (low-power, high-performance mode), 3=15 Hz (low-power, high-performance mode), 4=30 Hz (low-power, high-performance mode), 5=60 Hz (low-power, high-performance mode)
							//6=120 Hz (low-power, high-performance mode); 7=240 Hz (low-power, high-performance mode); 8=480 Hz (high-performance mode), 9=960 Hz (high-performance mode)
							//9=1.92 kHz (high-performance mode), 10=1.92 kHz (high-performance mode), 11=3.84 kHz (high-performance mode); 12=7.68 kHz (high-performance mode)
	ZapiszSPIu8(chDane, 2);

	chDane[0] = PLSC6DSV_CTRL6;
	chDane[1] = (3 << 4) |	//LPF1_G_BW_[2:0]: Gyroscope low-pass filter (LPF1) bandwidth selection: 0-3=96Hz
				(3 << 0);	//FS_G_[3:0]: Gyroscope UI chain full-scale selection: 	(0000: ±125 dps (default);	1: ±250 dps; 2: ±500 dps; 3: ±1000 dps; 4: ±2000 dps; 5: ±4000 dps(1)
	ZapiszSPIu8(chDane, 2);

	chDane[0] = PLSC6DSV_CTRL7;
	chDane[1] = (1 << 0);	//LPF1_G_EN, Enables the gyroscope digital LPF1 filter. If the OIS chain is disabled, the bandwidth can be selected through LPF1_G_BW_[2:0] in CTRL6 (15h)
	ZapiszSPIu8(chDane, 2);

	chDane[0] = PLSC6DSV_CTRL8;
	chDane[1] = (1 << 5) |	//HP_LPF2_XL_BW_[2:0] Accelerometer LPF2 and HP filter configuration and cutoff setting: 0=ODR/4, 1=ODR/10, 2=ODR/20, 3=ODR/45, 4=ODR/100, 5=ODR/200; 6=ODR/400, 7=ODR/800
				(0 << 3) |	//XL_DualC_EN. Enables dual-channel mode. When this bit is set to 1, data with the maximum full scale are sent to the output registers at addresses 34h to 39h. The UI processing chain is used. Default value: 0 (0: disabled; 1: enabled)
				(2 << 0);	//FS_XL_[1:0] Accelerometer full-scale selection: 0: ±2 g; 1: ±4 g; 2: ±8 g; 3: ±16 g)
	ZapiszSPIu8(chDane, 2);

	chDane[0] = PLSC6DSV_CTRL9;
	chDane[1] = (0 << 6) |	//HP_REF_MODE_XL Enables accelerometer high-pass filter reference mode (valid for high-pass path - HP_SLOPE_XL_EN bit must be 1). Default value: 0 (0: disabled, 1: enabled
				(0 << 5) |	//XL_FASTSETTL_MODE Enables accelerometer LPF2 and HPF fast-settling mode. The filter sets the first sample after writing this bit. Active only during device exit from power-down mode. Default value: 0 (0: disabled, 1: enabled)
				(0 << 4) |	//HP_SLOPE_XL_EN Accelerometer slope filter / high-pass filter selection. Refer to Figure 30. Default value: 0 	(0: low-pass filter path selected; 1: high-pass filter path selected)
				(1 << 3) |	//LPF2_XL_EN Accelerometer high-resolution selection. Refer to Figure 30. Default value: 0 (0: output from first stage digital filtering selected; 1: output from LPF2 second filtering stage selected)
				(1 << 1) |	//USR_OFF_W Weight of XL user offset bits of registers X_OFS_USR (73h), Y_OFS_USR (74h), Z_OFS_USR (75h). Default value: 0 (0: 2-10 g/LSB; 1: 2-6 g/LSB)
				(0 << 0);	//USR_OFF_ON_OUT Enables accelerometer user offset correction block; it is valid for the low-pass path. Refer to Figure 30. Default value: 0 (0: accelerometer user offset correction block bypassed; 1: accelerometer user offset correction block enabled)
	ZapiszSPIu8(chDane, 2);

	uDaneCM4.dane.nZainicjowano |= INIT_LSM6DSV;
	return ERR_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Realizuje sekwencję obsługową czujnika do wywołania w wyższej warstwie
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania: 20us @200MHz SPI@10MHz
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaLSM6DSV(void)
{
	uint8_t chErr = ERR_OK;
	uint8_t chDane[15];

	if ((uDaneCM4.dane.nZainicjowano & INIT_LSM6DSV) != INIT_LSM6DSV)	//jeżeli czujnik nie jest zainicjowany
	{
		chErr = InicjujLSM6DSV();
	}
	else
	{
		chDane[0] = PLSC6DSV_OUT_TEMP_L | READ_SPI;
		HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_RESET);	//CS = 0
		HAL_SPI_TransmitReceive(&hspi2, chDane, chDane, 15, 5);
		HAL_GPIO_WritePin(MOD_SPI_NCS_GPIO_Port, MOD_SPI_NCS_Pin, GPIO_PIN_SET);	//CS = 1

		uDaneCM4.dane.fTemper[TEMP_IMU2] = (int16_t)((chDane[2] <<8) + chDane[1]) / 256.0f + 25.0f + KELVIN;		//temperatura w K
		ObliczOffsetTemperaturowyZyro(stWspKalOffsetuZyro2, uDaneCM4.dane.fTemper[TEMP_IMU2], fOffsetZyro2);		//oblicz offset dla bieżącej temperatury

		for (uint16_t n=0; n<3; n++)
		{
			uDaneCM4.dane.fAkcel2[n] = (float)((int16_t)(chDane[2*n+10] <<8) + chDane[2*n+9]) * (8.0 / 32768.0);		//+-8g
			uDaneCM4.dane.fZyroSur2[n] = (float)((int16_t)(chDane[2*n+4] <<8)  + chDane[2*n+3]) * (10000.0 / 32768.0) * chZnakZyro2[n];	//+-1000°/s
			uDaneCM4.dane.fZyroKal2[n] = uDaneCM4.dane.fZyroSur2[n] - fOffsetZyro2[n];	//żyro po kalibracji offsetu
		}
	}
	return chErr;
}


