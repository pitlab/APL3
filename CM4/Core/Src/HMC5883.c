//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Obsługa magnetometru HMC5883 na magistrali I2C
//
// (c) Pit Lab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "HMC5883.h"
#include "wymiana_CM4.h"

//definicje zmiennych

extern volatile unia_wymianyCM4_t uDaneCM4;
extern I2C_HandleTypeDef hi2c3;
uint8_t chDaneMagHMC[6];

// Obsługa magnetometru wymaga wykonania kilku czynności rozłożonych w czasie
// 1) Wystartowanie konwersji trwającej 6ms
// 2) Wystartowanie odczytu wykonanego pomiaru. Odczyt wykonuje się w tle w procedurze przerwania I2C i
//    trwa ok. 4ms
// 3) Przepisania odczytanych w przerwaniu danych do zmiennych pomiarowych

//Czas od rozpoczęcia pomiaru do gotowych danych 6ms

////////////////////////////////////////////////////////////////////////////////
// Budzi układ i startuje konwersję magnetometru HMC5883
// Dane powinny pojawią się 6ms po starcie konwersji
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 760us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujPomiarMagHMC(void)
{
	chDaneMagHMC[0] = MODE;
	chDaneMagHMC[1] = (1 << 0);   //Mode Select:0=Continuous-Measurement Mode, 1=Single-Measurement Mode, 2-3=Idle Mode.

    return HAL_I2C_Master_Transmit_DMA(&hi2c3, HMC_I2C_ADR, chDaneMagHMC, 2);		//rozpocznij pomiar na I2C1
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje odczyt danych z magnetometru HMC5883
// Parametry: brak
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 760us przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t StartujOdczytMagHMC(void)
{
	chDaneMagHMC[0] = DATA_XH;
	return HAL_I2C_Master_Transmit_DMA(&hi2c3, HMC_I2C_ADR, chDaneMagHMC, 1);	//wyślij polecenie odczytu wszystkich pomiarów
}



// Parametry: *dane_wy wskaźnik na dane wychodzące
// Zwraca: kod błędu HAL
// Czas zajęcia magistrali I2C: 2,2ms przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t CzytajMagnetometrHMC(void)
{
    return HAL_I2C_Master_Receive_DMA(&hi2c3, HMC_I2C_ADR, chDaneMagHMC, 6);		//odczytaj dane
}



////////////////////////////////////////////////////////////////////////////////
// Inicjuje zmienne konfiguracyjne
// Parametry: 
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMagnetometrHMC(void)
{
	uint8_t chErr;

    //sprawdź obecność magnetometru na magistrali I2C
    chErr = SprawdzObecnoscHMC5883();
    if (chErr)
    	return chErr;

    chDaneMagHMC[0] = CONF_A;
    //Ustaw tryb pracy na 30Hz
    //Configuration Register A
    chDaneMagHMC[1] = 	(0 << 0)|   //Measurement Configuration: 0=Normal measurement configuration, 1=Positive bias configuration, 2=Negative bias configuration
    					(5 << 2)|   //Data Output Rate:6=75Hz, 5=30, 4=15, 3=7,5, 2=3
						(2 << 5)|   //Select number of samples averaged per measurement output: 00 = 1; 01 = 2; 10 = 4; 11 = 8
						(0 << 7);   //This bit must be cleared for correct operation.
    //Configuration Register B
    chDaneMagHMC[2] = 	(1 << 5);   //Gain Configuration: 0=0,88 Gaussa, 1=1,3; 2=1,9; , 3=2,5; 4=4; 5=4,7; 6=5,6; 7=8,1 Gaussa
    //Mode Register
    chDaneMagHMC[3] = 	(0 << 0)|   //Mode Select:0=Continuous-Measurement Mode, 1=Single-Measurement Mode, 2-3=Idle Mode.
              	  	  	(0 << 2);   //Bits 2-7 must be cleared for correct operation.

    return HAL_I2C_Master_Transmit(&hi2c3, HMC_I2C_ADR, chDaneMagHMC, 4, MAG_TIMEOUT);		//zapisz 3 rejestry jedną transmisją
}


////////////////////////////////////////////////////////////////////////////////
// Testuje obecność magnetometru HMC5883 na magistrali korzystając z Identification Register A,B,C
// Komunikacja w trybie blokującym
// Parametry: brak
// Zwraca: systemowy kod błędu
// Czas zajęcia magistrali I2C: ms przy zegarze 100kHz
////////////////////////////////////////////////////////////////////////////////
uint8_t SprawdzObecnoscHMC5883(void)
{
	uint8_t err;

	chDaneMagHMC[0] = ID_A;
    err = HAL_I2C_Master_Transmit(&hi2c3, HMC_I2C_ADR, chDaneMagHMC, 1, MAG_TIMEOUT);	//wyślij polecenie odczytu rejestrów identyfikacyjnych
    if (!err)
    {
        err =  HAL_I2C_Master_Receive(&hi2c3, HMC_I2C_ADR, chDaneMagHMC, 3, MAG_TIMEOUT);		//odczytaj dane
        if (!err)
        {
        	if ((chDaneMagHMC[0] == 'H') && (chDaneMagHMC[1] == '4') && (chDaneMagHMC[2] == '3'))
        	{
        		uDaneCM4.dane.nZainicjowano |= INIT_HMC5883;
        		return ERR_OK;
        	}
        }
        uDaneCM4.dane.nZainicjowano &= ~INIT_HMC5883;
    }
    return ERR_BRAK_MAG_ZEW;
}


