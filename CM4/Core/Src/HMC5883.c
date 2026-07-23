//////////////////////////////////////////////////////////////////////////////extern
//
// AutoPitLot v3.0
// Obsługa magnetometru HMC5883 na magistrali I2C
//
// (c) Pit Lab 2025-26
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "HMC5883.h"
#include "WymianaCM4.h"
#include "Fram.h"
#include "KonfigFram.h"

//definicje zmiennych

extern volatile unia_wymianyCM4_t uDaneCM4;
extern I2C_HandleTypeDef hi2c3;
uint8_t cDaneMagHMC[6];
uint8_t cPoleceniaHMC[2];
uint8_t cSekwencjaPomiaruHMC;		//w trakcie pracy wyznacza kolejność sekwencji pomiarowych, w trakcie inicjalizacj pełni rolę licznika prób inicjalizacji
extern uint8_t cCzujnikOdczytywanyNaI2CExt;	//identyfikator czujnika odczytywanego na zewntrznym I2C. Potrzebny do tego aby powiązać odczytane dane z rodzajem obróbki
float fPrzesMagn3[3], fSkaloMagn3[3];

// Obsługa magnetometru wymaga wykonania kilku czynności rozłożonych w czasie
// 1) Wystartowanie konwersji trwającej 6ms
// 2) Wystartowanie odczytu wykonanego pomiaru. Odczyt wykonuje się w tle w procedurze przerwania I2C i
//    trwa ok. 4ms
// 3) Przepisania odczytanych w przerwaniu danych do zmiennych pomiarowych

//Czas od rozpoczęcia pomiaru do gotowych danych 6ms



////////////////////////////////////////////////////////////////////////////////
// Inicjuje zmienne konfiguracyjne
// Parametry: 
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujMagnetometrHMC(void)
{
	uint8_t cBłąd;

    //sprawdź obecność magnetometru na magistrali I2C
	cBłąd = SprawdzObecnoscHMC5883();
    if (cBłąd)
    	return cBłąd;

    cDaneMagHMC[0] = CONF_A;
    //Ustaw tryb pracy na 30Hz
    //Configuration Register A
    cDaneMagHMC[1] = 	(0 << 0)|   //Measurement Configuration: 0=Normal measurement configuration, 1=Positive bias configuration, 2=Negative bias configuration
    					(5 << 2)|   //Data Output Rate:6=75Hz, 5=30, 4=15, 3=7,5, 2=3
						(2 << 5)|   //Select number of samples averaged per measurement output: 00 = 1; 01 = 2; 10 = 4; 11 = 8
						(0 << 7);   //This bit must be cleared for correct operation.
    //Configuration Register B
    cDaneMagHMC[2] = 	(1 << 5);   //Gain Configuration: 0=0,88 Gaussa, 1=1,3; 2=1,9; , 3=2,5; 4=4; 5=4,7; 6=5,6; 7=8,1 Gaussa
    //Mode Register
    cDaneMagHMC[3] = 	(0 << 0)|   //Mode Select:0=Continuous-Measurement Mode, 1=Single-Measurement Mode, 2-3=Idle Mode.
              	  	  	(0 << 2);   //Bits 2-7 must be cleared for correct operation.
    cBłąd = HAL_I2C_Master_Transmit(&hi2c3, HMC_I2C_ADR, cDaneMagHMC, 4, MAG_TIMEOUT);		//zapisz 3 rejestry jedną transmisją
    if (cBłąd)
    	return cBłąd;

    for (uint16_t n=0; n<3; n++)
    {
    	cBłąd |= CzytajFramFloatZWalidacja(FAH_MAGN3_SKLADNIK_X + 4*n, &fPrzesMagn3[n], VMIN_SKLADNIK_MAGN, VMAX_SKLADNIK_MAGN, VDOM_SKLADNIK_MAGN);
    	cBłąd |= CzytajFramFloatZWalidacja(FAH_MAGN3_MNOZNIK_X + 4*n, &fSkaloMagn3[n], VMIN_MNOZNIK_MAGN, VMAX_MNOZNIK_MAGN, VDOM_MNOZNIK_MAGN);
    }
    return cBłąd;
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
	uint8_t cBłąd;

	cDaneMagHMC[0] = ID_A;
	cBłąd = HAL_I2C_Master_Transmit(&hi2c3, HMC_I2C_ADR, cDaneMagHMC, 1, MAG_TIMEOUT);	//wyślij polecenie odczytu rejestrów identyfikacyjnych
    if (!cBłąd)
    {
    	cBłąd =  HAL_I2C_Master_Receive(&hi2c3, HMC_I2C_ADR, cDaneMagHMC, 3, MAG_TIMEOUT);		//odczytaj dane
        if (!cBłąd)
        {
        	if ((cDaneMagHMC[0] == 'H') && (cDaneMagHMC[1] == '4') && (cDaneMagHMC[2] == '3'))
        	{

        		return BLAD_OK;
        	}
        }
        uDaneCM4.dane.nZainicjowano &= ~INIT_HMC5883;
    }
    return BLAD_BRAK_CZUJNIKA;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj jeden element sekwencji potrzebnych do uzyskania pomiaru
// Parametry: nic
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t ObslugaHMC5883(void)
{
	uint8_t cBłąd = BLAD_OK;

	if (uDaneCM4.dane.nBrakCzujnika & INIT_HMC5883)
		return BLAD_OK;		//czujnik jest opcjonalny, więc nie sygnalizuj błędu

	if ((uDaneCM4.dane.nZainicjowano & INIT_HMC5883) != INIT_HMC5883)
	{
		//ogranicz liczbę prób inicjalizacji aby w przypadku braku magnetometru nie blokować innych zadań
		if (cSekwencjaPomiaruHMC < MAX_PROB_INICJALIZACJI)
		{
			cSekwencjaPomiaruHMC++;
			cBłąd = InicjujMagnetometrHMC();
			if (cBłąd == BLAD_OK)
			{
				uDaneCM4.dane.nZainicjowano |= INIT_HMC5883;
				cSekwencjaPomiaruHMC = 0;
			}
		}
		else
		{
			uDaneCM4.dane.nBrakCzujnika |= INIT_HMC5883;
			cBłąd = BLAD_BRAK_CZUJNIKA;
		}
		return cBłąd;
	}

	switch (cSekwencjaPomiaruHMC)
	{
	case 0:		//startuj pomiar
		cPoleceniaHMC[0] = MODE;
		cPoleceniaHMC[1] = (1 << 0);   //Mode Select:0=Continuous-Measurement Mode, 1=Single-Measurement Mode, 2-3=Idle Mode.
		cBłąd = HAL_I2C_Master_Transmit_DMA(&hi2c3, HMC_I2C_ADR, cPoleceniaHMC, 2);
	    break;

	case 1:	//startuj odczyt
		cPoleceniaHMC[0] = DATA_XH;
		cBłąd = HAL_I2C_Master_Seq_Transmit_DMA(&hi2c3, HMC_I2C_ADR, cPoleceniaHMC, 1, I2C_FIRST_FRAME);	//wyślij polecenie odczytu pomiarów nie kończąc transferu STOP-em
		break;

	case 2:
		cBłąd = HAL_I2C_Master_Seq_Receive_DMA(&hi2c3, HMC_I2C_ADR, cDaneMagHMC, 6, I2C_LAST_FRAME);		//odczytaj status i zakończ STOP
		cCzujnikOdczytywanyNaI2CExt = MAG_HMC;		//informacja o tym jak mają być interpretowane dane odebrane w HAL_I2C_MasterRxCpltCallback()
		break;

	default: break;
	}
	cSekwencjaPomiaruHMC++;
	if (cSekwencjaPomiaruHMC >= 3)
		cSekwencjaPomiaruHMC = 0;
	return cBłąd;
}

