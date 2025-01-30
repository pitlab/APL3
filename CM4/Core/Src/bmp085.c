//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v2.0
// Obs�uga czujnika ci�nienia BMP085 formu Bosch
//
// (c) Pit Lab
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "bmp085.h"
#include "petla_glowna.h"

// Czujnik ma adres 0xEE
// Dopuszczalna pr�dko�� magistrali 3,4MHz
static unsigned char chRdWrBuf[3];
unsigned char chOSS;    //oversampling
static short sAC1, sAC2, sAC3, sB1, sB2, sMB, sMC, sMD; //parametry konfiguracyjne w EEPROM
static unsigned short sAC4, sAC5, sAC6;
static long lB5;


extern unsigned char chI2C0End;     //flagi ko�ca operacji na magistrali


////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna konwersj� temperatury
// Po starcie konwersji musi odczeka� ok 4,5ms na gotowo�� pomiaru
// Parametry: nic
// Zwraca: kod b��du
////////////////////////////////////////////////////////////////////////////////
unsigned char BMP085_StartTempConversion(void)
{
    chRdWrBuf[0] = 0x2E;
    return StartI2C0(0xEE, 0xF4, chRdWrBuf, 1, 0);
}


////////////////////////////////////////////////////////////////////////////////
// Odczytuje warto�� temperatury
// Parametry: nic
// Zwraca: temepratura
// Czas wykonania: 264us przy I2C pracujacej na 200kHz
////////////////////////////////////////////////////////////////////////////////
float BMP085_ReadTemperature(void)
{
    unsigned short sTemp;
    long lX1, lX2;
    float fTemp;

    sTemp = BMP085_Read16bit(0xF6);
    lX1 = ((sTemp - sAC6) * sAC5)/32768;
    lX2 = sMC * 2048 / (lX1 + sMD);
    lB5 = lX1 + lX2;
    fTemp = (lB5 + 8) / 160;
    return fTemp;
}


////////////////////////////////////////////////////////////////////////////////
// Rozpoczyna konwersj� ci�nienia
// Po starcie konwersji musi odczeka� na gotowo�� pomiaru. Czas zale�y od ilo�ci pr�bek
// dla Oversampling=0 - 4,5ms, 1 - 7,5ms, 2 - 13,5ms, 3 - 25,5ms
// Parametry: 
// [i] chOversampling - liczba 0..3 definiuj�ca ilo�� pomiar�w wykonywanych w czujniku
// Zwraca: kod b��du
////////////////////////////////////////////////////////////////////////////////
unsigned char BMP085_StartPresConversion(unsigned char chOversampling)
{
    chOSS = chOversampling;
    chRdWrBuf[0] = 0x34+(chOversampling<<6);
    return StartI2C0(0xEE, 0xF4, chRdWrBuf, 1, 0);
}


////////////////////////////////////////////////////////////////////////////////
// Odczytuje warto�� ci�nienia
// Parametry: nic
// Zwraca: cisnienie w kPa
// Czas wykonania: 318us przy I2C pracujacej na 200kHz
////////////////////////////////////////////////////////////////////////////////
float BMP085_ReadPressure(void)
{
    long lPres, lB3, lB6, lX1, lX2, lX3, lP;
    unsigned long lB4, lB7;

	uint32_t nCzasStart;

    nCzasStart = PobierzCzas();		//inicjuj pomiar czasu


    StartI2C0(0xEE, 0xF6, chRdWrBuf, 0, 3);
    while (!chI2C0End)      //czekaj na zako�czenie operacji na magistrali
    {
    	if (MinalCzas(nCzasStart) > 2400)   //czekaj maksymalnie 1200us
            return ERR_TIMEOUT;
    }

    lPres = chRdWrBuf[0];
    lPres = lPres << 8;
    lPres += chRdWrBuf[1];
    lPres = lPres << 8;
    lPres += chRdWrBuf[2];
    lPres = lPres >> (8 - chOSS);

    lB6 = lB5 - 4000;
    lX1 = (sB2 * (lB6 * lB6 >> 12)) >> 11;
    lX2 = sAC2 * lB6 >> 11;
    lX3 = lX1 + lX2;
    lB3 = (sAC1 * 4 + lX3)<<chOSS;
    lB3 = (lB3 + 2) >> 2;
    lX1 = sAC3 * lB6 >> 13;
    lX2 = (sB1 * (lB6 * lB6 >> 12)) >> 16;
    lX3 = ((lX1 + lX2) + 2) >> 2;
    lB4 = sAC4 * (unsigned long)(lX3 + 32768) >> 15;
    lB7 = ((unsigned long)lPres - lB3) * (50000 >> chOSS);
    if (lB7 < 0x80000000)
        lP = lB7 * 2 / lB4;
    else
        lP = lB7 / lB4 * 2;
    lX1 = (lP >> 8) * (lP >> 8);
    lX1 = (lX1 * 3038) >> 16;
    lX2 = (-7357 * lP) >> 16;
    lP += (lX1 + lX2 + 3791) >> 4;
    return (float)lP/1000;
}


////////////////////////////////////////////////////////////////////////////////
// Odczytuje 16 bitow� warto�� z czujnika
// Parametry: 
// chAddress - adres wewn�trznego rejestru lub kom�rki EEPROM
// Zwraca: odczytana warto��
// Czas wykonania: 
////////////////////////////////////////////////////////////////////////////////
unsigned short BMP085_Read16bit(unsigned char chAddress)
{
    unsigned short sResult;


    sResult = chRdWrBuf[0];
    sResult *= 0x100;
    sResult += chRdWrBuf[1];
    return sResult;
}



////////////////////////////////////////////////////////////////////////////////
// Wykonaj inicjalizacj� czujnika. Odczytaj wszystkie parametry konfiguracyjne z EEPROMu
// Parametry: nic
// Zwraca: kod b��du
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
unsigned char InitBMP085(void)
{
    unsigned short sCnt = 100;
    do
    {
    sAC1 = (short)BMP085_Read16bit(0xAA);
    sAC2 = (short)BMP085_Read16bit(0xAC);
    sAC3 = (short)BMP085_Read16bit(0xAE);
    sAC4 = BMP085_Read16bit(0xB0);
    sAC5 = BMP085_Read16bit(0xB2);
    sAC6 = BMP085_Read16bit(0xB4);
    sB1 = (short)BMP085_Read16bit(0xB6);
    sB2 = (short)BMP085_Read16bit(0xB8);
    sMB = (short)BMP085_Read16bit(0xBA);
    sMC = (short)BMP085_Read16bit(0xBC);
    sMD = (short)BMP085_Read16bit(0xBE);
    sCnt--;
    }
    while ((!sAC1 | !sAC2 | !sAC3 | !sAC4 | !sAC5 | !sAC6 | !sB1 | !sB2 | !sMB | !sMC | !sMD) & sCnt);
    
    if (sCnt)
        return ERR_OK;
    else
        return ERR_TIMEOUT;
}
