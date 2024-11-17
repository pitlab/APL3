//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi sterownika ekranu dotykowego na układzie ADS7846
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "dotyk.h"
#include "moduly_SPI.h"

//deklaracje zmiennych
extern SPI_HandleTypeDef hspi5;
uint16_t sDotykAdc[4];

////////////////////////////////////////////////////////////////////////////////
// Czyta dane ze sterownika ekranu dotykowego
// Parametry: chChannel - kod kanału X, Y, Z1, Z2
// Zwraca: wartość rezystancji
////////////////////////////////////////////////////////////////////////////////
uint16_t CzytajKanalDotyku(uint8_t  chKanal)
{
	uint8_t chKonfig, chDane[2];
	uint16_t sDotyk;


	chKonfig = (3<<0)|	//PD1-PD0 Power-Down Mode Select Bits: 0=Power-Down Between Conversions, 1=Reference is off and ADC is on, 2=Reference is on and ADC is off, 3=Device is always powered.
			   (0<<2)|	//SER/DFR 1=Single-Ended 0=Differential Reference Select Bit
			   (0<<3)|	//MODE 12-Bit/8-Bit Conversion Select Bit. This bit controls the number of bits for the next conversion: 0=12-bits, 1=8-bits
			   (chKanal<<4)|	//A2-A0 Channel Select Bits
			   (1<<7);	//S Start Bit.

	UstawDekoderZewn(CS_TP);											//TP_CS=0
	HAL_SPI_Transmit(&hspi5, &chKonfig, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi5, chDane, 2, HAL_MAX_DELAY);
	UstawDekoderZewn(CS_NIC);							//TP_CS=1

	//złóż liczbę 12-bitową i wyrównaj do lewej
	sDotyk = chDane[0];
	sDotyk <<= 8;
	sDotyk |= chDane[1];
	sDotyk >>= 3;
	return sDotyk;
}



////////////////////////////////////////////////////////////////////////////////
// Czytaj komplet danych ze sterownika ekranu dotykowego
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void CzytajDotyk(void)
{
	uint32_t nOld_SPI_CFG1;

	nOld_SPI_CFG1 = hspi5.Instance->CFG1;	//zachowaj nastawy konfiguracji SPI

	//Ponieważ zegar SPI = 100MHz a układ może pracować z prędkością max 2,5MHz a jest na tej samej magistrali co TFT przy każdym odczytcie przestaw dzielnik zegara z 4 na 64
	hspi5.Instance->CFG1 &= ~SPI_BAUDRATEPRESCALER_256;	//maska preskalera
	hspi5.Instance->CFG1 |= SPI_BAUDRATEPRESCALER_64;	//Bits 30:28 MBR[2:0]: master baud rate: 011: SPI master clock/64

	//domyslnie używam innej orientacji, więc zaimeń osie X i Y
	sDotykAdc[0] = CzytajKanalDotyku(TPCHY);	//najlepiej się zmienia dla osi X
	sDotykAdc[1] = CzytajKanalDotyku(TPCHX);	//najlepiej się zmienia dla osi Y
	sDotykAdc[2] = CzytajKanalDotyku(TPCHZ1);
	sDotykAdc[3] = CzytajKanalDotyku(TPCHZ2);

	/*/oblicz siłę dotyku - coś jest skopane, wskazania są odwrotnie proporcjonalne do siły nacisku
	if (sDotykAdc[2])
		sDotykAdc[4] = (200 * sDotykAdc[0]/4096) * ((sDotykAdc[3]/sDotykAdc[2])-1);
	else
		sDotykAdc[4] = 0; */
	hspi5.Instance->CFG1 = nOld_SPI_CFG1;	//przywróc poprzednie nastawy
}
