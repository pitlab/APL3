//////////////////////////////////////////////////////////////////////////////
//
// Obsługa wyświetlania na obrazie cyfrowym
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "osd.h"
#include "display.h"
#include "LCD_mem.h"
#include "cmsis_os.h"

//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) chBuforOSD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu OSD w formacie RGB888
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforOSD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu OSD w formacie RGB888
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu wyświetlacza w formacie RGB888
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[SZER_ZDJECIA * WYS_ZDJECIA / 2];
extern DMA2D_HandleTypeDef hdma2d;
static char chNapisOSD[60];
static uint8_t chTransferZakonczony;

////////////////////////////////////////////////////////////////////////////////
// Inicjuje zasoby używane przez OSD
// Parametry:
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOSD(void)
{
	uint8_t chErr = BLAD_OK;

	//chErr = HAL_DMA2D_Init(&hdma2d);
	return chErr;
}

////////////////////////////////////////////////////////////////////////////////
// Funkcja rysuje w buforze pamieci elementy ekranu OSD
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujOSD()
{
	WypelnijBuforEkranu(chBuforOSD, 0x222222);


	RysujProstokatWypelnionywBuforze(0, 0, DISP_X_SIZE, 20, chBuforOSD, 0x00FFFF);	//OK
	RysujLiniewBuforze(20, 100, 400, 300, chBuforOSD, 0x00FF00);
	RysujLiniePoziomawBuforze(20, 400, 100, chBuforOSD, 0xFF0000);
	RysujLiniePionowawBuforze(20, 100, 300, chBuforOSD, 0x0000FF);
	RysujOkragwBuforze(240, 160, 140, chBuforOSD, 0x7F007F);
	RysujZnakwBuforze('A', 20, 20, chBuforOSD, 0xAFAFAF, 0x555555, 0);
	RysujZnakwBuforze('B', 20, 40, chBuforOSD, 0xAFAFAF, 0x555555, 1);

	sprintf(chNapisOSD, "t: %d us ", 123);
	RysujNapiswBuforze(chNapisOSD, RIGHT, 60, chBuforOSD, 0xFFFF00, 0x555555, 1);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja wykonuje zmieszanie zawartosci bufora OSD z obrazem z kamery
// Parametry:
//	*chObrazKamery - adres bufora foreground
//	*chBuforOSD - adre buforw backbground
//	*chBuforWyjsciowy - adres bufora wyjściowego
//	sSzerokosc, sWysokosc - rozmiary obrazu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazKamery, uint8_t *chBuforOSD, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr;
	uint8_t chTimeout = 100;

	if (hdma2d.State == HAL_DMA2D_STATE_BUSY)
		HAL_DMA2D_Abort(&hdma2d);
	chTransferZakonczony = 0;
	chErr = HAL_DMA2D_BlendingStart(&hdma2d, (uint32_t)chObrazKamery, (uint32_t)chBuforOSD, (uint32_t)chBuforWyjsciowy, (uint32_t)sSzerokosc, (uint32_t)sWysokosc);
	do
	{
		osDelay(10);
		chTimeout--;
	}
	while (!chTransferZakonczony && chTimeout);
	if (chTimeout == 0)
		chErr = BLAD_TIMEOUT;

	return chErr;
}




//callback for transfer complete.
void XferCpltCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{
	chTransferZakonczony = 1;
}


//callback for transfer error.
void XferErrorCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{
	chTransferZakonczony = 2;
}


//callback for line event.
void LineEventCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{
	chTransferZakonczony = 0;
}

//callback for CLUT loading completion.
void CLUTLoadingCpltCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{

}
