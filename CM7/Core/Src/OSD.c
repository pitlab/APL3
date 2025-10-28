//////////////////////////////////////////////////////////////////////////////
//
// Obsługa wyświetlania na obrazie cyfrowym
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "OSD.h"
#include "display.h"
#include "LCD_mem.h"

uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforOSD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu OSD w formacie RGB888
extern uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];	//pamięć obrazu wyświetlacza w formacie RGB888
extern uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[SZER_ZDJECIA * WYS_ZDJECIA / 2];
extern DMA2D_HandleTypeDef hdma2d;
static char chNapisOSD[60];

////////////////////////////////////////////////////////////////////////////////
// Inicjuje zasoby używane przez OSD
// Parametry:
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOSD(void)
{
	uint8_t chErr;
	uint32_t DstAddress;

	chErr = HAL_DMA2D_Init(&hdma2d);

	chErr = DMA2D_SetConfig(&hdma2d, (uint32_t*)chBuforLCD, DstAddress, DISP_X_SIZE, DISP_Y_SIZE);

	return chErr;
}

////////////////////////////////////////////////////////////////////////////////
// Funkcja rysuje w buforze pamieci elementy ekranu OSD
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujOSD()
{
	WypelnijBuforEkranu(chBuforLCD, 0x222222);


	RysujProstokatWypelnionywBuforze(0, 0, DISP_X_SIZE, 20, chBuforOSD, 0x00FFFF);	//OK
	RysujLiniewBuforze(20, 100, 400, 300, chBuforOSD, 0x00FF00);
	RysujLiniePoziomawBuforze(20, 400, 100, chBuforOSD, 0xFF0000);
	RysujLiniePionowawBuforze(20, 100, 300, chBuforOSD, 0x0000FF);
	RysujOkragwBuforze(240, 160, 140, chBuforOSD, 0x7F007F);
	RysujZnakwBuforze('A', 20, 20, chBuforOSD, 0xAFAFAF, 0x555555, 0);
	RysujZnakwBuforze('B', 20, 40, chBuforOSD, 0xAFAFAF, 0x555555, 1);

	sprintf(chNapisOSD, "t: %ld us ", 123);
	RysujNapiswBuforze(chNapisOSD, RIGHT, 60, chBuforLCD, 0xFFFF00, 0x555555, 1);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja wykonuje zmieszanie zawartosci bufora OSD z obrazem z kamery
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void PolaczBuforOSDzObrazem(uint8_t *chBuforOSD, uint8_t *chObrazKamery, uint8_t *chBuforWyjsciowy)
{
	//HAL_DMA2D_Start_IT(&hdma2d);
	//HAL_DMA2D_BlendingStart_IT();
}




//callback for transfer complete.
void XferCpltCallback()
{

}


//callback for transfer error.
void XferErrorCallback()
{

}


//callback for line event.
void LineEventCallback()
{

}

//callback for CLUT loading completion.
void CLUTLoadingCpltCallback()
{

}
