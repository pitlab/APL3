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
static uint8_t chTransferDMA2DZakonczony;

////////////////////////////////////////////////////////////////////////////////
// Inicjuje zasoby używane przez OSD
// Parametry:
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOSD(void)
{
	uint8_t chErr = BLAD_OK;

	hdma2d.XferCpltCallback = XferCpltCallback;
	hdma2d.XferErrorCallback = XferErrorCallback;

	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M_BLEND;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB888;
	hdma2d.Init.OutputOffset = 0;
	//1=foreground: OSD
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB4444;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
	hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_SWAP;
	hdma2d.LayerCfg[1].ChromaSubSampling = DMA2D_NO_CSS;
	//background: kamera
	hdma2d.LayerCfg[0].InputOffset = 0;
	hdma2d.LayerCfg[0].InputColorMode = DMA2D_INPUT_RGB565;
	hdma2d.LayerCfg[0].AlphaMode = DMA2D_REPLACE_ALPHA;
	hdma2d.LayerCfg[0].InputAlpha = 0xFF;		//nieprzezroczysty
	chErr |= HAL_DMA2D_Init(&hdma2d);
	chErr |= HAL_DMA2D_ConfigLayer(&hdma2d, 0);
	chErr |= HAL_DMA2D_ConfigLayer(&hdma2d, 1);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja rysuje w buforze pamieci elementy ekranu OSD
// Parametry:
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void RysujOSD()
{
	uint16_t sKolor, sTlo, sBrakTla;

	sKolor = 0x0000;
	WypelnijEkranwBuforze(chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	sBrakTla = 0;
	sprintf(chNapisOSD, "Abc");

	//zwiększaj przezroczystość w połnej skali 16 kroków
	for (uint8_t n=0; n<16; n++)
	{
		sKolor = 0x0F00 | n << 12;	//czerwony
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 0, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x00F0 | n << 12;	//zielony
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 20, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x000F | n << 12;	//niebieski
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 40, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x00FF | n << 12;	//cyjan
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 60, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x0F0F | n << 12;	//magenta
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 80, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
		sKolor = 0x0FF0 | n << 12;	//żółty
		RysujProstokatWypelnionywBuforze(n * DISP_X_SIZE/16, 100, DISP_X_SIZE/16, 20, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

		sKolor = 0x7F0F;	//magenta
		sTlo = 0x0999 | n << 12;	//szary
		RysujNapiswBuforze(chNapisOSD, n * DISP_X_SIZE/16, 120, chBuforOSD, (uint8_t*)&sKolor, (uint8_t*)&sTlo, ROZMIAR_KOLORU_OSD);
		sKolor = 0x0F0F | n << 12;	//magenta
		RysujNapiswBuforze(chNapisOSD, n * DISP_X_SIZE/16, 140, chBuforOSD, (uint8_t*)&sKolor, (uint8_t*)&sBrakTla, ROZMIAR_KOLORU_OSD);
	}

	//podwójna linia ukośna
	sKolor = 0x70F0;	//zielony
	RysujLiniewBuforze(20, 100, 400, 300, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	sKolor = 0x7F0F;	//przeciwstawny do zielonego
	RysujLiniewBuforze(21, 101, 401, 301, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	sKolor = 0x7F00;	//czerwony
	RysujLiniePoziomawBuforze(20, 400, 160, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	sKolor = 0x700F;	//niebieski
	RysujLiniePionowawBuforze(20, 160, 300, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);

	//podwójny okrąg o przeciwstawnych kolorach
	sKolor = 0x7707;	//fioletowy
	RysujOkragwBuforze(240, 180, 80, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
	sKolor = 0x77F7;	//
	RysujOkragwBuforze(240, 180, 81, chBuforOSD, (uint8_t*)&sKolor, ROZMIAR_KOLORU_OSD);
}



////////////////////////////////////////////////////////////////////////////////
// Funkcja wykonuje zmieszanie zawartosci bufora OSD z obrazem z kamery
// Parametry:
//	*chObrazFront - adres bufora foreground
//	*chObrazTlo - adre bufora backbground
//	*chBuforWyjsciowy - adres bufora wyjściowego
//	sSzerokosc, sWysokosc - rozmiary obrazu
// Zwraca: kod błędu
// Czas operacji 16,5ms dla obrazu 480x320
////////////////////////////////////////////////////////////////////////////////
uint8_t PolaczBuforOSDzObrazem(uint8_t *chObrazFront, uint8_t *chObrazTlo, uint8_t *chBuforWyjsciowy, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr;
	uint8_t chTimeout = 100;
	uint32_t nErr;

	if (hdma2d.State == HAL_DMA2D_STATE_BUSY)
		HAL_DMA2D_Abort(&hdma2d);
	chTransferDMA2DZakonczony = 0;
	chErr = HAL_DMA2D_BlendingStart_IT(&hdma2d, (uint32_t)chObrazFront, (uint32_t)chObrazTlo, (uint32_t)chBuforWyjsciowy, (uint32_t)sSzerokosc, (uint32_t)sWysokosc);
	do
	{
		osDelay(1);
		chTimeout--;
	}
	while (!chTransferDMA2DZakonczony && chTimeout);
	if (chTimeout == 0)
	{
		//chErr = BLAD_TIMEOUT;
		nErr = HAL_DMA2D_GetError(&hdma2d);
		chErr = nErr & 0xFF;
	}

	return chErr;
}




//callback for transfer complete.
void XferCpltCallback(DMA2D_HandleTypeDef *hdma2d)
{
	chTransferDMA2DZakonczony = 1;
}


//callback for transfer error.
void XferErrorCallback(DMA2D_HandleTypeDef *hdma2d)
{
	chTransferDMA2DZakonczony = 2;
}


//callback for line event.
void LineEventCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{
	chTransferDMA2DZakonczony = 0;
}

//callback for CLUT loading completion.
void CLUTLoadingCpltCallback(struct __DMA2D_HandleTypeDef *hdma2d)
{

}
