//////////////////////////////////////////////////////////////////////////////
//
// Obsługa sprzętowej kompresji obrazu
// Opracowane na podstawie AN4996 v4
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "jpeg.h"
#include "cmsis_os.h"
#include "ff.h"
#include "czas.h"
#include "rejestrator.h"
#include "analiza_obrazu.h"
#include "kamera.h"
#include "napisy.h"
#include "jpeg_utils_conf.h"
//#include "jpeg_utils.h"

extern JPEG_HandleTypeDef hjpeg;
extern MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
extern MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
extern const char *chNapisLcd[MAX_NAPISOW];
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM2"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG] = {0};	//są problemy z zapisem na kartę
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG] = {0};
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".Bufory_SRAM3"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG] = {0};
//uint8_t chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU];  //Bufor danych wejściowych do kompresji
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM2"))) chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU] = {0}; - wywala się ethernet, być może dane wchodzą na stos
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".Bufory_SRAM3"))) chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU] = {0};	- błąd DMA
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM")))chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU] = {0};

volatile uint8_t chStatusBufJpeg;	//przechowyje bity okreslające status procesu przepływu danych na kartę SD
volatile uint8_t chWskNapBufJpeg, chWskOprBufJpeg;	// wskazuje do którego bufora obecnie są zapisywane dane i z którego są odczytywane
uint32_t nRozmiarObrazuJPEG;	//w bajtach
uint8_t chWynikKompresji;		//flagi ustawiane w callbackach, określajace postęp przepływu danych przez enkoder JPEG
uint8_t chWskNapBufMcu;			//wskaźnik napełniania buforów MCU
volatile uint16_t sZajetoscBuforaWeJpeg, sZajetoscBuforaWyJpeg;		//liczba bajtów w buforach wejściowym i wyjściowym kompresora
extern uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
uint8_t chNaglJpegExif[ROZMIAR_EXIF];
const uint8_t chNaglJpegEOI[ROZMIAR_ZNACZ_xOI] = {0xFF, 0xD9};	//EOI (End Of Image) - zapisywany na końcu pliku jpeg

static uint16_t JPEG_Y_MCU_LUT[256];
static uint16_t JPEG_Y_MCU_444_LUT[64];

static uint16_t JPEG_Cb_MCU_420_LUT[256];
static uint16_t JPEG_Cb_MCU_422_LUT[256];
static uint16_t JPEG_Cb_MCU_444_LUT[64];

static uint16_t JPEG_Cr_MCU_420_LUT[256];
static uint16_t JPEG_Cr_MCU_422_LUT[256];
static uint16_t JPEG_Cr_MCU_444_LUT[64];

static uint16_t JPEG_K_MCU_420_LUT[256];
static uint16_t JPEG_K_MCU_422_LUT[256];
static uint16_t JPEG_K_MCU_444_LUT[64];

static int32_t RED_Y_LUT[256];            /* Red to Y color conversion Look Up Table  */
static int32_t RED_CB_LUT[256];           /* Red to Cb color conversion Look Up Table  */
static int32_t BLUE_CB_RED_CR_LUT[256];   /* Red to Cr and Blue to Cb color conversion Look Up Table  */
static int32_t GREEN_Y_LUT[256];          /* Green to Y color conversion Look Up Table*/
static int32_t GREEN_CR_LUT[256];         /* Green to Cr color conversion Look Up Table*/
static int32_t GREEN_CB_LUT[256];         /* Green to Cb color conversion Look Up Table*/
static int32_t BLUE_Y_LUT[256];           /* Blue to Y color conversion Look Up Table */
static int32_t BLUE_CR_LUT[256];          /* Blue to Cr color conversion Look Up Table */
//JPEG_RGBToYCbCr_Convert_Function pRGBToYCbCr_Convert_Function;
static  JPEG_MCU_RGB_ConvertorTypeDef JPEG_ConvertorParams;

// Nagłówek JPEG dla obrazu 480x320, 8-bit Y8 grayscale
/*static const uint8_t chNaglJpeg_480x320_y8[] = {
	0xFF, 0xD8,                                     // Start Of Image
	0xFF, 0xE0, 0x00, 0x10, 'J', 'F', 'I', 'F', 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00,0x00,	// APP0 (JFIF)

	0xFF, 0xDB, 	// DQT Tablica kwantyzacji luminancji
	0x00, 0x43,		// długość tablicy (67)
	0x00,			//0 = luminancja
	16,11,10,16,24,40,51,61, 12,12,14,19,26,58,60,55,	//tablica 8x8
	14,13,16,24,40,57,69,56, 14,17,22,29,51,87,80,62,
	18,22,37,56,68,109,103,77, 24,35,55,64,81,104,113,92,
	49,64,78,87,103,121,120,101, 72,92,95,98,112,100,103,99,

	0xFF, 0xC0, 	// SOF (Start Of Frame 0) (Baseline DCT)
	0x00, 0x0B,		// rozmiar: 12
	0x08,           // precyzja: 8 bitów na piksel
	0x01, 0x40,     // wysokość = 320 (0x0140)
	0x01, 0xE0,     // szerokość = 480 (0x01E0)
	0x01,          	// liczba komponentów = 1 (Y)
	0x01, 			// ID=1
	0x11, 			// sampling 1x1
	0x00,         	// DQT=0

	0xFF, 0xC4, 	// DFT (Define Huffman Table) (DC Luminancja)
	0x00, 0x1F, 	// długość: 31
	0x00,			// klasa: 0x=DC x0=Destination
	0x00,0x01,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,

	0xFF, 0xC4, 	// DFT (Define Huffman Table)  (AC Luminancja)
	0x00, 0xB5, 	// długość: 181
	0x10,			// klasa: 1x=AC x0=Destination
	0x00,0x02,0x01,0x03,0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04,0x00,0x00,0x01,0x7D,
	0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
	0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,
	0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,
	0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
	0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
	0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
	0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,
	0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,
	0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,
	0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
	0xF9,0xFA,

	0xFF, 0xDA, 	// SOS (Start of Scan)
	0x00, 0x08,		// rozmiar: 8
	0x01, 			// liczba komponentów: 1 - Luminancja
	0x01, 0x00,		// użyte tablice: 1={DC=0, AC=0}
	0x00, 0x3F, 	// spectral selection: 0..63
	0x00			// succesive approximation: 00
};

static const uint8_t chNaglJpeg_480x320_yuv420[] = {
	0xFF, 0xD8,		// SOI
	0xFF, 0xE0, 0x00, 0x10, 'J','F','I','F',0x00, 0x01,0x01,0x00,0x00,0x01,0x00,0x01,0x00,0x00,		// APP0 JFIF

	// DQT luminancja (ID=0)
	0xFF, 0xDB, 0x00, 0x43, 0x00,
	16,11,10,16,24,40,51,61, 12,12,14,19,26,58,60,55,
	14,13,16,24,40,57,69,56, 14,17,22,29,51,87,80,62,
	18,22,37,56,68,109,103,77, 24,35,55,64,81,104,113,92,
	49,64,78,87,103,121,120,101, 72,92,95,98,112,100,103,99,

	// DQT chrominancja (ID=1)
	0xFF, 0xDB, 0x00, 0x43, 0x01,
	17,18,24,47,99,99,99,99, 18,21,26,66,99,99,99,99,
	24,26,56,99,99,99,99,99, 47,66,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99,
	99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99,

	// SOF0 (Baseline DCT)
	0xFF, 0xC0, 0x00, 0x11,
	0x08,             // 8-bit per sample
	0x01, 0x40,       // height = 320
	0x01, 0xE0,       // width  = 480
	0x03,             // 3 components (Y, Cb, Cr)
	0x01, 0x22, 0x00, // Component Y: ID=1, sampling 2x2 (H=2, V=2), uses QTable 0
	0x02, 0x11, 0x01, // Component Cb: ID=2, sampling 1x1, uses QTable 1
	0x03, 0x11, 0x01, // Component Cr: ID=3, sampling 1x1, uses QTable 1

	// DHT DC Luminance
	0xFF,0xC4,0x00,0x1F,0x00,
	0x00,0x01,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,

	// DHT AC Luminance
	0xFF,0xC4,0x00,0xB5,0x10,
	0x00,0x02,0x01,0x03,0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04,0x00,0x00,0x01,0x7D,
	0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,
	0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,
	0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,
	0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
	0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,
	0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
	0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,
	0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,
	0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,
	0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
	0xF9,0xFA,

	// DHT DC Chrominance
	0xFF,0xC4,0x00,0x1F,0x01,
	0x00,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,
	0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,

	// DHT AC Chrominance
	0xFF,0xC4,0x00,0xB5,0x11,
	0x00,0x02,0x01,0x02,0x04,0x04,0x03,0x04,0x07,0x05,0x04,0x04,0x00,0x01,0x02,0x77,
	0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,
	0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,0xF0,
	0x15,0x62,0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,0x26,
	0x27,0x28,0x29,0x2A,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,
	0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,
	0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x82,0x83,0x84,0x85,0x86,0x87,
	0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,
	0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,
	0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,
	0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,
	0xF9,0xFA,

	// Start of Scan (SOS)
	0xFF, 0xDA, 0x00, 0x0C,
	0x03,              // number of components
	0x01, 0x00,        // Y  uses DC=0, AC=0
	0x02, 0x11,        // Cb uses DC=1, AC=1
	0x03, 0x11,        // Cr uses DC=1, AC=1
	0x00, 0x3F, 0x00   // spectral selection
};*/



////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja kompresji sprzętowej
// Parametry: brak
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
  uint8_t InicjalizujJpeg(void)
{
	uint8_t chErr;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	//ustaw piny PB0 i PB1 do machania. W CM4 są skonfigurowane jako GPIO_MODE_OUTPUT_PP
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;	//Test
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	chErr = HAL_JPEG_Init(&hjpeg);
	HAL_JPEG_RegisterCallback(&hjpeg, HAL_JPEG_ERROR_CB_ID, HAL_JPEG_ErrorCallback);
   	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja kompresji sprzętowej
// Parametry: sSzerokosc, sWysokosc - rozmiary kompresowanego obrazu
// chTypKoloru - stała definiująca przestrzeń kolorów dla sprzętowego enkodera
// chJakoscObrazu - współczynnik z zakresu 0..100 okreslający jakość kompresji
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu)
{
	JPEG_ConfTypeDef pConf;

	if (hjpeg.State == HAL_JPEG_STATE_ERROR)
		HAL_JPEG_Abort(&hjpeg);
	pConf.ColorSpace = chTypKoloru;					//JPEG_GRAYSCALE_COLORSPACE, JPEG_YCBCR_COLORSPACE
	pConf.ChromaSubsampling = chTypChrominancji;	//JPEG_444_SUBSAMPLING, JPEG_422_SUBSAMPLING, JPEG_420_SUBSAMPLING
	pConf.ImageWidth = sSzerokosc;
	pConf.ImageHeight = sWysokosc;
	pConf.ImageQuality = chJakoscObrazu;
	return HAL_JPEG_ConfigEncoding(&hjpeg, &pConf);
}



////////////////////////////////////////////////////////////////////////////////
// Callback HAL_JPEG_EncodeCpltCallback is asserted when the HAL JPEG driver has ended the current JPEG encoding operation,
// and all output data has been transmitted to the application.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
	chStatusBufJpeg |= STAT_JPG_GOTOWY;		//podany obraz właśnie został skompresowany
}



////////////////////////////////////////////////////////////////////////////////
// Callback HAL_JPEG_ErrorCallback is asserted when an error occurred during the current operation.
// the application can call the function HAL_JPEG_GetError()  to retrieve the error codes.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
	printf("JPEG_Error: %ld\n\r", HAL_JPEG_GetError(hjpeg));
	HAL_JPEG_Abort(hjpeg);
	nRozmiarObrazuJPEG = 0;
}



////////////////////////////////////////////////////////////////////////////////
//Można podać enkoderowi kolejne dane
// Callback HAL_JPEG_GetDataCallback is asserted for both encoding and decoding
// operations to inform the application that the input buffer has been
// consumed by the peripheral and to ask for a new data chunk if the operation
// (encoding/decoding) has not been complete yet.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{
	sZajetoscBuforaWeJpeg -= NbDecodedData;
	if (sZajetoscBuforaWeJpeg)	//jeżeli po pobraniu danych w buforze wejsściowym są jeszcze dane, to kompresuj je
	{
		HAL_JPEG_ConfigInputBuffer(hjpeg, chBuforMCU + NbDecodedData, sZajetoscBuforaWeJpeg);
		printf("Kont ");
	}
	else	//jeżeli kompresor zużył wszystkie dane jakie były przygotowane
	{
		HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);
		chStatusBufJpeg |= STAT_JPG_ZATRZYMANE_WE;
		//printf("PauWe, ");
	}
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
}



////////////////////////////////////////////////////////////////////////////////
//skompresowane dane sa gotowe
// Callback HAL_JPEG_DataReadyCallback is asserted when the HAL JPEG driver has filled the given output buffer with the given size.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
	sZajetoscBuforaWyJpeg += OutDataLength;
	nRozmiarObrazuJPEG += OutDataLength;
	printf("Rozm: %ld wsk: %d buf: %d\r\n",  nRozmiarObrazuJPEG, chWskNapBufJpeg, sZajetoscBuforaWyJpeg);
	chWskNapBufJpeg++;							//przełącz bufor danych skompresowanych na następny
	chWskNapBufJpeg &= MASKA_LICZBY_BUF;

	if (sZajetoscBuforaWyJpeg < ILOSC_BUF_JPEG * ROZM_BUF_WY_JPEG)
	{
		HAL_JPEG_ConfigOutputBuffer(hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);	//ustaw następny bufor
	}
	else
	{
		HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);
		chStatusBufJpeg |= STAT_JPG_ZATRZYMANE_WY;
		printf("PauWy, ");
		if (!(chStatusRejestratora & STATREJ_ZAPISZ_JPG))
			chStatusRejestratora |= STATREJ_ZAPISZ_JPG;	//wyłącz flagę obsługi pliku JPEG
	}
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
}



////////////////////////////////////////////////////////////////////////////////
// Czeka z timeoutem aż JPEG skompresuje wszystkie MCU
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzekajNaKoniecPracyJPEG(void)
{
	uint8_t chErr = BLAD_OK;
	uint8_t chLicznikTimeoutu = 200;

	while (((chStatusBufJpeg & STAT_JPG_GOTOWY) != STAT_JPG_GOTOWY) && chLicznikTimeoutu)
	{
		osDelay(5);		//przełącz na inny wątek
		chLicznikTimeoutu--;
	}

	if (chLicznikTimeoutu == 0)
	{
		HAL_JPEG_Abort(&hjpeg);
		chErr = BLAD_TIMEOUT;
	}
	else
		printf("Czekano %dms, ", (200 - chLicznikTimeoutu) * 5);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Przygotuj pojedynczy MCU Y8 czyli blok 8x8 dla sprzętowego kompresora pracującego w formacie JPEG_GRAYSCALE_COLORSPACE
// Format Y8 zawiera same bloki luminancji, przez co można na nim uzyskać najwyższy wspóczynnik kompresji
// Skompresowane dane są pakowane do bufora aby wątek rejestratora mógł zapisać je do pliku
// Pomiary pokazują że kolejne 6 MCU 8x8 pobierane są co 136us. Pełen obraz 1280x960 zawiera 160x120 = 19200 MCU stąd kompresja obrazu 19200/6*136us zajmuje 436,2ms
// Aby uzyskać strumień przynajmniej 15fps czas kompresji musi być mniejszy niż 66,6(6)ms co odpowiada 2941 MCU czyli np. 77x38 MCU = 616x304
// Parametry:
// [we] *chObrazWe - wskaźnik na czarno-biały obraz wejsciowy Y8
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujY8(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nOffsetMCU_wzgObrazu;	//przesuniecie bloku wzgledem początku obrazu
	uint32_t nOffsetMCU_wzgWiersza;
	uint32_t nOffset_wzgMCU;		//przesunęcie obrazu wejściowego względem początku MCU
	uint8_t chLiczbaMCU_Szer = sSzerokosc / SZEROKOSC_BLOKU;	//liczba MCU luminancji po szerokosci obrazu
	uint8_t chLiczbaMCU_Wys = sWysokosc / WYSOKOSC_BLOKU;	//liczba MCU luminancji po wysokosci obrazu

	for (uint8_t m=0; m<ILOSC_BUF_JPEG; m++)
	{
		for (uint16_t n=0; n<ROZM_BUF_WY_JPEG; n++)
			chBuforJpeg[m][n] = 0;
	}
	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_GRAYSCALE_COLORSPACE, JPEG_444_SUBSAMPLING, 80);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);


	for (uint16_t my=0; my<chLiczbaMCU_Wys; my++)	//pętla pracująca na MCU wysokości obrazu
	{
		nOffsetMCU_wzgObrazu = my * chLiczbaMCU_Szer * ROZMIAR_BLOKU;
		for (uint16_t mx=0; mx<chLiczbaMCU_Szer; mx++)	//pętla pracująca na MCU szerokości obrazu
		{
			//rozpocznij formowanie MCU

			//sprawdź czy jest miejsce na kolejny MCU
			while (sZajetoscBuforaWeJpeg > ((ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU) - ROZMIAR_MCU_Y8))
			{
				printf("PelenBuf, ");
				chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję
				osDelay(5);
			}

			nOffsetMCU_wzgWiersza = mx * SZEROKOSC_BLOKU;					//przesunięcie o liczbę MCU w bieżącym wierszu
			for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)	//pętla pracująca na wysokosci wewnątrz MCU
			{
				nOffset_wzgMCU = (y * chLiczbaMCU_Szer * SZEROKOSC_BLOKU);	//przesunięcie pełnych wierszy obrazu
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)	//pętla pracująca na szerokości wewnątrz MCU
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + chWskNapBufMcu * ROZMIAR_BLOKU] = *(chObrazWe + x + nOffset_wzgMCU + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);
			}
			//MCU gotowe
			sZajetoscBuforaWeJpeg += ROZMIAR_BLOKU;
			//printf("in%d:%d, ", sZajetoscBuforaWeJpeg, chWskNapBufMcu);
			chWskNapBufMcu++;

			//jeżeli bufor wejściowy jest przepełniony, to przerwij pracę i posprzątaj po sobie
			if (sZajetoscBuforaWeJpeg > (ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU))
			{
				chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//wymuś zamknięcie otwartego pliku
				printf("Zatrzymano, ");
				return ERR_BUF_OVERRUN;
			}

			//używam podwójnego buforowania, więc dzielę bufor na 2 części. Jedna jest napełnianam druga opróżniana
			if ((chWskNapBufMcu == ILOSC_BUF_WE_MCU/2) || (chWskNapBufMcu == ILOSC_BUF_WE_MCU))		//czy mamy już połowę bufora lub cały bufor? Dla Y8 1 blok = 1 MCU
			{
				//printf("in%d:%d, ", sZajetoscBuforaWeJpeg, chWskNapBufMcu);

				//wskaż na źródło danych w pierwszej lub drugiej połowie bufora
				if (chWskNapBufMcu == ILOSC_BUF_WE_MCU/2)
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//pierwsza połowa bufora
				else
				{
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU + ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//druga połowa
					chWskNapBufMcu = 0;	//zacznij napełniać od początku
				}

				if (hjpeg.State == HAL_JPEG_STATE_READY)	//tylko dla pierwszego uruchomienia
				{
					chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU, chBuforJpeg[chWskNapBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
					printf("Start1, ");
				}
				else
				{
					if (chStatusBufJpeg & STAT_JPG_ZATRZYMANE_WE)
					{
						chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję
						chStatusBufJpeg &= ~STAT_JPG_ZATRZYMANE_WE;
						printf("Wzn1, ");
					}
					else
						osDelay(1);
				}
			}
		}
	}

	chErr = CzekajNaKoniecPracyJPEG();		//czekaj aż ostatnie MCU zostanie skompresowane
	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
	printf("Koniec.\r\n");
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Przygotuj pojedynczy MCU YUV444 czyli blok 8x8 z luminancją i 2 bloki z chrominancją z dla sprzętowego kompresora pracującego w formacie JPEG_444_SUBSAMPLING
// Format YUV444 zawiera pełną informację o chrominancji przez co zapewnia najlepsze odwzorowanie kolorów
// Skompresowane dane są pakowane do bufora aby wątek obsługi rejestratora mógł zapisać je do pliku
// Parametry:
// [we] *chObrazWe - wskaźnik na kolorowy obraz wejciowy YUV444
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujYUV444(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nOffsetMCU_wzgObrazu;	//przesuniecie bloku wzgledem początku obrazu
	uint32_t nOffsetMCU_wzgWiersza;
	uint32_t nOffset_wzgMCU;		//przesunęcie obrazu wejściowego względem początku MCU
	uint8_t chLiczbaMCU_Szer = sSzerokosc / SZEROKOSC_BLOKU;	//liczba MCU luminancji po szerokosci obrazu
	uint8_t chLiczbaMCU_Wys = sWysokosc / WYSOKOSC_BLOKU;	//liczba MCU luminancji po wysokosci obrazu

	for (uint8_t m=0; m<MASKA_LICZBY_BUF; m++)
	{
		for (uint16_t n=0; n<ROZM_BUF_WY_JPEG; n++)
			chBuforJpeg[m][n] = 0;
	}

	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_444_SUBSAMPLING, 80);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);

	for (uint16_t my=0; my<chLiczbaMCU_Wys; my++)	//pętla pracująca na MCU wysokości obrazu
	{
		nOffsetMCU_wzgObrazu = my * chLiczbaMCU_Szer * 3 * ROZMIAR_BLOKU;
		for (uint16_t mx=0; mx<chLiczbaMCU_Szer; mx++)	//pętla pracująca na MCU szerokości obrazu
		{
			//rozpocznij formowanie MCU składajacego sie z 3 bloków 8x8: Y, Cr, i Cb z danych ułożonych sekwencyjnie: YUVYUV (U=Cb, V=Cr)
			nOffsetMCU_wzgWiersza = mx * 3 * SZEROKOSC_BLOKU;					//przesunięcie o liczbę MCU w bieżącym wierszu
			for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)	//pętla pracująca na wysokosci wewnątrz MCU
			{
				nOffset_wzgMCU = (y * chLiczbaMCU_Szer * 3 * SZEROKOSC_BLOKU);	//przesunięcie pełnych wierszy obrazu

				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)	//pętla pracująca na szerokości wewnątrz MCU
				{
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + 0 * ROZMIAR_BLOKU] = *(chObrazWe + 3 * x + 0 + nOffset_wzgMCU + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);	//Y
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + 1 * ROZMIAR_BLOKU] = *(chObrazWe + 3 * x + 1 + nOffset_wzgMCU + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);	//U=Cb
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + 2 * ROZMIAR_BLOKU] = *(chObrazWe + 3 * x + 2 + nOffset_wzgMCU + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);	//V=Cr
				}
			}

			//MCU gotowe
			chWskNapBufMcu += 3;
			sZajetoscBuforaWeJpeg += 3 * ROZMIAR_BLOKU;

			//jeżeli bufor wejściowy jest przepełniony, to przerwij pracę i posprzątaj po sobie
			if (sZajetoscBuforaWeJpeg > (ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU))
			{
				chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//wymuś zamknięcie otwartego pliku
				printf("Zatrzymano, ");
				return ERR_BUF_OVERRUN;
			}

			//kompresja jednego MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie, niech się kompresuje na bieżąco
			if (hjpeg.State == HAL_JPEG_STATE_READY)
			{
				HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, 3 * ROZMIAR_BLOKU);	//przekaż bufor z nowymi danymi
				chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, 3 * ROZMIAR_BLOKU, chBuforJpeg[chWskNapBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
			}
			else
				chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję

			if (chStatusBufJpeg & STAT_JPG_OTWORZ)
				osDelay(5);	//daj czas na otwarcie pliku
		}
	}

	chErr = CzekajNaKoniecPracyJPEG();		//czekaj aż ostatnie MCU zostanie skompresowane
	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Przygotuj pojedynczy MCU czyli zestaw bloków 8x8 dla sprzętowego kompresora w formacie YUV420: Y, Y, Y, Y, Cb, Cr
// Do kompresji wybieram format YUV420 jako najbardziej efektywny. Minimalna jednostka MCU zawiera 4 sąsiadująceze sobą bloki luminancji i 2 bloki chrominancji:
// niebieski Cb i czerwony Cr. Każdy z bloków luminancji jest fragmentem obrazu Y8 (czarnio-białego) 8x8 pikseli
// W MCU znjdują się 4 sąsiadujace ze sobą bloki chrominancji, czyli obszar (2x2) x (8x8). Odpowiada obszarowi 16x16 ale nie liniowo tylko w kratkę.
// Kolejne MCU są brane aż do końca wiersza MCU, potem kolejny wiersz MCU i tak aż do konca obrazu i podawane na bieżąco do enkodera
// Parametry:
// [we] *chObrazWe - wskaźnik na kolorowy obraz wejsciowy YUV420 na razie kompresowany jako czarno-biały z pominięciem chrominancji
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 16
// [wy] *chDaneSkompresowane - wskaźnik na bufor danych skompresowanych
// [we] nRozmiarBuforaJPEG - wielkość bufora na dane wyjsciowe
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nOffsetMCU_wzgObrazu;	//przesuniecie bloku wzgledem początku obrazu
	uint32_t nOffsetMCU_wzgWiersza;	//przesunięcie obrazu wejsciowego wzgledem początku bieżącego wiersza MCU
	uint32_t nOffsetObrazuWe;		//przesunęcie obrazu wejściowego względem początku MCU
	uint8_t chLiczbaMCU_Szer = sSzerokosc / (2 * SZEROKOSC_BLOKU);	//liczba MCU luminancji (czwórek bloków) po szerokosci obrazu
	uint8_t chLiczbaMCU_Wys = sWysokosc / (2 * WYSOKOSC_BLOKU);	//liczba MCU luminancji (czwórek bloków) po wysokosci obrazu

	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_420_SUBSAMPLING, 60);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);

	for (uint16_t my=0; my<chLiczbaMCU_Wys; my++)	//pętla pracująca na wysokości obrazu
	{
		nOffsetMCU_wzgObrazu = my * chLiczbaMCU_Szer * 4 * ROZMIAR_BLOKU;
		for (uint16_t mx=0; mx<chLiczbaMCU_Szer; mx++)	//pętla pracująca na szerokości obrazu
		{
			//rozpocznij formowanie MCU
			nOffsetMCU_wzgWiersza = (mx * 2 * SZEROKOSC_BLOKU);					//przesunięcie o liczbę MCU w bieżącym wierszu
			for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)	//pętla pracująca na wysokosci bloków wewnątrz całego MCU
			{
				//pierwszy blok luminancji Y
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (0 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + (0 * ROZMIAR_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);

				//następny z prawej blok luminancji Y
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (1 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + (1 * ROZMIAR_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);

				//trzeci blok luminancji Y poniżej pierwszego, przesuniety o 2 bloki i liczbę MCU w szerokości obrazu
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (chLiczbaMCU_Szer * 2 * ROZMIAR_BLOKU)		//przesunięcie o pierwszy wiersz bloków
								+ (0 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + (2 * ROZMIAR_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);

				//czwarty blok luminancji Y
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (chLiczbaMCU_Szer * 2 * ROZMIAR_BLOKU)		//przesunięcie o pierwszy wiersz bloków
								+ (1 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + (3 * ROZMIAR_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);


				//blok chrominancji Cb
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (0 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU

				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + (4 * ROZMIAR_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);

				//blok chrominancji Cr
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (0 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU

				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (y * SZEROKOSC_BLOKU) + (5 * ROZMIAR_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);
			}

			//MCU gotowe
			chWskNapBufMcu += 6;
			sZajetoscBuforaWeJpeg += 6 * ROZMIAR_BLOKU;

			//jeżeli bufor wejściowy jest przepełniony, to przerwij pracę i posprzątaj po sobie
			if (sZajetoscBuforaWeJpeg > (ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU))
			{
				chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//wymuś zamknięcie otwartego pliku
				printf("Zatrzymano, ");
				return ERR_BUF_OVERRUN;
			}

			//kompresja jednego MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie, niech się kompresuje na bieżąco
			if (hjpeg.State == HAL_JPEG_STATE_READY)
			{
				HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, 6 * ROZMIAR_BLOKU);	//przekaż bufor z nowymi danymi
				chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, 6 * ROZMIAR_BLOKU, chBuforJpeg[chWskNapBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
			}
			else
				chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję

			if (chStatusBufJpeg & STAT_JPG_OTWORZ)
				osDelay(5);	//daj czas na otwarcie pliku
		}
	}

	chErr = CzekajNaKoniecPracyJPEG();		//czekaj aż ostatnie MCU zostanie skompresowane
	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kompresuj obraz RGB888 w formacie Y8
// Parametry:
// [we] *obrazRGB888 - wskaźnik na bufor z obrazem wejściowym
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujRGB888doY8(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc)
{
	uint32_t nOffsetWiersza, nOffsetBloku, nOffsetWierszaBlokow;
	uint32_t nOfsetWe ;
	uint32_t nOffsetWyjscia;
	uint16_t chLiczbaParBlokow = sSzerokosc / 16;
	uint8_t chR, chG, chB;
	uint8_t chY1, chY2;
	int8_t chCb1, chCr1, chCb2, chCr2;
	uint8_t chErr, chDaneDoZapisu;
	uint16_t sIloscBlokowPionowo = sWysokosc / 8;
	uint8_t chLicznikTimeoutu = 0;

	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_GRAYSCALE_COLORSPACE, JPEG_444_SUBSAMPLING, chJakosc);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);

	for (uint16_t by=0; by<sIloscBlokowPionowo; by++)	//pętla po wierszu bloków 8x8 na wysokości obrazu
	{
		nOffsetWierszaBlokow = by * chLiczbaParBlokow * 6 * ROZMIAR_BLOKU;
		for (uint8_t bx=0; bx<chLiczbaParBlokow; bx++)	//petla po połowie bloków na szerokosci obrazu - obrabiane są 2 bloki jednocześnie, bo mają mieć wspólną chrominancję
		{
			//rozpocznij formowanie MCU
			nOffsetBloku =  2 * bx * SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
			nOffsetBloku += nOffsetWierszaBlokow;
			for (uint8_t y=0; y<WYSOKOSC_BLOKU; y++)				//pętla po wierszach bloku
			{
				nOffsetWiersza = y * sSzerokosc * LICZBA_KOLOR_RGB888;
				nOffsetWiersza += nOffsetBloku;
				for (uint8_t x=0; x<SZEROKOSC_BLOKU; x++)			//pętla po  kolumnach bloku
				{
					nOfsetWe = nOffsetWiersza + LICZBA_KOLOR_RGB888 * x;
					chR = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku lewego
					chG = *(obrazRGB888 + nOfsetWe + 1);
					chB = *(obrazRGB888 + nOfsetWe + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY1, &chCb1, &chCr1);

					nOfsetWe += SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
					chR = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku prawego
					chG = *(obrazRGB888 + nOfsetWe + 1);
					chB = *(obrazRGB888 + nOfsetWe + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY2, &chCb2, &chCr2);

					//Formowanie MCU
					nOffsetWyjscia = (chWskNapBufMcu * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU) + x;
					chBuforMCU[nOffsetWyjscia + 0 * ROZMIAR_BLOKU] = chY1;
					chBuforMCU[nOffsetWyjscia + 1 * ROZMIAR_BLOKU] = chY2;
				}		//pętla po  kolumnach bloku
			} 		//pętla po wierszach bloku

			chWskNapBufMcu += 2;	//produkowane są jednoczesnie po 2 bloki wejściowe
			sZajetoscBuforaWeJpeg += 2 * ROZMIAR_BLOKU;

			//jeżeli bufor wejściowy jest przepełniony, to przerwij pracę i posprzątaj po sobie
			if (sZajetoscBuforaWeJpeg > (ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU))
			{
				chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//wymuś zamknięcie otwartego pliku
				printf("Zatrzymano, ");
				return ERR_BUF_OVERRUN;
			}

			//używam podwójnego buforowania, więc dzielę bufor na 2 części. Jedna jest napełnianam tutaj, druga opróżniana przez MDMA Jpega
			if ((chWskNapBufMcu == ILOSC_BUF_WE_MCU/2) || (chWskNapBufMcu == ILOSC_BUF_WE_MCU))		//czy mamy już połowę bufora lub cały bufor? Dla Y8 1 blok = 1 MCU
			{
				//wskaż na źródło danych w pierwszej lub drugiej połowie bufora
				if (chWskNapBufMcu == ILOSC_BUF_WE_MCU/2)
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//pierwsza połowa bufora
				else
				{
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU + ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//druga połowa
					chWskNapBufMcu = 0;	//zacznij napełniać od początku
				}

				//kompresja bufora MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie
				chDaneDoZapisu = 1;
				do
				{
					if (hjpeg.State == HAL_JPEG_STATE_READY)
					{
						chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, ROZMIAR_BLOKU * ILOSC_BUF_WE_MCU / 2, chBuforJpeg[chWskNapBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
						chDaneDoZapisu = 0;
						printf("Start, ");
					}
					else
					{
						if (chStatusBufJpeg & STAT_JPG_ZATRZYMANE_WE)
						{
							chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wznów kompresję
							chStatusBufJpeg &= ~STAT_JPG_ZATRZYMANE_WE;
							chDaneDoZapisu = 0;
							//printf("Wzn, ");
						}
						else
						{
							printf("Cze, ");
							osDelay(2);
							chLicznikTimeoutu++;
							if (chLicznikTimeoutu > 100)
							{
								printf("Timeout\r\n");
								chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
								return BLAD_TIMEOUT;
							}
						}
					}
					if (chStatusBufJpeg & STAT_JPG_OTWORZ)
						osDelay(5);	//daj czas na otwarcie pliku
				} while (chDaneDoZapisu);

			}		// czy połowa lub cały bufor
		}		//petla po połowie bloków na szerokosci obrazu
	}		//pętla po wierszu bloków 8x8 na wysokości obrazu

	//Obsługa końcówki obrazu jeżeli nie jest podzielna przez ILOSC_BUF_WE_MCU / 2
	if (chWskNapBufMcu)
	{
		if (chWskNapBufMcu <= ILOSC_BUF_WE_MCU/2)
			HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, chWskNapBufMcu * ROZMIAR_BLOKU);	//pierwsza połowa bufora
		else
		{
			HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU + ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU, (ILOSC_BUF_WE_MCU - chWskNapBufMcu) * ROZMIAR_BLOKU);	//druga połowa
			chWskNapBufMcu = 0;	//zacznij napełniać od początku
		}

		chDaneDoZapisu = 1;
		do
		{
			if (chStatusBufJpeg & STAT_JPG_ZATRZYMANE_WE)
			{
				chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wznów kompresję
				chStatusBufJpeg &= ~STAT_JPG_ZATRZYMANE_WE;
				chDaneDoZapisu = 0;
				printf("Ostatnia porcja %d MCU, ", chWskNapBufMcu);
			}
			else
				osDelay(1);
		} while (chDaneDoZapisu);
	}

	chErr = CzekajNaKoniecPracyJPEG();
	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kompresuj obraz RGB888 w formacie YUV444
// Parametry:
// [we] *obrazRGB888 - wskaźnik na bufor z obrazem wejściowym
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujRGB888doYUV444(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc)
{
	uint32_t nOffsetWiersza, nOffsetBloku, nOffsetWierszaBlokow;
	uint32_t nOfsetWe ;
	uint32_t nOffsetWyjscia;
	uint16_t chLiczbaBlokow = sSzerokosc / 8;
	uint8_t chR, chG, chB;
	uint8_t chY;
	int8_t chCb, chCr;
	uint8_t chErr, chDaneDoZapisu;
	uint16_t sIloscBlokowPionowo = sWysokosc / 8;
	uint8_t chLicznikTimeoutu = 0;

	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_444_SUBSAMPLING, chJakosc);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);

	for (uint16_t by=0; by<sIloscBlokowPionowo; by++)	//pętla po wierszu bloków 8x8 na wysokości obrazu
	{
		nOffsetWierszaBlokow = by * chLiczbaBlokow * LICZBA_KOLOR_RGB888 * ROZMIAR_BLOKU;
		for (uint8_t bx=0; bx<chLiczbaBlokow; bx++)	//petla blokach na szerokosci obrazu
		{
			//rozpocznij formowanie MCU
			nOffsetBloku =  bx * SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
			nOffsetBloku += nOffsetWierszaBlokow;
			for (uint8_t y=0; y<WYSOKOSC_BLOKU; y++)				//pętla po wierszach bloku
			{
				nOffsetWiersza = y * sSzerokosc * LICZBA_KOLOR_RGB888;
				nOffsetWiersza += nOffsetBloku;
				for (uint8_t x=0; x<SZEROKOSC_BLOKU; x++)			//pętla po  kolumnach bloku
				{
					nOfsetWe = nOffsetWiersza + LICZBA_KOLOR_RGB888 * x;
					chR = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku
					chG = *(obrazRGB888 + nOfsetWe + 1);
					chB = *(obrazRGB888 + nOfsetWe + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY, &chCb, &chCr);

					//Formowanie MCU
					nOffsetWyjscia = (chWskNapBufMcu * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU) + x;
					chBuforMCU[nOffsetWyjscia + 0 * ROZMIAR_BLOKU] = chY;
					chBuforMCU[nOffsetWyjscia + 1 * ROZMIAR_BLOKU] = chCb;
					chBuforMCU[nOffsetWyjscia + 2 * ROZMIAR_BLOKU] = chCr;
				}
			}

			//MCU gotowe
			chWskNapBufMcu += 3;
			sZajetoscBuforaWeJpeg += 3 * ROZMIAR_BLOKU;

			//jeżeli bufor wejściowy jest przepełniony, to przerwij pracę i posprzątaj po sobie
			if (sZajetoscBuforaWeJpeg > (ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU))
			{
				chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//wymuś zamknięcie otwartego pliku
				printf("Zatrzymano, ");
				return ERR_BUF_OVERRUN;
			}

			//używam podwójnego buforowania, więc dzielę bufor na 2 części. Jedna jest napełnianam tutaj, druga opróżniana przez MDMA Jpega
			if ((chWskNapBufMcu == ILOSC_BUF_WE_MCU/2) || (chWskNapBufMcu == ILOSC_BUF_WE_MCU))		//czy mamy już połowę bufora lub cały bufor? Dla Y8 1 blok = 1 MCU
			{
				//wskaż na źródło danych w pierwszej lub drugiej połowie bufora
				if (chWskNapBufMcu == ILOSC_BUF_WE_MCU/2)
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//pierwsza połowa bufora
				else
				{
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU + ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//druga połowa
					chWskNapBufMcu = 0;	//zacznij napełniać od początku
				}

				//kompresja bufora MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie
				chDaneDoZapisu = 1;
				do
				{
					if (hjpeg.State == HAL_JPEG_STATE_READY)
					{
						//czyść cache
						SCB_CleanInvalidateDCache_by_Addr((uint32_t*)chBuforMCU, ROZMIAR_BLOKU * ILOSC_BUF_WE_MCU / 4);
						SCB_CleanInvalidateDCache_by_Addr((uint32_t*)chBuforJpeg, ROZM_BUF_WY_JPEG);
						chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, ROZMIAR_BLOKU * ILOSC_BUF_WE_MCU / 2, chBuforJpeg[chWskNapBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
						chDaneDoZapisu = 0;
						printf("Start, ");
					}
					else
					{
						if (chStatusBufJpeg & STAT_JPG_ZATRZYMANE_WE)
						{
							chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wznów kompresję
							chStatusBufJpeg &= ~STAT_JPG_ZATRZYMANE_WE;
							chDaneDoZapisu = 0;
							chLicznikTimeoutu = 0;
							//printf("Wzn, ");
						}
						else
						{
							printf("Cze, ");
							osDelay(2);
							chLicznikTimeoutu++;
							if (chLicznikTimeoutu > 100)
							{
								printf("Timeout\r\n");
								chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
								return BLAD_TIMEOUT;
							}
						}
					}
					if (chStatusBufJpeg & STAT_JPG_OTWORZ)
						osDelay(5);	//daj czas na otwarcie pliku
				} while (chDaneDoZapisu);

			}	// czy połowa lub cały bufor
		}	//bx
	}	//by

	chErr = CzekajNaKoniecPracyJPEG();
	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Kompresuj obraz RGB888 w formacie YUV422
// Parametry:
// [we] *obrazRGB888 - wskaźnik na bufor z obrazem wejściowym
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujRGB888doYUV422(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc)
{
	uint32_t nOffsetWiersza, nOffsetBloku, nOffsetWierszaBlokow;
	uint32_t nOfsetWe ;
	uint32_t nOffsetWyjscia;
	uint16_t chLiczbaParBlokow = sSzerokosc / 16;
	uint8_t chR, chG, chB;
	uint8_t chY1, chY2;
	int8_t chCb1, chCr1, chCb2, chCr2;
	uint8_t chErr, chDaneDoZapisu;
	uint16_t sIloscBlokowPionowo = sWysokosc / 8;
	uint8_t chLicznikTimeoutu = 0;

	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_422_SUBSAMPLING, chJakosc);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);

	for (uint16_t by=0; by<sIloscBlokowPionowo; by++)	//pętla po wierszu bloków 8x8 na wysokości obrazu
	{
		nOffsetWierszaBlokow = by * chLiczbaParBlokow * 2 * LICZBA_KOLOR_RGB888 * ROZMIAR_BLOKU;
		for (uint8_t bx=0; bx<chLiczbaParBlokow; bx++)	//pętla po połowie bloków na szerokosci obrazu
		{
			//rozpocznij formowanie MCU
			nOffsetBloku =  2 * bx * SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
			nOffsetBloku += nOffsetWierszaBlokow;
			for (uint8_t y=0; y<WYSOKOSC_BLOKU; y++)				//pętla po wierszach
			{
				nOffsetWiersza = y * sSzerokosc * LICZBA_KOLOR_RGB888;
				nOffsetWiersza += nOffsetBloku;
				for (uint8_t x=0; x<SZEROKOSC_BLOKU; x++)			//pętla po  kolumnach
				{
					nOfsetWe = nOffsetWiersza + LICZBA_KOLOR_RGB888 * x;
					chR = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku lewego
					chG = *(obrazRGB888 + nOfsetWe + 1);
					chB = *(obrazRGB888 + nOfsetWe + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY1, &chCb1, &chCr1);

					nOfsetWe += SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
					chR = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku prawego
					chG = *(obrazRGB888 + nOfsetWe + 1);
					chB = *(obrazRGB888 + nOfsetWe + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY2, &chCb2, &chCr2);

					//Formowanie MCU
					nOffsetWyjscia = (chWskNapBufMcu * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU) + x;
					chBuforMCU[nOffsetWyjscia + 0 * ROZMIAR_BLOKU] = chY1;
					chBuforMCU[nOffsetWyjscia + 1 * ROZMIAR_BLOKU] = chY2;
					chBuforMCU[nOffsetWyjscia + 2 * ROZMIAR_BLOKU] = chCb1;
					chBuforMCU[nOffsetWyjscia + 3 * ROZMIAR_BLOKU] = chCr1;
				}
			}

			//MCU gotowe
			chWskNapBufMcu += 4;
			sZajetoscBuforaWeJpeg += 4 * ROZMIAR_BLOKU;

			//jeżeli bufor wejściowy jest przepełniony, to przerwij pracę i posprzątaj po sobie
			if (sZajetoscBuforaWeJpeg > (ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU))
			{
				chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//wymuś zamknięcie otwartego pliku
				printf("Zatrzymano, ");
				return ERR_BUF_OVERRUN;
			}

			//używam podwójnego buforowania, więc dzielę bufor na 2 części. Jedna jest napełnianam tutaj, druga opróżniana przez MDMA Jpega
			if ((chWskNapBufMcu == ILOSC_BUF_WE_MCU/2) || (chWskNapBufMcu == ILOSC_BUF_WE_MCU))		//czy mamy już połowę bufora lub cały bufor? Dla Y8 1 blok = 1 MCU
			{
				//wskaż na źródło danych w pierwszej lub drugiej połowie bufora
				if (chWskNapBufMcu == ILOSC_BUF_WE_MCU/2)
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//pierwsza połowa bufora
				else
				{
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU + ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU);	//druga połowa
					chWskNapBufMcu = 0;	//zacznij napełniać od początku
				}

				//kompresja bufora MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie
				chDaneDoZapisu = 1;
				do
				{
					if (hjpeg.State == HAL_JPEG_STATE_READY)
					{
						chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, ROZMIAR_BLOKU * ILOSC_BUF_WE_MCU / 2, chBuforJpeg[chWskNapBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
						chDaneDoZapisu = 0;
						printf("Start, ");
					}
					else
					{
						if (chStatusBufJpeg & STAT_JPG_ZATRZYMANE_WE)
						{
							chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wznów kompresję
							chStatusBufJpeg &= ~STAT_JPG_ZATRZYMANE_WE;
							chDaneDoZapisu = 0;
							chLicznikTimeoutu = 0;
							//printf("Wzn, ");
						}
						else
						{
							printf("Cze, ");
							osDelay(2);
							chLicznikTimeoutu++;
							if (chLicznikTimeoutu > 100)
							{
								printf("Timeout\r\n");
								chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
								return BLAD_TIMEOUT;
							}
						}
					}
					if (chStatusBufJpeg & STAT_JPG_OTWORZ)
						osDelay(5);	//daj czas na otwarcie pliku
				} while (chDaneDoZapisu);

			}	// czy połowa lub cały bufor
		}	//bx
	}	//by

	chErr = CzekajNaKoniecPracyJPEG();
	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje obraz RGB na YCbCr i zapisuje jedną składową do pliku BMP
// Parametry:
// [we] *obrazRGB888 - wskaźnik na bufor z obrazem wejściowym
// [wy] *buforWy - wskaźnik na bufor obrazu YCbCr do zapisu
// [wy] *chDaneSkompresowane - wskaźnik na bufor danych skompresowanych
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t TestKonwersjiRGB888doYCbCr(uint8_t *obrazRGB888, uint8_t *buforWy, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chR, chG, chB;
	uint8_t chY;
	int8_t chCb, chCr;
	uint32_t nOfsetWiersza, nOfsetWe, nOfsetWy;

	for (uint16_t y=0; y<sWysokosc; y++)
	{
		nOfsetWiersza = y * sSzerokosc * LICZBA_KOLOR_RGB888;
		for (uint16_t x=0; x<sSzerokosc; x++)
		{
			nOfsetWe = nOfsetWiersza + x * LICZBA_KOLOR_RGB888;
			chR = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku lewego
			chG = *(obrazRGB888 + nOfsetWe + 1);
			chB = *(obrazRGB888 + nOfsetWe + 2);
			KonwersjaRGB888doYCbCr(chR, chG, chB, &chY, &chCb, &chCr);

			nOfsetWy = y * sSzerokosc + x;
			*(buforWy + nOfsetWy) = chY;		//zapisz luminancję
			//*(buforWy + nOfsetWy) = chCb;		//zapisz chrominancję niebieską
			//*(buforWy + nOfsetWy) = chCr;		//zapisz chrominancję czerwoną
		}
	}
	return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Kompresuj obraz RGB888 na YUV422 - przykład dołączony przez STM
// ze wzgledu na niewielki bufor wyjściowy, który jest przewidziany do pracy porcjami obrazu na bieząco,
// należy podać niewieli obraz aby nie porzepełnić bufora danych skompresowanych o rozmiarze 4 x ROZM_BUF_WY_JPEG
// Parametry:
// [we] *obrazRGB888 - wskaźnik na bufor z obrazem wejściowym
// [wy] *chMCU - wskaźnik na bufor obrazu pośredniego zawierajacy poukładane MCU
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujPrzyklad(uint8_t *obrazRGB888,  uint8_t *chMCU, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr = 0;
	uint32_t BlockIndex = 0;
	uint32_t DataCount = sSzerokosc * sWysokosc;
	uint32_t ConvertedDataCount;
	uint32_t hMCU, vMCU;

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_422_SUBSAMPLING, 80);

	JPEG_ConvertorParams.BlockSize = ROZMIAR_MCU422;
	JPEG_ConvertorParams.Cb_MCU_LUT = JPEG_Cb_MCU_422_LUT;
	JPEG_ConvertorParams.Cr_MCU_LUT = JPEG_Cr_MCU_422_LUT;
	JPEG_ConvertorParams.Y_MCU_LUT = JPEG_Y_MCU_LUT;
	JPEG_ConvertorParams.K_MCU_LUT = JPEG_K_MCU_422_LUT;
	JPEG_ConvertorParams.ChromaSubsampling = JPEG_422_SUBSAMPLING;
	JPEG_ConvertorParams.ColorSpace = JPEG_YCBCR_COLORSPACE;
	JPEG_ConvertorParams.ImageHeight = sWysokosc;
	JPEG_ConvertorParams.ImageWidth = sSzerokosc;
	JPEG_ConvertorParams.ImageSize_Bytes = sSzerokosc * sWysokosc;
	JPEG_ConvertorParams.LineOffset =JPEG_ConvertorParams.ImageWidth % 16;
	JPEG_ConvertorParams.WidthExtend = JPEG_ConvertorParams.ImageWidth + JPEG_ConvertorParams.LineOffset;
	JPEG_ConvertorParams.ScaledWidth = JPEG_BYTES_PER_PIXEL * JPEG_ConvertorParams.ImageWidth;
	JPEG_ConvertorParams.H_factor = 16;
	JPEG_ConvertorParams.V_factor = 8;
	hMCU = (JPEG_ConvertorParams.ImageWidth / JPEG_ConvertorParams.H_factor);
	if((JPEG_ConvertorParams.ImageWidth % JPEG_ConvertorParams.H_factor) != 0)
	{
	hMCU++; /*+1 for horizenatl incomplete MCU */
	}

	vMCU = (JPEG_ConvertorParams.ImageHeight / JPEG_ConvertorParams.V_factor);
	if((JPEG_ConvertorParams.ImageHeight % JPEG_ConvertorParams.V_factor) != 0)
	{
	vMCU++; /*+1 for vertical incomplete MCU */
	}


	//wyczyść bufor danych skompresowanych
	for (uint16_t x=0; x<ILOSC_BUF_JPEG; x++)
	{
		for (uint16_t n=0; n<ROZM_BUF_WY_JPEG; n++)
			chBuforJpeg[x][n] = 0x22;
	}
	//wypełnij bufor MCU
	for (uint16_t n=0; n<DataCount; n++)
		*(chMCU + n) = 0x33;

	JPEG_ConvertorParams.MCU_Total_Nb = (hMCU * vMCU);

	chWskNapBufJpeg = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;	//otwórz plik
	JPEG_ARGB_MCU_YCbCr422_ConvertBlocks(obrazRGB888, chMCU, BlockIndex, DataCount, &ConvertedDataCount);
	HAL_JPEG_ConfigInputBuffer(&hjpeg, chMCU, ROZMIAR_MCU422 * sSzerokosc / 16 * sWysokosc);	//przekaż bufor z nowymi danymi
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);
	chErr = HAL_JPEG_Encode_DMA(&hjpeg, chMCU, ROZMIAR_MCU422 * sSzerokosc / 16 * sWysokosc, chBuforJpeg[chWskNapBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję

	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
	return chErr;
}


////////////////////////////////////////////////////////////////////////////////
// Znajduje położenie danych za znacznikiem w skompresowanych danych JPEG
// Parametry:
// [we] *chDaneSkompresowane - wskaźnik na skompresowane dane wejsciowe
// [we] chZnacznik - poszukiwany znacznik 0xFFcośtam
// Zwraca: wskaźnik na dane po znaczniku lub NULL jeżeli nie znalazło
////////////////////////////////////////////////////////////////////////////////
uint8_t* ZnajdzZnacznikJpeg(uint8_t *chDaneSkompresowane, uint8_t chZnacznik)
{
	uint8_t* chPoczatek = NULL;

	for (uint16_t n=0; n<10250; n++)	//przeszukaj początek danych
	{
		if (*(chDaneSkompresowane + n) == 0xFF)
		{
			if (*(chDaneSkompresowane + n + 1) == chZnacznik)
			{
				chPoczatek = (uint8_t*)(chDaneSkompresowane + n + 2);
				break;
			}
		}
	}
	return chPoczatek;
}



////////////////////////////////////////////////////////////////////////////////
// Buduje strukturę bloku Exif do umieszczenia w pliku jpeg
// Pierwszych 18 bajtów jest już ustawionych, trzeba wstawić kolejne tagi
// Parametry:
// [we] *stKonf - wskaźnik na konfigurację kamery
// [we] *stDane - wskaźnik na dane autopilota
// Zwraca: rozmiar struktury
////////////////////////////////////////////////////////////////////////////////
uint32_t PrzygotujExif(stKonfKam_t *stKonf, volatile stWymianyCM4_t *stDane, RTC_DateTypeDef stData, RTC_TimeTypeDef stCzas)
{
	uint32_t nRozmiar, nOffset;
	uint8_t chBufor[25];
	uint8_t *wskchAdresTAG;
	uint8_t *wskchAdresDanych;
	uint8_t *wskchAdresExif, *wskchAdresGPS;
	uint8_t *wskchPoczatekTIFF = &chNaglJpegExif[12];	//początek TIFF
	float fTemp1, fTemp2;

	chNaglJpegExif[0] = 0xFF;	//SOI
	chNaglJpegExif[1] = 0xD8;
	chNaglJpegExif[2] = 0xFF;	//APP1
	chNaglJpegExif[3] = 0xE1;
	chNaglJpegExif[4] = 0x00;	// Rozmiar (192 bajty)
	chNaglJpegExif[5] = 0xC0;
	chNaglJpegExif[6] = 'E';
	chNaglJpegExif[7] = 'x';
	chNaglJpegExif[8] = 'i';
	chNaglJpegExif[9] = 'f';
	chNaglJpegExif[10] = 0x00;
	chNaglJpegExif[11] = 0x00;

	//TIFF header (little endian)
	chNaglJpegExif[12] = 'I';		//"II" = Intel - określa porządek bajtów
	chNaglJpegExif[13] = 'I';
	chNaglJpegExif[14] = 0x2A;		//0x2A00 = magic
	chNaglJpegExif[15] = 0x00;
	chNaglJpegExif[16] = 0x08;		// offset do IFD0
	chNaglJpegExif[17] = 0x00;
	chNaglJpegExif[18] = 0x00;
	chNaglJpegExif[19] = 0x00;

	//IFD0: Liczba tagów - Interoperability number
	chNaglJpegExif[20] = (uint8_t)LICZBA_TAGOW_IFD0;
	chNaglJpegExif[21] = (uint8_t)(LICZBA_TAGOW_IFD0 >> 8);

	wskchAdresTAG = (uint8_t*)chNaglJpegExif + 22;	//adres miejsca gdzie zapisać pierwszy TAG
	wskchAdresDanych = wskchAdresTAG + (LICZBA_TAGOW_IFD0 * ROZMIAR_TAGU) + 4;	//adres za grupą tagów gdzie zapisać dane + wskaźnik 4 bajty do IFD1

	//TAG-i IFD0 ***********************************************************************************************************************************
	switch (stKonf->chFormatObrazu)
	{
	case OBR_RGB565:nRozmiar = sprintf((char*)chBufor, "%s %s YUV422 ", chNapisLcd[STR_EXIF], chNapisLcd[STR_JPEG]);	break;	//obraz kolorowy
	case OBR_Y8:	nRozmiar = sprintf((char*)chBufor, "%s %s Y8 ", chNapisLcd[STR_EXIF], chNapisLcd[STR_JPEG]);	break;		//obraz monochromatyczny
	default:		nRozmiar = sprintf((char*)chBufor, "nieznany %s ", chNapisLcd[STR_JPEG]);	break;
	}
	PrzygotujTag(&wskchAdresTAG, EXTAG_IMAGE_DESCRIPTION, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "%s ", chNapisLcd[STR_PITLAB]);
	PrzygotujTag(&wskchAdresTAG, EXTAG_IMAGE_INPUT_MAKE, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "APL hv%d.%d ",WER_GLOWNA,  WER_PODRZ);
	PrzygotujTag(&wskchAdresTAG, EXTAG_EQUIPMENT_MODEL, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "APL sv%d ", WER_REPO);
	PrzygotujTag(&wskchAdresTAG, EXTAG_SOFTWARE, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "Piotr Laskowski ");	//dodać: artystę zapisać gdzieś w osobnej strukturze
	PrzygotujTag(&wskchAdresTAG, EXTAG_ARTIST, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	chBufor[0] = 1;	//punkt (0,0) jest: 1=LGR, 2PGR, 3=DGR, 4=DLR,
	chBufor[1] = 0;	//obrócone rząd z kolumną: 5=LGR, 6=PGR, 7=DPR, 8=DLR
	//chBufor[2] = 0;
	//chBufor[3] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_ORIENTATION, EXIF_TYPE_SHORT, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);	//wstaw dane zamiast offsetu

	nRozmiar = sprintf((char*)chBufor, "%4d:%02d:%02d %02d:%02d:%02d", stData.Year + 2000, stData.Month, stData.Date, stCzas.Hours, stCzas.Minutes, stCzas.Seconds);
	PrzygotujTag(&wskchAdresTAG, EXTAG_DATE_TIME, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	nRozmiar = sprintf((char*)chBufor, "%c %s 2025 ", 0xA9, chNapisLcd[STR_PITLAB]);
	PrzygotujTag(&wskchAdresTAG, EXTAG_COPYRIGT, EXIF_TYPE_ASCII, chBufor, nRozmiar, &wskchAdresDanych, wskchPoczatekTIFF);

	wskchAdresExif = wskchAdresDanych + ROZMIAR_INTEROPER;
	wskchAdresExif = (uint8_t*)(((uint32_t)wskchAdresExif + 1) & 0xFFFFFFFE);	//wyrównanie do 2
	nOffset = (uint32_t)wskchAdresExif - (uint32_t)wskchPoczatekTIFF;	//offset do Exif IFD, czyli różnica adresów IFD0 i Exif_IFD
	chBufor[0] = (uint8_t)(nOffset);	//młodszy przodem
	chBufor[1] = (uint8_t)(nOffset >> 8);
	chBufor[2] = (uint8_t)(nOffset >> 16);
	chBufor[3] = (uint8_t)(nOffset >> 24);
	PrzygotujTag(&wskchAdresTAG, EXTAG_EXIF_IFD, EXIF_TYPE_LONG, chBufor, 4, &wskchAdresDanych, wskchPoczatekTIFF);	//rozmiar=0, dane umieść w TAGu

	wskchAdresGPS = wskchAdresExif + LICZBA_TAGOW_EXIF * (ROZMIAR_TAGU + 8) + ROZMIAR_INTEROPER;	//liczbę nadmiarowych danych tagów Exif - przyjmuję jako 8 na tag, bo to głównie Rational
	wskchAdresGPS = (uint8_t*)(((uint32_t)wskchAdresGPS + 1) & 0xFFFFFFFE);	//wyrównanie do 2
	nOffset = (uint32_t)wskchAdresGPS - (uint32_t)wskchPoczatekTIFF;	//offset do Exif IFD, czyli różnica adresów IFD0 i GPS_IFD
	chBufor[0] = (uint8_t)(nOffset);	//młodszy przodem
	chBufor[1] = (uint8_t)(nOffset >> 8);
	chBufor[2] = (uint8_t)(nOffset >> 16);
	chBufor[3] = (uint8_t)(nOffset >> 24);
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_IFD, EXIF_TYPE_LONG, chBufor, 4, &wskchAdresDanych, wskchPoczatekTIFF);		//rozmiar=0, dane umieść w TAGu

	//wskaźnik do IFD1: 0 = brak
	*(wskchAdresTAG + 0) = 0;	//młodszy przodem
	*(wskchAdresTAG + 1) = 0;
	*(wskchAdresTAG + 2) = 0;
	*(wskchAdresTAG + 3) = 0;
	//Tutaj lądują dane do których wskaźniki są podane w tagach IFD0.....

	//TAG-i Exif IFD ***********************************************************************************************************************************
	*(wskchAdresExif + 0) = (uint8_t)LICZBA_TAGOW_EXIF;			//Liczba tagów - Interoperability number
	*(wskchAdresExif + 1) = (uint8_t)(LICZBA_TAGOW_EXIF >> 8);

	wskchAdresTAG = wskchAdresExif + ROZMIAR_INTEROPER;	//adres miejsca gdzie zapisać pierwszy TAG w grupie Exif
	wskchAdresDanych = wskchAdresTAG + (LICZBA_TAGOW_EXIF * ROZMIAR_TAGU);	//adres za grupą tagów gdzie zapisać dane

	fTemp1 = stDane->fTemper[0] - 273.15f;	//Dodać temperaturę otoczenia, na razie jest temperatura IMU [°C]
	fTemp2 = floorf(fTemp1);	//pełne dziesiate części stopni
	chBufor[0] = (int8_t)fTemp2;		//liczba ze znakiem
	chBufor[1] = (int8_t)((int16_t)fTemp2 >> 8);
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_TEMPERATURE, EXIF_TYPE_SRATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//SRATIONAL x1

	fTemp2 = floorf(stDane->fCisnieBzw[0] / 100);	//ciśnienie w hPa
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = (uint8_t)((uint16_t)fTemp2 >> 8);
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// ciśnienie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_PRESSURE, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	fTemp1 = stDane->fKatIMU1[1] * 180.0f / M_PI;	//kąt pochylenie IMU w dziesiatych częściach stopnia docelowo dodać IMU kamery
	fTemp2 = floorf(fTemp1);	//pełne dziesiate części stopni
	chBufor[0] = (int8_t)fTemp2;		//liczba ze znakiem
	chBufor[1] = (int8_t)((int16_t)fTemp2 >> 8);
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_CAM_ELEVATION, EXIF_TYPE_SRATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//SRATIONAL x1

	chBufor[0] = (uint8_t)(stKonf->chSzerWe / stKonf->chSzerWy);	//zoom cyfrowy po szerokości
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// zoom / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_DIGITAL_ZOOM, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	//TAG-i GPS IFD ***********************************************************************************************************************************
	*(wskchAdresGPS + 0) = (uint8_t)LICZBA_TAGOW_GPS;			//Liczba tagów - Interoperability number
	*(wskchAdresGPS + 1) = (uint8_t)(LICZBA_TAGOW_GPS >> 8);

	wskchAdresTAG = wskchAdresGPS + ROZMIAR_INTEROPER;	//adres miejsca gdzie zapisać pierwszy TAG w grupie GPS
	wskchAdresDanych = wskchAdresTAG + (LICZBA_TAGOW_GPS * ROZMIAR_TAGU);	//adres za grupą tagów gdzie zapisać dane

	chBufor[0] = 2;
	chBufor[1] = 3;
	chBufor[2] = 0;
	chBufor[3] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_TAG_VERSION, EXIF_TYPE_BYTE, chBufor, 4, &wskchAdresDanych, wskchPoczatekTIFF);	//BYTE x4, ale nie wstawiaj ich do danych

	fTemp1 = stDane->stGnss1.dSzerokoscGeo;
	if (fTemp1 < 0)
	{
		chBufor[0] = 'S';
		fTemp1 *= -1.0f;	//zmień znak na dodatni
	}
	else
		chBufor[0] = 'N';
	chBufor[1] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_NS_LATI_REF, EXIF_TYPE_ASCII, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);	//ASCII x2
	fTemp2 = (uint8_t)floorf(fTemp1);		//pełne stopnie;
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;

	fTemp2 = (fTemp1 - fTemp2) * 60.0f;	//ułamkowa część stopni -> minuty
	fTemp1 = floorf(fTemp2);			//pełne minuty
	chBufor[8] = (uint8_t)fTemp1;
	chBufor[9] = 0;
	chBufor[10] = 0;
	chBufor[11] = 0;
	chBufor[12] = 1;		// minuty / 1
	chBufor[13] = 0;
	chBufor[14] = 0;
	chBufor[15] = 0;

	fTemp2 = (fTemp2 - fTemp1) * 6000.0f;	//ułamkowa część minut -> //sekundy * 100
	fTemp1 = floorf(fTemp2);			//pełne setki sekund
	chBufor[17] = (uint8_t)fTemp1;
	chBufor[16] = (uint8_t)((uint16_t)fTemp1 >> 8);
	chBufor[18] = 0;
	chBufor[19] = 0;
	chBufor[20] = 100;		// sekundy / 100
	chBufor[21] = 0;
	chBufor[22] = 0;
	chBufor[23] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_LATITUDE, EXIF_TYPE_RATIONAL, chBufor, 24, &wskchAdresDanych, wskchPoczatekTIFF);

	fTemp1 = stDane->stGnss1.dDlugoscGeo;
	if (fTemp1 < 0)
	{
		chBufor[0] = 'W';
		fTemp1 *= -1.0f;	//zmień znak na dodatni
	}
	else
		chBufor[0] = 'E';
	chBufor[1] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_EW_LONGI_REF, EXIF_TYPE_ASCII, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);	//ASCII x2

	//fTemp2 = fTemp1 * 180 / M_PI;	//radiany -> stopnie
	fTemp2 = floorf(fTemp1);			//pełne stopnie
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// stopnie / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;

	fTemp2 = (fTemp1 - fTemp2) * 60.0f;	//ułamkowa część stopni -> minuty
	fTemp1 = floorf(fTemp2);			//pełne minuty
	chBufor[8] = (uint8_t)fTemp1;
	chBufor[9] = 0;
	chBufor[10] = 0;
	chBufor[11] = 0;
	chBufor[12] = 1;		// minuty / 1
	chBufor[13] = 0;
	chBufor[14] = 0;
	chBufor[15] = 0;

	fTemp2 = (fTemp2 - fTemp1) * 6000.0f;		//ułamkowa część minut
	fTemp1 = floorf(fTemp2);	//pełne setki sekund
	chBufor[17] = (uint8_t)fTemp1;
	chBufor[16] = (uint8_t)((uint16_t)fTemp1 >> 8);
	chBufor[18] = 0;
	chBufor[19] = 0;
	chBufor[20] = 100;		// sekundy / 100
	chBufor[21] = 0;
	chBufor[22] = 0;
	chBufor[23] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_LONGITUDE, EXIF_TYPE_RATIONAL, chBufor, 24, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x3

	fTemp1 = stDane->stGnss1.fWysokoscMSL;
	if (fTemp1 < 0)
	{
		chBufor[0] = 1;		//1 = poniżej poziomu morza
		fTemp1 *= -1.0f;	//zamień na dodatnie
	}
	else
		chBufor[0] = 0;		//0 = powyżej poziomu morza
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_ALTITUDE_REF, EXIF_TYPE_BYTE, chBufor, 1, &wskchAdresDanych, wskchPoczatekTIFF);	//BYTE x1

	fTemp1 *= 10.0f;			//dziesiąte części metra
	fTemp2 = floorf(fTemp1);		//pełne dziesiątki
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = (uint8_t)((uint32_t)fTemp2 >> 8);
	chBufor[2] = (uint8_t)((uint32_t)fTemp2 >> 16);
	chBufor[3] = (uint8_t)((uint32_t)fTemp2 >> 24);
	chBufor[4] = 10;		// metrów / 10
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_ALTITUDE, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	//timestamp GPS
	chBufor[0] = stDane->stGnss1.chGodz;
	chBufor[1] = 0;
	chBufor[2] = 0;
	chBufor[3] = 0;
	chBufor[4] = 1;		// godz / 1
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	chBufor[8] = stDane->stGnss1.chMin;
	chBufor[9] = 0;
	chBufor[10] = 0;
	chBufor[11] = 0;
	chBufor[12] = 1;	// minuty / 1
	chBufor[13] = 0;
	chBufor[14] = 0;
	chBufor[15] = 0;
	chBufor[16] = stDane->stGnss1.chSek;
	chBufor[17] = 0;
	chBufor[18] = 0;
	chBufor[19] = 0;
	chBufor[20] = 1;	// sekundy / 1
	chBufor[21] = 0;
	chBufor[22] = 0;
	chBufor[23] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_TIME_STAMP, EXIF_TYPE_RATIONAL, chBufor, 24, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x3

	chBufor[0] = 'K';	//km/h
	chBufor[1] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_SPEED_REF, EXIF_TYPE_ASCII, chBufor, 2, &wskchAdresDanych, wskchPoczatekTIFF);		//ASCII,

	fTemp1 = stDane->stGnss1.fPredkoscWzglZiemi;	//prędkość w m/s
	fTemp2 = fTemp1 * 10.0f / 3.6f;					//prędkość w 10*km/h
	fTemp2 = floorf(fTemp1);						//pełne dziesiątki
	chBufor[0] = (uint8_t)fTemp2;
	chBufor[1] = (uint8_t)((uint32_t)fTemp2 >> 8);
	chBufor[2] = (uint8_t)((uint32_t)fTemp2 >> 16);
	chBufor[3] = (uint8_t)((uint32_t)fTemp2 >> 24);
	chBufor[4] = 10;		// km/h / 10
	chBufor[5] = 0;
	chBufor[6] = 0;
	chBufor[7] = 0;
	PrzygotujTag(&wskchAdresTAG, EXTAG_GPS_SPEED, EXIF_TYPE_RATIONAL, chBufor, 8, &wskchAdresDanych, wskchPoczatekTIFF);	//RATIONAL x1

	//aktualizuj rozmiar struktury Exif
	nRozmiar = wskchAdresDanych - (uint8_t*)chNaglJpegExif;	//rozmiar to różnica wskaźników początku i końca danych
	chNaglJpegExif[4] = (uint8_t)(nRozmiar >> 8);   // Rozmiar (big endian)
	chNaglJpegExif[5] = (uint8_t)nRozmiar;
	return nRozmiar;
}



////////////////////////////////////////////////////////////////////////////////
// Buduje strukturę tagu w Exif. Rozmiar TAG-a to 12 bajtów
// Pierwsze 2 bajty to identyfikator, koleje 2 to typ danych, kolejne 4 bajty to rozmiar, kolejne 4 to offset danych lub bezpośrednio dane
// Jeżeli podany rozmiar jest zerem to zamiast offsetu podaj dane
// Parametry:
// [wy] *nTag - wskaźnik wskaźnik na budowany TAG
// [we] sID - identyfikator TAG-u
// [we] sTyp - typ danych
// [we] *chDane wskaźnik na dane
// [we] chRozmiar - rozmiar danych wyrażony w bajtach
// [we] nOffset - offset do danych począwszy od nagłówka TIFF
// Zwraca: rozmiar struktury
////////////////////////////////////////////////////////////////////////////////
void PrzygotujTag(uint8_t **chWskTaga, uint16_t sTagID, uint16_t sTyp, uint8_t *chDane, uint32_t nRozmiar, uint8_t **chWskDanych, uint8_t *chPoczatekTIFF)
{
	uint32_t nOffset = *chWskDanych - chPoczatekTIFF;
	uint8_t chRozmiarTagu;

	//oblicz jednostkowy rozmiar tagu dla różnych typów danych
	switch (sTyp)
	{
	case EXIF_TYPE_BYTE:
	case EXIF_TYPE_ASCII:		chRozmiarTagu = nRozmiar;		break;		//ASCII
	case EXIF_TYPE_SHORT:		chRozmiarTagu = nRozmiar / 2;		break;	//SHORT
	case EXIF_TYPE_LONG:				//LONG 32-bit
	case EXIF_TYPE_SLONG:		chRozmiarTagu = nRozmiar / 4;	break;	//Signed LONG 32-bit
	case EXIF_TYPE_RATIONAL:											//RATIONAL: 2x LONG. Pierwszy numerator, drugi denominator
	case EXIF_TYPE_SRATIONAL: 	chRozmiarTagu = nRozmiar / 8;	break;	//Signed RATIONAL: 2x SLONG
	}

	//dla tagów o podanej zerowej długosci przyjmij rozmiar 1
	if (chRozmiarTagu == 0)
		chRozmiarTagu = 1;

	*(*chWskTaga +  0) = (uint8_t)(sTagID);
	*(*chWskTaga +  1) = (uint8_t)(sTagID >> 8);
	*(*chWskTaga +  2) = (uint8_t)(sTyp);
	*(*chWskTaga +  3) = (uint8_t)(sTyp >> 8);
	*(*chWskTaga +  4) = chRozmiarTagu;
	*(*chWskTaga +  5) = 0;
	*(*chWskTaga +  6) = 0;
	*(*chWskTaga +  7) = 0;
	if (nRozmiar > 4)	//jeżeli rozmiar nie zmieści się w strukturze to wstaw offset do segmentu danych, jeżeli zmieści, to wstaw 4 bajty danych zamiast offsetu
	{
		*(*chWskTaga +  8) = (uint8_t)(nOffset);			//Offset od początku nagłówka TIFF do miejsca gdzie dane są zapisane
		*(*chWskTaga +  9) = (uint8_t)(nOffset >>  8);
		*(*chWskTaga + 10) = (uint8_t)(nOffset >> 16);
		*(*chWskTaga + 11) = (uint8_t)(nOffset >> 24);

		for (uint32_t n=0; n<nRozmiar; n++)
			*(*chWskDanych + n) = *(chDane + n);
		*(*chWskDanych + nRozmiar) = 0;
		nRozmiar++;		//powiększ rozmiar o zero terminujące wartość
		*chWskDanych += nRozmiar;
	}
	else	//jeżeli rozmiar zmieści sie w 4 bajtach to go wstaw zamiast offsetu
	{
		for (uint8_t n=0; n<nRozmiar; n++)
			*(*chWskTaga + n + 8) = *(chDane + n);
		//*(*chWskTaga +  9) = *(chDane + 1);
		//*(*chWskTaga + 10) = *(chDane + 2);
		//*(*chWskTaga + 11) = *(chDane + 3);
	}
	*chWskTaga += ROZMIAR_TAGU;	//wskaż na adres następnego tagu
}



void JPEG_Init_MCU_LUT(void)
{
  uint32_t i, j, offset;

  /*Y LUT */
  for(i = 0; i < 16; i++)
  {
    for(j = 0; j < 16; j++)
    {
      offset =  j + (i*8);
      if((j>=8) && (i>=8)) offset+= 120;
      else  if((j>=8) && (i<8)) offset+= 56;
      else  if((j<8) && (i>=8)) offset+= 64;

      JPEG_Y_MCU_LUT[i*16 + j] = offset;
    }
  }

  /*Cb Cr K LUT*/
  for(i = 0; i < 16; i++)
  {
    for(j = 0; j < 16; j++)
    {
      offset = i*16 + j;

      JPEG_Cb_MCU_420_LUT[offset] = (j/2) + ((i/2)*8) + 256;
      JPEG_Cb_MCU_422_LUT[offset] = (j/2) + (i*8) + 128;

      JPEG_Cr_MCU_420_LUT[offset] = (j/2) + ((i/2)*8) + 320;
      JPEG_Cr_MCU_422_LUT[offset] = (j/2) + (i*8) + 192;

      JPEG_K_MCU_420_LUT[offset] = (j/2) + ((i/2)*8) + 384;
      JPEG_K_MCU_422_LUT[offset] = (j/2) + ((i/2)*8) + 256;
    }
  }

  for(i = 0; i < 8; i++)
  {
    for(j = 0; j < 8; j++)
    {
      offset = i*8 + j;

      JPEG_Y_MCU_444_LUT[offset]  = offset;
      JPEG_Cb_MCU_444_LUT[offset] = offset + 64 ;
      JPEG_Cr_MCU_444_LUT[offset] = offset + 128 ;
      JPEG_K_MCU_444_LUT[offset]  = offset + 192 ;
    }
  }
}


/**
  * @brief  Convert RGB to YCbCr 4:2:2 blocks pixels
  * @param  pInBuffer  : pointer to input RGB888/ARGB8888 frame buffer.
  * @param  pOutBuffer : pointer to output YCbCr blocks buffer.
  * @param  BlockIndex : index of the input buffer first block in the final image.
  * @param  DataCount  : number of bytes in the input buffer .
  * @param  ConvertedDataCount  : number of converted bytes from input buffer.
  * @retval Number of blocks converted from RGB to YCbCr
  */
uint32_t JPEG_ARGB_MCU_YCbCr422_ConvertBlocks(uint8_t *pInBuffer, uint8_t *pOutBuffer, uint32_t BlockIndex, uint32_t DataCount, uint32_t *ConvertedDataCount)
{
  uint32_t numberMCU;
  uint32_t i,j, currentMCU, xRef,yRef, colones;

  uint32_t refline;
  int32_t ycomp, crcomp, cbcomp, offset;

  uint32_t red, green, blue;

  uint8_t *pOutAddr;
  uint8_t *pInAddr;

  numberMCU = ((2 * DataCount) / (JPEG_BYTES_PER_PIXEL * YCBCR_422_BLOCK_SIZE));

  currentMCU = BlockIndex;
  *ConvertedDataCount = numberMCU * JPEG_ConvertorParams.BlockSize;

  pOutAddr = &pOutBuffer[0];

  while(currentMCU < (numberMCU + BlockIndex))
  {
    xRef = ((currentMCU *JPEG_ConvertorParams.H_factor) / JPEG_ConvertorParams.WidthExtend)*JPEG_ConvertorParams.V_factor;

    yRef = ((currentMCU *JPEG_ConvertorParams.H_factor) % JPEG_ConvertorParams.WidthExtend);


    refline = JPEG_ConvertorParams.ScaledWidth * xRef + (JPEG_BYTES_PER_PIXEL*yRef);

    currentMCU++;

    if(((currentMCU *JPEG_ConvertorParams.H_factor) % JPEG_ConvertorParams.WidthExtend) == 0)
    {
      colones = JPEG_ConvertorParams.H_factor - JPEG_ConvertorParams.LineOffset;
    }
    else
    {
      colones = JPEG_ConvertorParams.H_factor;
    }
    offset = 0;

    for(i= 0; i <  JPEG_ConvertorParams.V_factor; i+=1)
    {

      pInAddr = &pInBuffer[0] ;

      for(j=0; j < colones; j+=2)
      {
        // First Pixel
#if (JPEG_RGB_FORMAT == JPEG_RGB565)
        red   = (((*(__IO uint16_t *)(pInAddr + refline)) & JPEG_RGB565_RED_MASK)   >> JPEG_RED_OFFSET) ;
        green = (((*(__IO uint16_t *)(pInAddr + refline)) & JPEG_RGB565_GREEN_MASK) >> JPEG_GREEN_OFFSET) ;
        blue  = (((*(__IO uint16_t *)(pInAddr + refline)) & JPEG_RGB565_BLUE_MASK)  >> JPEG_BLUE_OFFSET) ;
        red   = (red << 3)   | (red >> 2);
        green = (green << 2) | (green >> 4);
        blue  = (blue << 3)  | (blue >> 2);
#else
        red   = (*(pInAddr + refline + JPEG_RED_OFFSET/8)) ;
        green = (*(pInAddr + refline + JPEG_GREEN_OFFSET/8)) ;
        blue  = (*(pInAddr + refline + JPEG_BLUE_OFFSET/8)) ;
#endif
        ycomp  = (int32_t)(*(RED_Y_LUT + red)) + (int32_t)(*(GREEN_Y_LUT + green)) + (int32_t)(*(BLUE_Y_LUT + blue));
        cbcomp = (int32_t)(*(RED_CB_LUT + red)) + (int32_t)(*(GREEN_CB_LUT + green)) + (int32_t)(*(BLUE_CB_RED_CR_LUT + blue)) + 128;
        crcomp = (int32_t)(*(BLUE_CB_RED_CR_LUT + red)) + (int32_t)(*(GREEN_CR_LUT + green)) + (int32_t)(*(BLUE_CR_LUT + blue)) + 128;

        (*(pOutAddr + JPEG_ConvertorParams.Y_MCU_LUT[offset]))  = ycomp;
        (*(pOutAddr + JPEG_ConvertorParams.Cb_MCU_LUT[offset])) = cbcomp;
        (*(pOutAddr + JPEG_ConvertorParams.Cr_MCU_LUT[offset])) = crcomp;

        // Second Pixel
#if (JPEG_RGB_FORMAT == JPEG_RGB565)
        red   = (((*(__IO uint16_t *)(pInAddr + refline + JPEG_BYTES_PER_PIXEL)) & JPEG_RGB565_RED_MASK)   >> JPEG_RED_OFFSET) ;
        green = (((*(__IO uint16_t *)(pInAddr + refline + JPEG_BYTES_PER_PIXEL)) & JPEG_RGB565_GREEN_MASK) >> JPEG_GREEN_OFFSET) ;
        blue  = (((*(__IO uint16_t *)(pInAddr + refline + JPEG_BYTES_PER_PIXEL)) & JPEG_RGB565_BLUE_MASK)  >> JPEG_BLUE_OFFSET) ;
        red   = (red << 3)   | (red >> 2);
        green = (green << 2) | (green >> 4);
        blue  = (blue << 3)  | (blue >> 2);
#else
        red   = (*(pInAddr + refline + JPEG_BYTES_PER_PIXEL + JPEG_RED_OFFSET/8)) ;
        green = (*(pInAddr + refline + JPEG_BYTES_PER_PIXEL + JPEG_GREEN_OFFSET/8)) ;
        blue  = (*(pInAddr + refline + JPEG_BYTES_PER_PIXEL + JPEG_BLUE_OFFSET/8)) ;
#endif
        ycomp  = (int32_t)(*(RED_Y_LUT + red)) + (int32_t)(*(GREEN_Y_LUT + green)) + (int32_t)(*(BLUE_Y_LUT + blue));
        (*(pOutAddr + JPEG_ConvertorParams.Y_MCU_LUT[offset + 1]))  = ycomp;



        pInAddr += JPEG_BYTES_PER_PIXEL * 2;
        offset+=2;
      }
      offset += (JPEG_ConvertorParams.H_factor - colones);
      refline += JPEG_ConvertorParams.ScaledWidth ;

    }
    pOutAddr +=  JPEG_ConvertorParams.BlockSize;
  }

  return numberMCU;
}




/**
  * @brief  Retrieve Encoding RGB to YCbCr color conversion function and block number
  * @param  pJpegInfo  : JPEG_ConfTypeDef that contains the JPEG image information.
  *                      These info are available in the HAL callback "HAL_JPEG_InfoReadyCallback".
  * @param  pFunction  : pointer to JPEG_RGBToYCbCr_Convert_Function , used to Retrieve the color conversion function
  *                      depending of the jpeg image color space and chroma sampling info.
  * @param ImageNbMCUs : pointer to uint32_t, used to Retrieve the total number of MCU blocks in the jpeg image.
  * @retval HAL status : HAL_OK or HAL_ERROR.
  */
/*HAL_StatusTypeDef JPEG_GetEncodeColorConvertFunc(JPEG_ConfTypeDef *pJpegInfo, JPEG_RGBToYCbCr_Convert_Function *pFunction, uint32_t *ImageNbMCUs)
{
  uint32_t hMCU, vMCU;

  JPEG_ConvertorParams.ColorSpace = pJpegInfo->ColorSpace;
  JPEG_ConvertorParams.ChromaSubsampling = pJpegInfo->ChromaSubsampling;

  if(JPEG_ConvertorParams.ColorSpace == JPEG_YCBCR_COLORSPACE)
  {
    if(JPEG_ConvertorParams.ChromaSubsampling == JPEG_420_SUBSAMPLING)
    {
      *pFunction =  JPEG_ARGB_MCU_YCbCr420_ConvertBlocks;
    }
    else if (JPEG_ConvertorParams.ChromaSubsampling == JPEG_422_SUBSAMPLING)
    {
      *pFunction = JPEG_ARGB_MCU_YCbCr422_ConvertBlocks;
    }
    else if (JPEG_ConvertorParams.ChromaSubsampling == JPEG_444_SUBSAMPLING)
    {
      *pFunction = JPEG_ARGB_MCU_YCbCr444_ConvertBlocks;
    }
    else
    {
       return HAL_ERROR; // Chroma SubSampling Not supported
    }
  }
  else if(JPEG_ConvertorParams.ColorSpace == JPEG_GRAYSCALE_COLORSPACE)
  {
    *pFunction = JPEG_ARGB_MCU_Gray_ConvertBlocks;
  }
  else if(JPEG_ConvertorParams.ColorSpace == JPEG_CMYK_COLORSPACE)
  {
    *pFunction = JPEG_ARGB_MCU_YCCK_ConvertBlocks;
  }
  else
  {
     return HAL_ERROR; // Color space Not supported
  }

  JPEG_ConvertorParams.ImageWidth = pJpegInfo->ImageWidth;
  JPEG_ConvertorParams.ImageHeight = pJpegInfo->ImageHeight;
  JPEG_ConvertorParams.ImageSize_Bytes = pJpegInfo->ImageWidth * pJpegInfo->ImageHeight * JPEG_BYTES_PER_PIXEL;

  if((JPEG_ConvertorParams.ChromaSubsampling == JPEG_420_SUBSAMPLING) || (JPEG_ConvertorParams.ChromaSubsampling == JPEG_422_SUBSAMPLING))
  {
    JPEG_ConvertorParams.LineOffset = JPEG_ConvertorParams.ImageWidth % 16;

    JPEG_ConvertorParams.Y_MCU_LUT = JPEG_Y_MCU_LUT;

    if(JPEG_ConvertorParams.LineOffset != 0)
    {
      JPEG_ConvertorParams.LineOffset = 16 - JPEG_ConvertorParams.LineOffset;
    }

    JPEG_ConvertorParams.H_factor = 16;

    if(JPEG_ConvertorParams.ChromaSubsampling == JPEG_420_SUBSAMPLING)
    {
      JPEG_ConvertorParams.V_factor  = 16;

      if(JPEG_ConvertorParams.ColorSpace == JPEG_YCBCR_COLORSPACE)
      {
        JPEG_ConvertorParams.BlockSize =  YCBCR_420_BLOCK_SIZE;
      }

      JPEG_ConvertorParams.Cb_MCU_LUT = JPEG_Cb_MCU_420_LUT;
      JPEG_ConvertorParams.Cr_MCU_LUT = JPEG_Cr_MCU_420_LUT;

      JPEG_ConvertorParams.K_MCU_LUT  = JPEG_K_MCU_420_LUT;
    }
    else // 4:2:2
    {
      JPEG_ConvertorParams.V_factor = 8;

      if(JPEG_ConvertorParams.ColorSpace == JPEG_YCBCR_COLORSPACE)
      {
        JPEG_ConvertorParams.BlockSize =  YCBCR_422_BLOCK_SIZE;
      }

      JPEG_ConvertorParams.Cb_MCU_LUT = JPEG_Cb_MCU_422_LUT;
      JPEG_ConvertorParams.Cr_MCU_LUT = JPEG_Cr_MCU_422_LUT;

      JPEG_ConvertorParams.K_MCU_LUT  = JPEG_K_MCU_422_LUT;
    }
  }
  else if(JPEG_ConvertorParams.ChromaSubsampling == JPEG_444_SUBSAMPLING)
  {
    JPEG_ConvertorParams.LineOffset = JPEG_ConvertorParams.ImageWidth % 8;

    JPEG_ConvertorParams.Y_MCU_LUT = JPEG_Y_MCU_444_LUT;

    JPEG_ConvertorParams.Cb_MCU_LUT = JPEG_Cb_MCU_444_LUT;
    JPEG_ConvertorParams.Cr_MCU_LUT = JPEG_Cr_MCU_444_LUT;

    JPEG_ConvertorParams.K_MCU_LUT  = JPEG_K_MCU_444_LUT;

    if(JPEG_ConvertorParams.LineOffset != 0)
    {
      JPEG_ConvertorParams.LineOffset = 8 - JPEG_ConvertorParams.LineOffset;
    }
    JPEG_ConvertorParams.H_factor = 8;
    JPEG_ConvertorParams.V_factor = 8;

    if(JPEG_ConvertorParams.ColorSpace == JPEG_YCBCR_COLORSPACE)
    {
      JPEG_ConvertorParams.BlockSize = YCBCR_444_BLOCK_SIZE;
    }
    if(JPEG_ConvertorParams.ColorSpace == JPEG_CMYK_COLORSPACE)
    {
      JPEG_ConvertorParams.BlockSize = CMYK_444_BLOCK_SIZE;
    }
    else if(JPEG_ConvertorParams.ColorSpace == JPEG_GRAYSCALE_COLORSPACE)
    {
      JPEG_ConvertorParams.BlockSize = GRAY_444_BLOCK_SIZE;
    }

  }
  else
  {
     return HAL_ERROR; // Not supported
  }


  JPEG_ConvertorParams.WidthExtend = JPEG_ConvertorParams.ImageWidth + JPEG_ConvertorParams.LineOffset;
  JPEG_ConvertorParams.ScaledWidth = JPEG_BYTES_PER_PIXEL * JPEG_ConvertorParams.ImageWidth;

  hMCU = (JPEG_ConvertorParams.ImageWidth / JPEG_ConvertorParams.H_factor);
  if((JPEG_ConvertorParams.ImageWidth % JPEG_ConvertorParams.H_factor) != 0)
  {
    hMCU++; //+1 for horizenatl incomplete MCU
  }

  vMCU = (JPEG_ConvertorParams.ImageHeight / JPEG_ConvertorParams.V_factor);
  if((JPEG_ConvertorParams.ImageHeight % JPEG_ConvertorParams.V_factor) != 0)
  {
    vMCU++; //+1 for vertical incomplete MCU
  }
  JPEG_ConvertorParams.MCU_Total_Nb = (hMCU * vMCU);
  *ImageNbMCUs = JPEG_ConvertorParams.MCU_Total_Nb;

  return HAL_OK;
}*/



// 1 MCU dla YUV444 (192 B) lub YUV422 (256 B)
#define TEST_IS_YUV444 1
#if TEST_IS_YUV444
  #define TEST_MCU_SIZE  (3*64)   // 192
#else
  #define TEST_MCU_SIZE  (4*64)   // 256
#endif

uint8_t one_mcu[TEST_MCU_SIZE] __attribute__((aligned(32)));
uint8_t out_buf[4096] __attribute__((aligned(32)));

void test_one_mcu_encode(void)
{
    // fill neutral values: luma ~128, chroma ~128
    for (int i=0;i<TEST_MCU_SIZE;i++) one_mcu[i] = 128;

    // clean cache
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)one_mcu, TEST_MCU_SIZE);
    SCB_CleanInvalidateDCache_by_Addr((uint32_t*)out_buf, sizeof(out_buf));

    // start encoding
    HAL_StatusTypeDef st = HAL_JPEG_Encode_DMA(&hjpeg, one_mcu, TEST_MCU_SIZE, out_buf, sizeof(out_buf));
    printf("HAL_JPEG_Encode_DMA() => %d\n", (int)st);

    // poll status + DMA counters (10 s timeout)
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < 10000)
    {
        // print hjpeg state & error code occasionally
        static uint32_t last = 0;
        if (HAL_GetTick() - last >= 200) {
            last = HAL_GetTick();
            printf("t=%lu ms: hjpeg.State=%d, ErrorCode=0x%08lX\n",
                   (unsigned long)(HAL_GetTick()-start), (int)hjpeg.State,
                   (unsigned long)hjpeg.ErrorCode);

            // print DMA/MDMA info if available
            if (hjpeg.hdmain != NULL && hjpeg.hdmain->Instance != NULL) {
                printf("  HDMAIN CBNDTR=%lu CMAR=%ld CDAR=%ld\n",
                       (unsigned long)hjpeg.hdmain->Instance->CBNDTR, hjpeg.hdmain->Instance->CMAR, hjpeg.hdmain->Instance->CDAR);
            }
            if (hjpeg.hdmaout != NULL && hjpeg.hdmaout->Instance != NULL) {
                printf("  HDMAOUT CBNDTR=%lu CMAR=%ld CDAR=%ld\n",
                       (unsigned long)hjpeg.hdmaout->Instance->CBNDTR, hjpeg.hdmaout->Instance->CMAR, hjpeg.hdmaout->Instance->CDAR);
            }
        }

        // break if encoder finished or error
        if (hjpeg.State == HAL_JPEG_STATE_READY) { printf("ENCODE DONE (READY)\n"); break; }
        if (hjpeg.ErrorCode != HAL_JPEG_ERROR_NONE) { printf("ENCODE ERROR 0x%08lX\n", (unsigned long)hjpeg.ErrorCode); break; }

        osDelay(10);
    }
}
