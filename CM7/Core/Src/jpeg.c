//////////////////////////////////////////////////////////////////////////////
//
// Obsługa sprzętowej kompresji obrazu
// Opracowane na podstawie AN4996 v4
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "jpeg.h"
#include "exif.h"
#include "cmsis_os.h"
#include "ff.h"
#include "czas.h"
#include "rejestrator.h"
#include "analiza_obrazu.h"
#include "kamera.h"



extern JPEG_HandleTypeDef hjpeg;
extern MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
extern MDMA_HandleTypeDef hmdma_jpeg_infifo_th;

//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM2"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG] = {0};	//są problemy z zapisem na kartę
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG] = {0};
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG] = {0};
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM2"))) chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU] = {0}; - wywala się ethernet, być może dane wchodzą na stos
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".Bufory_SRAM3"))) chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU] = {0};	- błąd DMA
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaAxiSRAM")))chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU] = {0};

volatile uint8_t chStatusBufJpeg;	//przechowyje bity okreslające status procesu przepływu danych z bufora danych skompresowanych
volatile uint8_t chWskNapBufJpeg, chWskOprBufJpeg;	// wskazuje do którego bufora obecnie są zapisywane dane i z którego są odczytywane
uint32_t nRozmiarObrazuJPEG;	//w bajtach
uint8_t chWynikKompresji;		//flagi ustawiane w callbackach, określajace postęp przepływu danych przez enkoder JPEG
uint8_t chWskNapBufMcu;			//wskaźnik napełniania buforów MCU
volatile uint16_t sZajetoscBuforaWeJpeg, sZajetoscBuforaWyJpeg;		//liczba bajtów w buforach wejściowym i wyjściowym kompresora
extern uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
JPEG_ConfTypeDef stKonfJpeg;	//struktura konfiguracyjna JPEGa
const uint8_t chNaglJpegEOI[ROZMIAR_ZNACZ_xOI] = {0xFF, 0xD9};	//EOI (End Of Image) - zapisywany na końcu pliku jpeg



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
uint8_t KonfigurujKompresjeJpeg(JPEG_ConfTypeDef *stKonfJpeg, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu)
{
	if (hjpeg.State == HAL_JPEG_STATE_ERROR)
		HAL_JPEG_Abort(&hjpeg);
	stKonfJpeg->ColorSpace = chTypKoloru;					//JPEG_GRAYSCALE_COLORSPACE, JPEG_YCBCR_COLORSPACE
	stKonfJpeg->ChromaSubsampling = chTypChrominancji;	//JPEG_444_SUBSAMPLING, JPEG_422_SUBSAMPLING, JPEG_420_SUBSAMPLING
	stKonfJpeg->ImageWidth = sSzerokosc;
	stKonfJpeg->ImageHeight = sWysokosc;
	stKonfJpeg->ImageQuality = chJakoscObrazu;
	return HAL_JPEG_ConfigEncoding(&hjpeg, stKonfJpeg);
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
// Można podać enkoderowi kolejne dane
// Callback HAL_JPEG_GetDataCallback is asserted for both encoding and decoding
// operations to inform the application that the input buffer has been
// consumed by the peripheral and to ask for a new data chunk if the operation
// (encoding/decoding) has not been complete yet.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{
	sZajetoscBuforaWeJpeg -= NbDecodedData;
	if (sZajetoscBuforaWeJpeg)	//jeżeli po pobraniu danych w buforze wejściowym są jeszcze dane, to kompresuj je
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
// Skompresowane dane sa gotowe
// Callback HAL_JPEG_DataReadyCallback is asserted when the HAL JPEG driver has filled the given output buffer with the given size.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
	if (sZajetoscBuforaWyJpeg < ILOSC_BUF_JPEG * ROZM_BUF_WY_JPEG)
	{
		chWskNapBufJpeg++;							//przełącz bufor danych skompresowanych na następny
		chWskNapBufJpeg &= MASKA_LICZBY_BUF;
		sZajetoscBuforaWyJpeg += OutDataLength;
		nRozmiarObrazuJPEG += OutDataLength;
		printf("Rozm: %ld wsk: %d buf: %d\r\n",  nRozmiarObrazuJPEG, chWskNapBufJpeg, sZajetoscBuforaWyJpeg);
		HAL_JPEG_ConfigOutputBuffer(hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);	//ustaw następny bufor
	}
	else
	{
		HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_OUTPUT);
		chStatusBufJpeg |= STAT_JPG_ZATRZYMANE_WY;
		printf("PauWy, ");
	}
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

	chErr = KonfigurujKompresjeJpeg(&stKonfJpeg, sSzerokosc, sWysokosc, JPEG_GRAYSCALE_COLORSPACE, JPEG_444_SUBSAMPLING, 80);
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

	chErr = KonfigurujKompresjeJpeg(&stKonfJpeg, sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_444_SUBSAMPLING, 80);
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
// Kompresuj obraz YUV420 z kamery do JPEG
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

	chErr = KonfigurujKompresjeJpeg(&stKonfJpeg, sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_420_SUBSAMPLING, 60);
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
// Kompresuj obraz RGB888 do JPEG w formacie Y8
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
	uint16_t sIloscBlokowPionowo = sWysokosc / 8;
	uint8_t chR, chG, chB;
	uint8_t chY1, chY2;
	int8_t chCb1, chCr1, chCb2, chCr2;
	uint8_t chErr, chDaneDoZapisu;
	uint8_t chLicznikTimeoutu = 0;

	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(&stKonfJpeg, sSzerokosc, sWysokosc, JPEG_GRAYSCALE_COLORSPACE, JPEG_444_SUBSAMPLING, chJakosc);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);

	for (uint16_t by=0; by<sIloscBlokowPionowo; by++)	//pętla po parze wierszy bloków 8x8 na wysokości obrazu
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

			chWskNapBufMcu += 2;	//produkowane są jednocześnie po 2 bloki wejściowe
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
// Kompresuj obraz RGB888 do JPEG w formacie YUV444
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

	chErr = KonfigurujKompresjeJpeg(&stKonfJpeg, sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_444_SUBSAMPLING, chJakosc);
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
// Kompresuj obraz RGB888 do JPEG w formacie YUV422
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

	chErr = KonfigurujKompresjeJpeg(&stKonfJpeg, sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_422_SUBSAMPLING, chJakosc);
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
		for (uint8_t bx=0; bx<chLiczbaParBlokow; bx++)	//pętla po parze bloków na szerokosci obrazu
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
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY1, &chCb, &chCr);

					nOfsetWe += SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
					chR = *(obrazRGB888 + nOfsetWe + 0);		//piksele bloku prawego
					chG = *(obrazRGB888 + nOfsetWe + 1);
					chB = *(obrazRGB888 + nOfsetWe + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY2, NULL, NULL);

					//Formowanie MCU
					nOffsetWyjscia = (chWskNapBufMcu * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU) + x;
					chBuforMCU[nOffsetWyjscia + 0 * ROZMIAR_BLOKU] = chY1;
					chBuforMCU[nOffsetWyjscia + 1 * ROZMIAR_BLOKU] = chY2;
					chBuforMCU[nOffsetWyjscia + 2 * ROZMIAR_BLOKU] = chCb;
					chBuforMCU[nOffsetWyjscia + 3 * ROZMIAR_BLOKU] = chCr;
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
// Kompresuj obraz RGB888 do JPEG w formacie YUV420
// Parametry:
// [we] *obrazRGB888 - wskaźnik na bufor z obrazem wejściowym
// [we] sSzerokosc, sWysokosc - rozmiary obrazu. Muszą być podzielne przez 16
// [we] chJakosc - współczynnik kompresji z zakresu 1..100
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujRGB888doYUV420(uint8_t *obrazRGB888, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakosc)
{
	uint32_t nOffsetWierszaBlokowGor, nOffsetWierszaBlokowDol;	//offset górnego i dolnego wiersza pary bloków
	uint32_t nOffsetWierszaGor, nOffsetWierszaDol, nOffsetBloku;
	uint32_t nOffset;
	uint32_t nOffsetWyjscia;
	uint16_t chLiczbaParBlokowX = sSzerokosc / 16;
	uint16_t chLiczbaParBlokowY = sWysokosc / 16;
	uint8_t chR, chG, chB;
	uint8_t chY1, chY2, chY3, chY4;	//luminancja 4 pikseli
	int8_t chCb, chCr;				//chrominancja niebieska i czerwona
	uint8_t chErr, chDaneDoZapisu;
	uint8_t chLicznikTimeoutu = 0;

	chWskNapBufJpeg = chWskOprBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	sZajetoscBuforaWeJpeg = 0;
	chWskNapBufMcu = 0;
	chStatusBufJpeg |= STAT_JPG_OTWORZ;		//ustaw flagę otwarcia pliku
	chStatusBufJpeg &= ~(STAT_JPG_ZATRZYMANE_WE + STAT_JPG_ZATRZYMANE_WY + STAT_JPG_GOTOWY);	//kasuj flagi, które będą ustawiane w procesie

	chErr = KonfigurujKompresjeJpeg(&stKonfJpeg, sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_420_SUBSAMPLING, chJakosc);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		printf("Blad konf JPEG: %ld\r\n", hjpeg.ErrorCode);
		chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
		return chErr;
	}
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskNapBufJpeg][0], ROZM_BUF_WY_JPEG);

	for (uint16_t by=0; by<chLiczbaParBlokowY; by++)	//pętla po parze wierszy bloków 8x8 na wysokości obrazu
	{
		nOffset = chLiczbaParBlokowX * 2 * LICZBA_KOLOR_RGB888 * ROZMIAR_BLOKU;	//rozmiar wiersza bloków
		nOffsetWierszaBlokowGor = by * 2 * nOffset;								//offset wiersza górnego
		nOffsetWierszaBlokowDol = nOffsetWierszaBlokowGor + nOffset;			//offset wiersza dolnego
		for (uint8_t bx=0; bx<chLiczbaParBlokowX; bx++)	//pętla po parze bloków na szerokosci obrazu
		{
			//rozpocznij formowanie MCU
			nOffsetBloku =  2 * bx * SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
			//nOffsetBloku += nOffsetWierszaBlokow;
			for (uint8_t y=0; y<WYSOKOSC_BLOKU; y++)				//pętla po wierszach
			{
				nOffset = nOffsetBloku + y * sSzerokosc * LICZBA_KOLOR_RGB888;	//offset wiersza w bloku
				nOffsetWierszaGor = nOffsetWierszaBlokowGor + nOffset;
				nOffsetWierszaDol = nOffsetWierszaBlokowDol + nOffset;

				for (uint8_t x=0; x<SZEROKOSC_BLOKU; x++)			//pętla po  kolumnach
				{
					nOffset = nOffsetWierszaGor + LICZBA_KOLOR_RGB888 * x;
					chR = *(obrazRGB888 + nOffset + 0);		//piksele bloku lewego
					chG = *(obrazRGB888 + nOffset + 1);
					chB = *(obrazRGB888 + nOffset + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY1, &chCb, &chCr);

					nOffset += SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
					chR = *(obrazRGB888 + nOffset + 0);		//piksele bloku prawego
					chG = *(obrazRGB888 + nOffset + 1);
					chB = *(obrazRGB888 + nOffset + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY2, NULL, NULL);

					nOffset = nOffsetWierszaDol + LICZBA_KOLOR_RGB888 * x;
					chR = *(obrazRGB888 + nOffset + 0);		//piksele bloku lewego
					chG = *(obrazRGB888 + nOffset + 1);
					chB = *(obrazRGB888 + nOffset + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY3, NULL, NULL);

					nOffset += SZEROKOSC_BLOKU * LICZBA_KOLOR_RGB888;
					chR = *(obrazRGB888 + nOffset + 0);		//piksele bloku prawego
					chG = *(obrazRGB888 + nOffset + 1);
					chB = *(obrazRGB888 + nOffset + 2);
					KonwersjaRGB888doYCbCr(chR, chG, chB, &chY4, NULL, NULL);

					//Formowanie MCU
					nOffsetWyjscia = (chWskNapBufMcu * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU) + x;
					chBuforMCU[nOffsetWyjscia + 0 * ROZMIAR_BLOKU] = chY1;
					chBuforMCU[nOffsetWyjscia + 1 * ROZMIAR_BLOKU] = chY2;
					chBuforMCU[nOffsetWyjscia + 2 * ROZMIAR_BLOKU] = chY3;
					chBuforMCU[nOffsetWyjscia + 3 * ROZMIAR_BLOKU] = chY4;
					chBuforMCU[nOffsetWyjscia + 4 * ROZMIAR_BLOKU] = chCb;
					chBuforMCU[nOffsetWyjscia + 5 * ROZMIAR_BLOKU] = chCr;
				}
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

			//Używam podwójnego buforowania, więc dzielę bufor na 2 części: jedna jest napełniana tutaj, druga opróżniana przez MDMA Jpega
			if ((chWskNapBufMcu == ILOSC_BUF_WE_MCU/2) || (chWskNapBufMcu == ILOSC_BUF_WE_MCU))		//czy mamy już połowę bufora lub cały bufor? Dla Y8 1 blok = 1 MCU
			{
				//Wskaż na źródło danych w pierwszej lub drugiej połowie bufora
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



