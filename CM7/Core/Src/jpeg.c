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
#include "rejestrator.h"

extern JPEG_HandleTypeDef hjpeg;
extern MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
extern MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforJpeg[ILOSC_BUF_JPEG][ROZM_BUF_WY_JPEG] = {0};	//SekcjaZewnSRAM ma wyłączony cache w MPU, więc może pracować z MDMA bez konieczności czyszczenia cache
volatile uint8_t chStatusBufJpeg;	//przechowyje bity okreslające status procesu przepływu danych na kartę SD
volatile uint8_t chWskaznikBufJpeg;	// wskazuje do którego bufora obecnie są zapisywane dane
volatile uint8_t chKolejkaBuf[ILOSC_BUF_JPEG];	//przechowuje kolejność zapisu buforów na kartę. Istotne gdy trzba zapisać więcej niż 1 bufor
uint8_t chWskNapKolejki, chWskOprKolejki;	//wskaźniki napełniania i opróżniania kolejki buforów do zapisu
uint32_t nRozmiarObrazuJPEG;	//w bajtach
uint8_t chWynikKompresji;		//flagi ustawiane w callbackach, określajace postęp przepływu danych przez enkoder JPEG
uint8_t chBuforMCU[ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU];  //Bufor danych wejściowych do kompresji
//uint8_t chWskNapBufMcu, chWskOprBufMcu;	//wskaźniki napełniania i opróżniania buforów MCU
uint8_t chWskNapBufMcu;
uint8_t chZatrzymanoWe;
volatile uint8_t chZajetoscBuforaWyJpeg;	//liczba buforów
volatile uint16_t sZajetoscBuforaWeJpeg, sZajetoscBuforaWyJpeg;		//liczba bajtów w buforach
extern uint8_t chStatusRejestratora;	//zestaw flag informujących o stanie rejestratora
const uint8_t chNaglJpegSOI[ROZMIAR_NAGL_JPEG] = {
		    0xFF, 0xD8, 	//SIOI (Start Of Image)
		    0xFF, 0xE0, 	// APPlication0 (JFIF)
			0x00, 0x10, 	// rozmiar: 16
			'J', 'F', 'I', 'F', 0x00, // Identyfikator
			0x01, 0x01,		//wersja 1.1
			0x00,			//jednostka 1 dpi
			0x00, 0x01, 	//gęstość poziomo
			0x00, 0x01,		//gęstość pionowo
			0x00, 0x00};	//thumbnail 0x0
const uint8_t chNaglJpegEOI[ROZMIAR_ZNACZ_xOI] = {0xFF, 0xD9};	//EOI (End Of Image)

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
	chErr = HAL_JPEG_Init(&hjpeg);
	if (chErr)
    	return chErr;

	/*hmdma_jpeg_infifo_th.Instance = MDMA_Channel0;
	hmdma_jpeg_infifo_th.Init.Request = MDMA_REQUEST_JPEG_INFIFO_TH;
	hmdma_jpeg_infifo_th.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
    //hmdma_jpeg_infifo_th.Init.Priority = MDMA_PRIORITY_LOW;
	hmdma_jpeg_infifo_th.Init.Priority = MDMA_PRIORITY_HIGH;
	hmdma_jpeg_infifo_th.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
	hmdma_jpeg_infifo_th.Init.SourceInc = MDMA_SRC_INC_BYTE;
	hmdma_jpeg_infifo_th.Init.DestinationInc = MDMA_DEST_INC_DISABLE;
	hmdma_jpeg_infifo_th.Init.SourceDataSize  = MDMA_SRC_DATASIZE_BYTE;
	hmdma_jpeg_infifo_th.Init.DestDataSize = MDMA_DEST_DATASIZE_WORD;
	hmdma_jpeg_infifo_th.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
	//hmdma_jpeg_infifo_th.Init.BufferTransferLength = 8;		//pojemność FIFO
	hmdma_jpeg_infifo_th.Init.BufferTransferLength = 32;
	hmdma_jpeg_infifo_th.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
	hmdma_jpeg_infifo_th.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
	hmdma_jpeg_infifo_th.Init.SourceBlockAddressOffset = 0;
	hmdma_jpeg_infifo_th.Init.DestBlockAddressOffset = 0;
    chErr = HAL_MDMA_Init(&hmdma_jpeg_infifo_th);
    if (chErr)
    	return chErr;

    hmdma_jpeg_outfifo_th.Instance = MDMA_Channel1;
    hmdma_jpeg_outfifo_th.Init.Request = MDMA_REQUEST_JPEG_OUTFIFO_TH;
    //hmdma_jpeg_outfifo_th.Init.BufferTransferLength = 8;	//pojemność FIFO
    hmdma_jpeg_outfifo_th.Init.BufferTransferLength = 32;
    hmdma_jpeg_outfifo_th.Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER;
    //hmdma_jpeg_outfifo_th.Init.Priority = MDMA_PRIORITY_LOW;
    hmdma_jpeg_outfifo_th.Init.Priority = MDMA_PRIORITY_HIGH;
    hmdma_jpeg_outfifo_th.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
    hmdma_jpeg_outfifo_th.Init.SourceInc = MDMA_SRC_INC_DISABLE;
    hmdma_jpeg_outfifo_th.Init.DestinationInc = MDMA_DEST_INC_BYTE;
    hmdma_jpeg_outfifo_th.Init.SourceDataSize = MDMA_SRC_DATASIZE_WORD;
    hmdma_jpeg_outfifo_th.Init.DestDataSize = MDMA_SRC_DATASIZE_BYTE;
    hmdma_jpeg_outfifo_th.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
    hmdma_jpeg_outfifo_th.Init.BufferTransferLength = 8;	//pojemność FIFO
    hmdma_jpeg_outfifo_th.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
    //hmdma_jpeg_outfifo_th.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_jpeg_outfifo_th.Init.DestBurst = MDMA_DEST_BURST_8BEATS;
    hmdma_jpeg_outfifo_th.Init.SourceBlockAddressOffset = 0;
    hmdma_jpeg_outfifo_th.Init.DestBlockAddressOffset = 0;
    chErr = HAL_MDMA_Init(&hmdma_jpeg_outfifo_th);*/
   	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja kompresji sprzętowej
// Parametry: sSzerokosc, sWysokosc - rozmiary kompresowanego obrazu
// chTypKoloru - stała definiująca przestrzeń kolorów dla sprzętowego enkodera
// chJakoscObrazu - współczynnik z zakresu 0..100 okreslający jakość kompresji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chTypKoloru, uint8_t chTypChrominancji, uint8_t chJakoscObrazu)
{
	JPEG_ConfTypeDef pConf;

	pConf.ColorSpace = chTypKoloru;					//JPEG_GRAYSCALE_COLORSPACE, JPEG_YCBCR_COLORSPACE
	pConf.ChromaSubsampling = chTypChrominancji;	//JPEG_444_SUBSAMPLING, JPEG_422_SUBSAMPLING, JPEG_420_SUBSAMPLING
	pConf.ImageWidth = sSzerokosc;
	pConf.ImageHeight = sWysokosc;
	pConf.ImageQuality = chJakoscObrazu;
	return HAL_JPEG_ConfigEncoding(&hjpeg, &pConf);
}



////////////////////////////////////////////////////////////////////////////////
// Callback HAL_JPEG_InfoReadyCallback is asserted if the current operation
// is a JPEG decoding to provide the application with JPEG image  parameters.
// This callback is asserted when the JPEG peripheral successfully parse the JPEG header.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo)
{

}



////////////////////////////////////////////////////////////////////////////////
// Callback HAL_JPEG_EncodeCpltCallback is asserted when the HAL JPEG driver has ended the current JPEG encoding operation,
// and all output data has been transmitted to the application.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
	chWynikKompresji |= KOMPR_OBR_GOTOWY;		//podany obraz właśnie został skompresowany
}



////////////////////////////////////////////////////////////////////////////////
// Callback HAL_JPEG_DecodeCpltCallback is asserted when the HAL JPEG driver has ended the
// current JPEG decoding operation. and all output data has been transmitted to the application.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{

}



////////////////////////////////////////////////////////////////////////////////
// Callback HAL_JPEG_ErrorCallback is asserted when an error occurred during the current operation.
// the application can call the function HAL_JPEG_GetError()  to retrieve the error codes.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
	printf("JPEG_Error: %ld", HAL_JPEG_GetError(hjpeg));
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

	if (sZajetoscBuforaWeJpeg == 0)	//jeżeli kompresor zużył wszystkie dane jakie były przygotowane
	{
		HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);
		//printf("Pau, ");
		chZatrzymanoWe = 1;
	}
	else	//w buforze wejsciowym są jeszcze dane, wiec kompresuj je
	{
		HAL_JPEG_ConfigInputBuffer(hjpeg, chBuforMCU + NbDecodedData, sZajetoscBuforaWeJpeg);
		printf("Kont ");
	}

	chWynikKompresji |= KOMPR_PUSTE_WE;		//na wejście enkodera można podać nowe dane
}



////////////////////////////////////////////////////////////////////////////////
//skompresowane dane sa gotowe
// Callback HAL_JPEG_DataReadyCallback is asserted when the HAL JPEG driver has filled the given output buffer with the given size.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
	//wstaw bufor danych do kolejki zapisu
	chKolejkaBuf[chWskNapKolejki] = chWskaznikBufJpeg;
	chWskNapKolejki++;
	chWskNapKolejki &= MASKA_LICZBY_BUF;
	chZajetoscBuforaWyJpeg++;

	//ustaw status
	//chStatusBufJpeg |= (1 << chWskaznikBufJpeg);	//ustaw bit zapełnienie bieżącego bufora danych JPEG. Strona odbiorczas wyczyści go gdy rozładuje dane
	chStatusBufJpeg |= STAT_JPG_PELEN_BUF;
	chWskaznikBufJpeg++;							//przełacz bufor skompresowanych danych na następny
	chWskaznikBufJpeg &= MASKA_LICZBY_BUF;

	HAL_JPEG_ConfigOutputBuffer(hjpeg, &chBuforJpeg[chWskaznikBufJpeg][0], ROZM_BUF_WY_JPEG);	//ustaw następny bufor
	nRozmiarObrazuJPEG += OutDataLength;
	chWynikKompresji |= KOMPR_MCU_GOTOWY;	//są gotowe skompresowane dane MCU
	printf("Rozm: %ld wsk: %d wnk: %d buf: %d\r\n",  nRozmiarObrazuJPEG, chWskaznikBufJpeg, chWskNapKolejki, chZajetoscBuforaWyJpeg);
}



////////////////////////////////////////////////////////////////////////////////
// Czeka z timeoutem aż JPEG skompresuje cały obraz
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzekajNaKoniecPracyJPEG(void)
{
	uint8_t chLicznikTimeoutu = 100;

	while (((chWynikKompresji & KOMPR_OBR_GOTOWY) != KOMPR_OBR_GOTOWY) && chLicznikTimeoutu)
	{
		osDelay(1);		//przełącz na inny wątek
		chLicznikTimeoutu--;
	}

	if (chLicznikTimeoutu)
		return BLAD_OK;
	else
		return BLAD_TIMEOUT;
}



////////////////////////////////////////////////////////////////////////////////
// Przygotuj pojedynczy MCU Y8 czyli blok 8x8 dla sprzętowego kompresora pracującego w formacie JPEG_GRAYSCALE_COLORSPACE
// Format Y8 zawiera same bloki luminancji, przez co można na nim uzyskać najwyższy wspóczynnik kompresji
// Skompresowane dane są pakowane do bufora aby wątek rejestratora mógł zapisać je do pliku
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
	uint8_t chLicznikTimeoutu;


	for (uint8_t m=0; m<MASKA_LICZBY_BUF; m++)
	{
		for (uint16_t n=0; n<ROZM_BUF_WY_JPEG; n++)
			chBuforJpeg[m][n] = 0;
	}
	chWskaznikBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	chZatrzymanoWe = 0;
	//ustaw pierwszy bufor z miejscem na nagłówek. Kolejne bufory będą ustawiane w HAL_JPEG_DataReadyCallback
	//HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskaznikBufJpeg][ROZMIAR_NAGL_JPEG - ROZMIAR_ZNACZ_xOI], ROZMIAR_BUF_JPEG - ROZMIAR_NAGL_JPEG);	//nie działa - pisze od początku bufora i wypełnia cały bufor
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskaznikBufJpeg][0], ROZM_BUF_WY_JPEG);

	chStatusBufJpeg = STAT_JPG_OTWORZ | STAT_JPG_NAGLOWEK;	//otwórz plik a gdy bądą pierwsze dane to zapisz nagłówek
	chWynikKompresji = KOMPR_PUSTE_WE;	//proces startuje z flagą gotowości do przyjęcia danych
	chWskNapKolejki = chWskOprKolejki = 0;

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_GRAYSCALE_COLORSPACE, JPEG_444_SUBSAMPLING, 80);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		return chErr;
	}

	//rozpocznij formowanie MCU
	for (uint16_t my=0; my<chLiczbaMCU_Wys; my++)	//pętla pracująca na MCU wysokości obrazu
	{
		nOffsetMCU_wzgObrazu = my * chLiczbaMCU_Szer * ROZMIAR_BLOKU;
		for (uint16_t mx=0; mx<chLiczbaMCU_Szer; mx++)	//pętla pracująca na MCU szerokości obrazu
		{

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

			/*/jeżeli kolejka buforów skompresowanych danych jest bliska zapełnienia, to wstrzymaj kompresor aby nie utracić danych
			chLicznikTimeoutu = 100;
			while (chZajetoscBuforaWyJpeg > ILOSC_BUF_JPEG - 2)
			{
				osDelay(1);
				chLicznikTimeoutu--;
				if (!chLicznikTimeoutu)
				{
					printf("Timeout wy: buf=%d/%d\n\r", chZajetoscBuforaWyJpeg, ILOSC_BUF_JPEG);
					chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;
					return BLAD_TIMEOUT;
				}
			}*/

			//jeżeli bufor wejściowy jest zapełnione więcej niż 3/4, to wstrzymuj wątek aby kompresor zdażył przetrawić dane do kompresji
			if (sZajetoscBuforaWeJpeg > (ILOSC_BUF_WE_MCU * ROZMIAR_BLOKU * 3 / 4 ))
			{
				printf("Del, ");
				osDelay(5);
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

				chWynikKompresji &= ~KOMPR_PUSTE_WE;	//kasuj flagę pustego enkodera
				if (hjpeg.State == HAL_JPEG_STATE_READY)	//tylko dla pierwszego uruchomienia
				{
					chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, ILOSC_BUF_WE_MCU / 2 * ROZMIAR_BLOKU, chBuforJpeg[chWskaznikBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
				}
				else
				{
					if (chZatrzymanoWe)
					{
						chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję
						chZatrzymanoWe = 0;
						//printf("Wzn, ");
					}
				}
			}

			/*chLicznikTimeoutu = 100;
			do	//kompresja jednego MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie
			{
				chWynikKompresji &= ~KOMPR_PUSTE_WE;	//kasuj flagę pustego enkodera
				if (hjpeg.State == HAL_JPEG_STATE_READY)
				{
					HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, ROZMIAR_BLOKU);	//przekaż bufor z nowymi danymi
					chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, ROZMIAR_BLOKU, chBuforJpeg[chWskaznikBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
				}
				else
				{
					if (chZatrzymanoWe)
					{
						chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję
						chZatrzymanoWe = 0;
						printf("Wzn, ");
					}
				}

				//jeżeli bufory wejściowe są zapełnione do połowy, to wstrzymaj wątek aby kompresor zdażył przetrawić dane do kompresji
				if (chZajetoscBuforaWeJpeg > ILOSC_BUF_MCU / 2)
					osDelay(1);

				chLicznikTimeoutu--;
				if (!chLicznikTimeoutu)
				{
					printf("Timeout we: buf=%d/%d\n\r", chZajetoscBuforaWeJpeg, ILOSC_BUF_MCU);
					chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;
					return BLAD_TIMEOUT;
				}
			} while(chZajetoscBuforaWeJpeg == ILOSC_BUF_MCU); */
		}
	}

	//czekaj aż ostatnie MCU zostanie skompresowane
	chLicznikTimeoutu = 100;
	while((chWynikKompresji & KOMPR_OBR_GOTOWY) == 0)	//czekaj aż HAL_JPEG_EncodeCpltCallback ustawi flagę końca kompresji i zaktualizuje rozmiar danych
	{
		osDelay(1);
		chLicznikTimeoutu--;
		if (!chLicznikTimeoutu)
			return BLAD_TIMEOUT;
	}
	chErr = CzekajNaKoniecPracyJPEG();
	chStatusBufJpeg |= STAT_JPG_ZAMKNIJ;	//ustaw polecenia zamknięcia pliku
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
	uint8_t chLicznikTimeoutu;

	for (uint8_t m=0; m<MASKA_LICZBY_BUF; m++)
	{
		for (uint16_t n=0; n<ROZM_BUF_WY_JPEG; n++)
			chBuforJpeg[m][n] = 0;
	}
	chWskaznikBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskaznikBufJpeg][0], ROZM_BUF_WY_JPEG);

	chStatusBufJpeg = STAT_JPG_OTWORZ | STAT_JPG_NAGLOWEK;	//otwórz plik a gdy bądą pierwsze dane to zapisz nagłówek
	chWynikKompresji = KOMPR_PUSTE_WE;	//proces startuje z flagą gotowości do przyjęcia danych
	chWskNapKolejki = chWskOprKolejki = 0;

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_444_SUBSAMPLING, 80);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		return chErr;
	}

	//rozpocznij formowanie MCU składajacego sie z 3 bloków 8x8: Y, Cr, i Cb z danych ułożonych sekwencyjnie: YUVYUV (U=Cb, V=Cr)
	for (uint16_t my=0; my<chLiczbaMCU_Wys; my++)	//pętla pracująca na MCU wysokości obrazu
	{
		nOffsetMCU_wzgObrazu = my * chLiczbaMCU_Szer * 3 * ROZMIAR_BLOKU;
		for (uint16_t mx=0; mx<chLiczbaMCU_Szer; mx++)	//pętla pracująca na MCU szerokości obrazu
		{
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

			/*/wyrzucam na debug
			printf("MCU %d\r\n", mx + my*chLiczbaMCU_Szer);
			for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)
			{
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					printf("%02X ", chBuforMCU[x + (y*SZEROKOSC_BLOKU) + 0 * ROZMIAR_BLOKU]);
				printf("\t");
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					printf("%02X ", chBuforMCU[x + (y*SZEROKOSC_BLOKU) + 1 * ROZMIAR_BLOKU]);
				printf("\t");
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					printf("%02X ", chBuforMCU[x + (y*SZEROKOSC_BLOKU) + 2 * ROZMIAR_BLOKU]);

				printf("\r\n");
				osDelay(1);		//czas na wyjscie printfa(1);
			}*/

			chLicznikTimeoutu = 100;
			while((chWynikKompresji & KOMPR_PUSTE_WE) == 0)	//czekaj aż callback ustawi flagę gotowości na przyjęcie danych
			{
				osDelay(1);
				chLicznikTimeoutu--;
				if (!chLicznikTimeoutu)
				{
					printf("Timeout1: mx=%d my=%d\n\r", mx, my);
					return BLAD_TIMEOUT;
				}
			}

			//jeżeli kolejka buforów jest bliska zapełnienia to wstrzymaj kompresor aby nie utracić danych
			if (chWskOprKolejki != chWskNapKolejki)
			{
				uint8_t chZajetoscBufora;
				if (chWskNapKolejki > chWskOprKolejki)
					chZajetoscBufora = chWskNapKolejki - chWskOprKolejki;
				else
					chZajetoscBufora = ILOSC_BUF_JPEG + chWskNapKolejki - chWskOprKolejki;

				chLicznikTimeoutu = 100;
				while (chZajetoscBufora > ILOSC_BUF_JPEG - 2)
				{
					osDelay(1);
					chLicznikTimeoutu--;
					if (!chLicznikTimeoutu)
					{
						printf("Timeout zajetosci bufora: %d/%d\n\r", chZajetoscBufora, ILOSC_BUF_JPEG);
						return BLAD_TIMEOUT;
					}
				}
			}

			//kompresja jednego MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie, niech się kompresuje na bieżąco
			chWynikKompresji &= ~KOMPR_PUSTE_WE;	//kasuj flagę pustego enkodera
			if (hjpeg.State == HAL_JPEG_STATE_READY)
			{
				HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, 3 * ROZMIAR_BLOKU);	//przekaż bufor z nowymi danymi
				chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, 3 * ROZMIAR_BLOKU, chBuforJpeg[chWskaznikBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
			}
			else
				chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję

			if (chStatusBufJpeg & STAT_JPG_OTWORZ)
				osDelay(5);	//daj czas na otwarcie pliku
		}
	}

	//czekaj aż ostatnie MCU zostanie skompresowane
	chLicznikTimeoutu = 100;
	while((chWynikKompresji & KOMPR_OBR_GOTOWY) == 0)	//czekaj aż HAL_JPEG_EncodeCpltCallback ustawi flagę końca kompresji i zaktualizuje rozmiar danych
	{
		osDelay(1);
		chLicznikTimeoutu--;
		if (!chLicznikTimeoutu)
			return BLAD_TIMEOUT;
	}
	chErr = CzekajNaKoniecPracyJPEG();
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
	uint8_t chLicznikTimeoutu;

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_YCBCR_COLORSPACE, JPEG_420_SUBSAMPLING, 60);
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		return chErr;
	}

	//for (uint32_t n=0; n<ROZMIAR_BUF_JPEG; n++)
		//chBuforJpeg[n] = 0;

	//wypełnij oba bloki chrominancji neutralną wartoscią, niezmienną dla całego obrazu
	for (uint16_t n=(4 * ROZMIAR_BLOKU); n<(6 * ROZMIAR_BLOKU); n++)
		chBuforMCU[n] = 128;

	chWskaznikBufJpeg = 0;
	nRozmiarObrazuJPEG = 0;
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, &chBuforJpeg[chWskaznikBufJpeg][0], ROZM_BUF_WY_JPEG);

	chStatusBufJpeg = STAT_JPG_OTWORZ | STAT_JPG_NAGLOWEK;	//otwórz plik a gdy bądą pierwsze dane to zapisz nagłówek
	chWynikKompresji = KOMPR_PUSTE_WE;	//proces startuje z flagą gotowości do przyjęcia danych
	chWskNapKolejki = chWskOprKolejki = 0;

	//rozpocznij formowanie MCU
	for (uint16_t my=0; my<chLiczbaMCU_Wys; my++)	//pętla pracująca na wysokości obrazu
	{
		nOffsetMCU_wzgObrazu = my * chLiczbaMCU_Szer * 4 * ROZMIAR_BLOKU;
		for (uint16_t mx=0; mx<chLiczbaMCU_Szer; mx++)	//pętla pracująca na szerokości obrazu
		{
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
			//teraz 2 bloki chrominancji, które na razie pomijam

			/*/wyrzucam na debug
			printf("MCU %d\r\n", mx + my*chLiczbaMCU_Szer);
			for (uint16_t n=0; n<4; n++)
			{
				for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)
				{
					for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					{
						printf("%02X ", chBuforMCU[x + (y*SZEROKOSC_BLOKU) + (n*ROZMIAR_BLOKU)]);
					}
					printf("\r\n");
					osDelay(1);		//czas na wyjscie printfa(1);
				}
				printf("\r\n");
			}*/

			chLicznikTimeoutu = 100;
			while((chWynikKompresji & KOMPR_PUSTE_WE) == 0)	//czekaj aż callback ustawi flagę gotowości na przyjęcie danych
			{
				osDelay(1);
				chLicznikTimeoutu--;
				if (!chLicznikTimeoutu)
				{
					printf("Timeout1: mx=%d my=%d\n\r", mx, my);
					return BLAD_TIMEOUT;
				}
			}

			//jeżeli kolejka buforów jest bliska zapełnienia to wstrzymaj kompresor aby nie utracić danych
			if (chWskOprKolejki != chWskNapKolejki)
			{
				uint8_t chZajetoscBufora;
				if (chWskNapKolejki > chWskOprKolejki)
					chZajetoscBufora = chWskNapKolejki - chWskOprKolejki;
				else
					chZajetoscBufora = ILOSC_BUF_JPEG + chWskNapKolejki - chWskOprKolejki;

				chLicznikTimeoutu = 100;
				while (chZajetoscBufora > ILOSC_BUF_JPEG - 2)
				{
					osDelay(1);
					chLicznikTimeoutu--;
					if (!chLicznikTimeoutu)
					{
						printf("Timeout zajetosci bufora: %d/%d\n\r", chZajetoscBufora, ILOSC_BUF_JPEG);
						return BLAD_TIMEOUT;
					}
				}
			}

			//kompresja jednego MCU na bieżąco aby nie przechowywać danych i nie czekać później na zakończenie, niech się kompresuje na bieżąco
			chWynikKompresji &= ~KOMPR_PUSTE_WE;	//kasuj flagę pustego enkodera
			if (hjpeg.State == HAL_JPEG_STATE_READY)
			{
				HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, 6 * ROZMIAR_BLOKU);	//przekaż bufor z nowymi danymi
				chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, 6 * ROZMIAR_BLOKU, chBuforJpeg[chWskaznikBufJpeg], ROZM_BUF_WY_JPEG);	//rozpocznij kompresję
			}
			else
				chErr = HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję

			if (chStatusBufJpeg & STAT_JPG_OTWORZ)
				osDelay(5);	//daj czas na otwarcie pliku
		}
	}

	//czekaj aż ostatnie MCU zostanie skompresowane
	chLicznikTimeoutu = 100;
	while((chWynikKompresji & KOMPR_OBR_GOTOWY) == 0)	//czekaj aż HAL_JPEG_EncodeCpltCallback ustawi flagę końca kompresji i zaktualizuje rozmiar danych
	{
		osDelay(1);
		chLicznikTimeoutu--;
		if (!chLicznikTimeoutu)
			return BLAD_TIMEOUT;
	}
	chErr = CzekajNaKoniecPracyJPEG();
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
