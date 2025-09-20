//////////////////////////////////////////////////////////////////////////////
//
// Obsługa sprzętowej kompresji obrazu
// Opracowane na podstawie AN4996 v4
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "jpeg.h"



extern JPEG_HandleTypeDef hjpeg;
extern MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
extern MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM2"))) chBuforJPEG[ROZMIAR_BUF_JPEG] = {0};	//SekcjaSRAM2 ma wyłączony cache w MPU, więc może pracować z MDMA bez konieczności czyszczenia cache
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforJPEG[ROZMIAR_BUF_JPEG] = {0};	//SekcjaZewnSRAM ma wyłączony cache w MPU, więc może pracować z MDMA bez konieczności czyszczenia cache


/*
const uint8_t chTabKwantyzacjiLuminancji[64] = {0x10, 0x0B, 0x0A, 0x10,	0x18, 0x28, 0x33, 0x3D, 0x0C, 0x0C, 0x0E, 0x13, 0x1A, 0x3A, 0x3C, 0x37, 0x0E, 0x0D, 0x10, 0x18, 0x28, 0x39, 0x45, 0x38, 0x0E, 0x11, 0x16, 0x1D, 0x33, 0x57, 0x50, 0x3E,
												0x12, 0x16, 0x25, 0x38, 0x44, 0x6D, 0x67, 0x4D, 0x18, 0x23, 0x37, 0x40, 0x51, 0x68, 0x71, 0x5C, 0x31, 0x40, 0x4E, 0x57, 0x67, 0x79, 0x78, 0x65, 0x48, 0x5C, 0x5F, 0x62, 0x70, 0x64, 0x67, 0x63};

const uint8_t chTabKwantyzacjiChrominancji[64] = {0x11, 0x12, 0x18, 0x2F, 0x63, 0x63, 0x63, 0x63, 0x12, 0x15, 0x1A, 0x42, 0x63, 0x63, 0x63, 0x63, 0x18, 0x1A, 0x38, 0x63, 0x63, 0x63, 0x63, 0x63, 0x2F, 0x42, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
												  0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63};

*/


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
    hmdma_jpeg_outfifo_th.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
    hmdma_jpeg_outfifo_th.Init.SourceBlockAddressOffset = 0;
    hmdma_jpeg_outfifo_th.Init.DestBlockAddressOffset = 0;
    chErr = HAL_MDMA_Init(&hmdma_jpeg_outfifo_th);*/
   	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Konfiguracja kompresji sprzętowej
// Parametry: sSzerokosc, sWysokosc - rozmiary kompresowanego obrazu
// chJakoscKompresji - współczynnik 0..100 okreslający jakość kompresji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chJakoscKompresji)
{
	JPEG_ConfTypeDef pConf;

	pConf.ColorSpace = JPEG_GRAYSCALE_COLORSPACE;
	pConf.ImageWidth = sSzerokosc;
	pConf.ImageHeight = sWysokosc;
	pConf.ImageQuality = chJakoscKompresji;
	return HAL_JPEG_ConfigEncoding(&hjpeg, &pConf);
}



/*
void InicjalizujJpeg(uint16_t sSzerokosc, uint16_t sWysokosc)
{
	//wypełnij tablice kwantyzacji w pamieci QRAM
	for (uint8_t n=0; n<16; n++)
	{
		hjpeg.QMEM0[n] = ((uint32_t)chTabKwantyzacjiLuminancji[4*n+0] << 24) + ((uint32_t)chTabKwantyzacjiLuminancji[4*n+1] << 16) + ((uint32_t)chTabKwantyzacjiLuminancji[4*n+2] << 8) + chTabKwantyzacjiLuminancji[4*n+3];
		//jpeg.QMEM1[n] = ((uint32_t)chTabKwantyzacjiChrominancji[4*n+0] << 24) + ((uint32_t)chTabKwantyzacjiChrominancji[4*n+1] << 16) + ((uint32_t)chTabKwantyzacjiChrominancji[4*n+2] << 8) + chTabKwantyzacjiChrominancji[4*n+3];
	}

	hjpeg.CONFR1 = (sWysokosc << 16)|	//Bits 31:16 YSIZE[15:0]: Y Size; This field defines the number of lines in source image.
				  (1 << 8)|	//Bit 8 HDR: Header processing. This bit enables the header processing (generation/parsing): 0: Disable, 1: Enable (tylko dla dekodowania)
				  (0 << 6)|	//Bits 7:6 NS[1:0]: Number of components for scan; This field defines the number of components minus 1 for scan header marker segment
				  (0 << 4)|	//Bits 5:4 COLSPACE[1:0]: Color space. This filed defines the number of quantization tables minus 1 to insert in the output stream: 00: Grayscale (1 quantization table), 01: YUV (2 quantization tables), 10: RGB (3 quantization tables), 11: CMYK (4 quantization tables)
				  (0 << 3)| //Bit 3 DE: Codec operation as coder or decoder. This bit selects the code or decode process: 0: Code, 1: Decode
				  (0 << 0);	//Bits 1:0 NF[1:0]: Number of color components. This field defines the number of color components minus 1: 00: Grayscale (1 color component), 01: - (2 color components), 10: YUV or RGB (3 color components), 11: CMYK (4 color components)
	hjpeg.CONFR2 = ((sSzerokosc / 8) * (sWysokosc / 8)) - 1;	//Bits 25:0 NMCU[25:0]: Number of MCUs
	hjpeg.CONFR3 = sSzerokosc;	//Bits 31:16 XSIZE[15:0]: X size. This field defines the number of pixels per line.

	//CONFR4 - dla luminancji, CONFR5 dla Cb i CONFR6 dla Cr
	hjpeg.CONFR4 = (1 << 12)|	//Bits 15:12 HSF[3:0]: Horizontal sampling factor. Horizontal sampling factor for component {x-4}.
			      (1 << 8)|	//Bits 11:8 VSF[3:0]: Vertical sampling factor.  Vertical sampling factor for component {x-4}.
				  (0 << 4)|	//Bits 7:4 NB[3:0]: Number of blocks.  Number of data units minus 1 that belong to a particular color in the MCU.
				  (0 << 2)|	//Bits 3:2 QT[1:0]: Quantization table. Selects quantization table used for component {x-4}: 00: Quantization table 0, 01: Quantization table 1, 10: Quantization table 2, 11: Quantization table 3. Wskazuje na QMEM0
				  (0 << 1)|	//Bit 1 HA: Huffman AC. Selects the Huffman table for encoding AC coefficients: 0: Huffman AC table 0, 1: Huffman AC table 1
				  (0 << 0);	//Bit 0 HD: Huffman DC. Selects the Huffman table for encoding DC coefficients: 0: Huffman DC table 0, 1: Huffman DC table 1

	hjpeg.CONFR5 = (1 << 12)|	//Bits 15:12 HSF[3:0]: Horizontal sampling factor. Horizontal sampling factor for component {x-4}.
			      (1 << 8)|	//Bits 11:8 VSF[3:0]: Vertical sampling factor.  Vertical sampling factor for component {x-4}.
				  (1 << 4)|	//Bits 7:4 NB[3:0]: Number of blocks.  Number of data units minus 1 that belong to a particular color in the MCU.
				  (1 << 2)|	//Bits 3:2 QT[1:0]: Quantization table. Selects quantization table used for component {x-4}: 00: Quantization table 0, 01: Quantization table 1, 10: Quantization table 2, 11: Quantization table 3. Wskazuje na QMEM1
				  (1 << 1)|	//Bit 1 HA: Huffman AC. Selects the Huffman table for encoding AC coefficients: 0: Huffman AC table 0, 1: Huffman AC table 1
				  (1 << 0);	//Bit 0 HD: Huffman DC. Selects the Huffman table for encoding DC coefficients: 0: Huffman DC table 0, 1: Huffman DC table 1

	hjpeg.CONFR6 = (1 << 12)|	//Bits 15:12 HSF[3:0]: Horizontal sampling factor. Horizontal sampling factor for component {x-4}.
			      (1 << 8)|	//Bits 11:8 VSF[3:0]: Vertical sampling factor.  Vertical sampling factor for component {x-4}.
				  (1 << 4)|	//Bits 7:4 NB[3:0]: Number of blocks.  Number of data units minus 1 that belong to a particular color in the MCU.
				  (1 << 2)|	//Bits 3:2 QT[1:0]: Quantization table. Selects quantization table used for component {x-4}: 00: Quantization table 0, 01: Quantization table 1, 10: Quantization table 2, 11: Quantization table 3. Wskazuje na QMEM1
				  (1 << 1)|	//Bit 1 HA: Huffman AC. Selects the Huffman table for encoding AC coefficients: 0: Huffman AC table 0, 1: Huffman AC table 1
				  (1 << 0);	//Bit 0 HD: Huffman DC. Selects the Huffman table for encoding DC coefficients: 0: Huffman DC table 0, 1: Huffman DC table 1
}
*/




void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo)
{

}

void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
	//SCB_InvalidateDCache_by_Addr((uint32_t*)g_pOutputBuf, (int32_t)g_outputBufSize)
}

void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{

}

void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{

}

void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{

}

void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{

}









