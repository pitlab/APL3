//////////////////////////////////////////////////////////////////////////////
//
// Obsługa sprzętowej kompresji obrazu
// Opracowane na podstawie AN4996 v4
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "jpeg.h"


JPEG_TypeDef jpeg;
const uint8_t chTabKwantyzacjiLuminancji[64] = {0x10, 0x0B, 0x0A, 0x10,	0x18, 0x28, 0x33, 0x3D, 0x0C, 0x0C, 0x0E, 0x13, 0x1A, 0x3A, 0x3C, 0x37, 0x0E, 0x0D, 0x10, 0x18, 0x28, 0x39, 0x45, 0x38, 0x0E, 0x11, 0x16, 0x1D, 0x33, 0x57, 0x50, 0x3E,
												0x12, 0x16, 0x25, 0x38, 0x44, 0x6D, 0x67, 0x4D, 0x18, 0x23, 0x37, 0x40, 0x51, 0x68, 0x71, 0x5C, 0x31, 0x40, 0x4E, 0x57, 0x67, 0x79, 0x78, 0x65, 0x48, 0x5C, 0x5F, 0x62, 0x70, 0x64, 0x67, 0x63};

const uint8_t chTabKwantyzacjiChrominancji[64] = {0x11, 0x12, 0x18, 0x2F, 0x63, 0x63, 0x63, 0x63, 0x12, 0x15, 0x1A, 0x42, 0x63, 0x63, 0x63, 0x63, 0x18, 0x1A, 0x38, 0x63, 0x63, 0x63, 0x63, 0x63, 0x2F, 0x42, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
												  0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63};





////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja kompresji sprzętowej
// Parametry: sSzerokosc, sWysokosc - rozmiar kompresowanego obrazu w pikselach
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjalizujJpeg(uint16_t sSzerokosc, uint16_t sWysokosc)
{
	//wypełnij tablice kwantyzacji w pamieci QRAM
	for (uint8_t n=0; n<16; n++)
	{
		jpeg.QMEM0[n] = ((uint32_t)chTabKwantyzacjiLuminancji[4*n+0] << 24) + ((uint32_t)chTabKwantyzacjiLuminancji[4*n+1] << 16) + ((uint32_t)chTabKwantyzacjiLuminancji[4*n+2] << 8) + chTabKwantyzacjiLuminancji[4*n+3];
		//jpeg.QMEM1[n] = ((uint32_t)chTabKwantyzacjiChrominancji[4*n+0] << 24) + ((uint32_t)chTabKwantyzacjiChrominancji[4*n+1] << 16) + ((uint32_t)chTabKwantyzacjiChrominancji[4*n+2] << 8) + chTabKwantyzacjiChrominancji[4*n+3];
	}

	jpeg.CONFR1 = (sWysokosc << 16)|	//Bits 31:16 YSIZE[15:0]: Y Size; This field defines the number of lines in source image.
				  (1 << 8)|	//Bit 8 HDR: Header processing. This bit enables the header processing (generation/parsing): 0: Disable, 1: Enable (tylko dla dekodowania)
				  (0 << 6)|	//Bits 7:6 NS[1:0]: Number of components for scan; This field defines the number of components minus 1 for scan header marker segment
				  (0 << 4)|	//Bits 5:4 COLSPACE[1:0]: Color space. This filed defines the number of quantization tables minus 1 to insert in the output stream: 00: Grayscale (1 quantization table), 01: YUV (2 quantization tables), 10: RGB (3 quantization tables), 11: CMYK (4 quantization tables)
				  (0 << 3)| //Bit 3 DE: Codec operation as coder or decoder. This bit selects the code or decode process: 0: Code, 1: Decode
				  (0 << 0);	//Bits 1:0 NF[1:0]: Number of color components. This field defines the number of color components minus 1: 00: Grayscale (1 color component), 01: - (2 color components), 10: YUV or RGB (3 color components), 11: CMYK (4 color components)
	jpeg.CONFR2 = ((sSzerokosc / 8) * (sWysokosc / 8)) - 1;	//Bits 25:0 NMCU[25:0]: Number of MCUs
	jpeg.CONFR3 = sSzerokosc;	//Bits 31:16 XSIZE[15:0]: X size. This field defines the number of pixels per line.

	//CONFR4 - dla luminancji, CONFR5 dla Cb i CONFR6 dla Cr
	jpeg.CONFR4 = (1 << 12)|	//Bits 15:12 HSF[3:0]: Horizontal sampling factor. Horizontal sampling factor for component {x-4}.
			      (1 << 8)|	//Bits 11:8 VSF[3:0]: Vertical sampling factor.  Vertical sampling factor for component {x-4}.
				  (0 << 4)|	//Bits 7:4 NB[3:0]: Number of blocks.  Number of data units minus 1 that belong to a particular color in the MCU.
				  (0 << 2)|	//Bits 3:2 QT[1:0]: Quantization table. Selects quantization table used for component {x-4}: 00: Quantization table 0, 01: Quantization table 1, 10: Quantization table 2, 11: Quantization table 3. Wskazuje na QMEM0
				  (0 << 1)|	//Bit 1 HA: Huffman AC. Selects the Huffman table for encoding AC coefficients: 0: Huffman AC table 0, 1: Huffman AC table 1
				  (0 << 0);	//Bit 0 HD: Huffman DC. Selects the Huffman table for encoding DC coefficients: 0: Huffman DC table 0, 1: Huffman DC table 1

	/*jpeg.CONFR5 = (1 << 12)|	//Bits 15:12 HSF[3:0]: Horizontal sampling factor. Horizontal sampling factor for component {x-4}.
			      (1 << 8)|	//Bits 11:8 VSF[3:0]: Vertical sampling factor.  Vertical sampling factor for component {x-4}.
				  (1 << 4)|	//Bits 7:4 NB[3:0]: Number of blocks.  Number of data units minus 1 that belong to a particular color in the MCU.
				  (1 << 2)|	//Bits 3:2 QT[1:0]: Quantization table. Selects quantization table used for component {x-4}: 00: Quantization table 0, 01: Quantization table 1, 10: Quantization table 2, 11: Quantization table 3. Wskazuje na QMEM1
				  (1 << 1)|	//Bit 1 HA: Huffman AC. Selects the Huffman table for encoding AC coefficients: 0: Huffman AC table 0, 1: Huffman AC table 1
				  (1 << 0);	//Bit 0 HD: Huffman DC. Selects the Huffman table for encoding DC coefficients: 0: Huffman DC table 0, 1: Huffman DC table 1

	jpeg.CONFR6 = (1 << 12)|	//Bits 15:12 HSF[3:0]: Horizontal sampling factor. Horizontal sampling factor for component {x-4}.
			      (1 << 8)|	//Bits 11:8 VSF[3:0]: Vertical sampling factor.  Vertical sampling factor for component {x-4}.
				  (1 << 4)|	//Bits 7:4 NB[3:0]: Number of blocks.  Number of data units minus 1 that belong to a particular color in the MCU.
				  (1 << 2)|	//Bits 3:2 QT[1:0]: Quantization table. Selects quantization table used for component {x-4}: 00: Quantization table 0, 01: Quantization table 1, 10: Quantization table 2, 11: Quantization table 3. Wskazuje na QMEM1
				  (1 << 1)|	//Bit 1 HA: Huffman AC. Selects the Huffman table for encoding AC coefficients: 0: Huffman AC table 0, 1: Huffman AC table 1
				  (1 << 0);	//Bit 0 HD: Huffman DC. Selects the Huffman table for encoding DC coefficients: 0: Huffman DC table 0, 1: Huffman DC table 1 */
}

