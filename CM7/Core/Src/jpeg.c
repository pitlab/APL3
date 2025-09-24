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


extern JPEG_HandleTypeDef hjpeg;
extern MDMA_HandleTypeDef hmdma_jpeg_outfifo_th;
extern MDMA_HandleTypeDef hmdma_jpeg_infifo_th;
//uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaSRAM2"))) chBuforJPEG[ROZMIAR_BUF_JPEG] = {0};	//SekcjaSRAM2 ma wyłączony cache w MPU, więc może pracować z MDMA bez konieczności czyszczenia cache
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforJPEG[ROZMIAR_BUF_JPEG] = {0};	//SekcjaZewnSRAM ma wyłączony cache w MPU, więc może pracować z MDMA bez konieczności czyszczenia cache
uint8_t chBlok[ROZMIAR_BLOKU];
uint32_t nRozmiarObrazuJPEG;	//w bajtach
uint8_t chObrazSkompresowany;

// Bufor wyjściowy dla jednego MCU
uint8_t chBuforMCU[6 * ROZMIAR_BLOKU];  // [Y0][Y1][Y2][Y3][Cb][Cr]



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
// chJakoscKompresji - współczynnik 0..100 okreslający jakość kompresji
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
uint8_t KonfigurujKompresjeJpeg(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chKolor, uint8_t chJakoscKompresji)
{
	JPEG_ConfTypeDef pConf;

	pConf.ColorSpace = chKolor;	//JPEG_GRAYSCALE_COLORSPACE, JPEG_YCBCR_COLORSPACE
	pConf.ChromaSubsampling = JPEG_420_SUBSAMPLING;		//nie ma znaczanie dla JPEG_GRAYSCALE_COLORSPACE
	pConf.ImageWidth = sSzerokosc;
	pConf.ImageHeight = sWysokosc;
	pConf.ImageQuality = chJakoscKompresji;
	return HAL_JPEG_ConfigEncoding(&hjpeg, &pConf);
}


//callbacki
void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo)
{

}

void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{
	nRozmiarObrazuJPEG = hjpeg->JpegOutCount;
	chObrazSkompresowany = 1;
}

void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg)
{

}

void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg)
{
	nRozmiarObrazuJPEG = 0;
}

void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData)
{

}

void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{

}


////////////////////////////////////////////////////////////////////////////////
// Czeka z timeoutem aż JPEG skompresuje cały obraz
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzekajNaKoniecPracyJPEG(void)
{
	uint8_t chLicznikTimeoutu = 100;
	do
	{
		osDelay(1);		//przełącz na inny wątek
		chLicznikTimeoutu--;
	}
	while ((!chObrazSkompresowany) && chLicznikTimeoutu);

	if (chLicznikTimeoutu)
		return BLAD_OK;
	else
		return BLAD_TIMEOUT;
}



////////////////////////////////////////////////////////////////////////////////
// Przygotuj pojedynczy MCU czyli zestaw bloków 8x8 dla sprzętowego kompresora w formacie YUV420: Y, Y, Y, Y, Cb, Cr
// Do kompresji wybieram format YUV420 jako najbardziej efektywny. Minimalna jednostka MCU zawiera 4 sąsiadująceze sobą bloki luminancji i 2 bloki chrominancji:
// niebieski Cb i czerwony Cr. Każdy z bloków luminancji jest fragmentem obrazu Y8 (czarnio-białego) 8x8 pikseli a bloko chromiannacji zawierają stałą = 128.
// W MCU znjdują się 4 sąsiadujace ze sobą bloki chrominancji, czyli obszar (2x2) x (8x8). Odpowiada obszarowi 16x16 ale nie liniowo tylko w kratkę.
// Kolejne MCU są brane aż do końca wiersza MCU, potem kolejny wiersz MCU i tak aż do konca obrazu.
// Taki zestaw danych (chBlokWyj) podawany jest na kompresor sprzetowy
// Parametry: *chObrazY8 - wskaźnik na wejsciowy obraz Y8 czarno-biały
//  *chBlokWyj - wskaźnik na zestaw danych idący do kompresora skonfigurowanego na YUV420
// sSzerokosc, sWysokosc - rozmiary obrazu podzielne przez 16
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint8_t *chBlokWyj, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nOffsetMCU_wzgObrazu;	//przesuniecie bloku wzgledem początku obrazu
	uint32_t nOffsetBloku_wzgMCU;	//przesuniecie bloku względem początku MCU
	//uint32_t nOffset_wzgBloku;		//przesuniecie danych wzgledem początku bloku
	uint8_t chLiczbaMCU_Szer = sSzerokosc / (2 * SZEROKOSC_BLOKU);	//liczba MCU luminancji (czwórek bloków) po szerokosci obrazu
	uint8_t chLiczbaMCU_Wys = sWysokosc / (2 * WYSOKOSC_BLOKU);	//liczba MCU luminancji (czwórek bloków) po wysokosci obrazu

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_GRAYSCALE_COLORSPACE, 75);	//JPEG_YCBCR_COLORSPACE,  JPEG_GRAYSCALE_COLORSPACE
	if (chErr)
		return chErr;

	for (uint32_t n=0; n<ROZMIAR_BUF_JPEG; n++)
		chBuforJPEG[n] = 0;

	//wypełnij oba bloki chrominancji neutralną wartoscią, niezmienną dla całego obrazu
	for (uint16_t n=(4 * ROZMIAR_BLOKU); n<(6 * ROZMIAR_BLOKU); n++)
		chBuforMCU[n] = 128;


	for (uint16_t my=0; my<chLiczbaMCU_Wys; my++)	//pętla pracująca na wysokości obrazu
	{
		for (uint16_t mx=0; mx<chLiczbaMCU_Szer; mx++)	//pętla pracująca na szerokości obrazu
		{
			//tutaj obrókba MCU
			nOffsetMCU_wzgObrazu = (mx * 4 * ROZMIAR_BLOKU) + (my * chLiczbaMCU_Szer * 4 * ROZMIAR_BLOKU);
			for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)	//pętla pracująca na wysokosci bloków wewnątrz całego MCU
			{
				//nOffset_wzgBloku = y * 4 * SZEROKOSC_BLOKU;

				//pierwszy blok luminancji Y
				nOffsetBloku_wzgMCU = (0 * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU);
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + nOffsetBloku_wzgMCU] = *(chObrazWe + x + nOffsetBloku_wzgMCU + nOffsetMCU_wzgObrazu);

				//następny z prawej blok luminancji Y
				nOffsetBloku_wzgMCU = (1 * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU);
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + nOffsetBloku_wzgMCU] = *(chObrazWe + x + nOffsetBloku_wzgMCU + nOffsetMCU_wzgObrazu);

				//trzeci blok luminancji Y poniżej pierwszego, przesuniety o 2 bloki i liczbę MCU w szerokości obrazu
				nOffsetBloku_wzgMCU =  (2 * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU);
				//nOffsetBloku = (2 * ROZMIAR_BLOKU) + nOffsetBloku_wzgObrazu;
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + nOffsetBloku_wzgMCU] = *(chObrazWe + x + nOffsetBloku_wzgMCU + nOffsetMCU_wzgObrazu);

				//czwarty blok luminancji Y
				nOffsetBloku_wzgMCU =  3 * ROZMIAR_BLOKU;
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + nOffsetBloku_wzgMCU] = *(chObrazWe + x + nOffsetBloku_wzgMCU + nOffsetMCU_wzgObrazu);
			}
			//teraz 2 bloki chrominancji


			//wyrzucam na debug
			printf("moje MCU\r\n");
			for (uint16_t n=0; n<4; n++)
			{
				for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)
				{
					for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					{
						printf("%2X ", chBuforMCU[x + (y*SZEROKOSC_BLOKU) + (n*ROZMIAR_BLOKU)]);
					}
					printf("\r\n");
					osDelay(1);		//czas na wyjscie printfa(1);
				}
				printf("\r\n");
			}

			//kompresja jednego MCU
			if (hjpeg.State == HAL_JPEG_STATE_READY)
			{
				//chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBlokWyj, 6 * ROZMIAR_BLOKU, chBuforJPEG, ROZMIAR_BUF_JPEG);
				chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, 4 * ROZMIAR_BLOKU, chBuforJPEG, ROZMIAR_BUF_JPEG);	//dla Y8 jest mniej bloków
			}
			else
				nRozmiarObrazuJPEG = 0;
		}
	}



	uint8_t chCroma[ROZMIAR_BLOKU] = {128};

	//prepareMCU(0, 0, uint8_t *Yplane, uint8_t *Cbplane, uint8_t *Crplane, chBlokWyj);
	//prepareMCU(mx, my, Yplane, Cbplane, Crplane, chBuforMCU);
	prepareMCU(0, 0, chObrazWe, chCroma, chCroma, chBuforMCU);

	printf("sztuczne MCU\r\n");
	for (uint16_t n=0; n<4; n++)
	{
		for (uint16_t y=0; y<WYSOKOSC_BLOKU; y++)
		{
			for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
			{
				printf("%2X ", chBuforMCU[x + (y*SZEROKOSC_BLOKU) + (n*ROZMIAR_BLOKU)]);
			}
			printf("\r\n");
			osDelay(1);		//czas na wyjscie printfa(1);
		}
		printf("\r\n");
	}

	//for (uint16_t n=0; n<ROZMIAR_MCU420; n++)
		//printf("%X", chBuforMCU[n]);

	//teraz taki zestaw danych można podać na kompresor
	if (hjpeg.State == HAL_JPEG_STATE_READY)
	{
		chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, 4 * ROZMIAR_BLOKU, chBuforJPEG, ROZMIAR_BUF_JPEG);

	}
	else
		nRozmiarObrazuJPEG = 0;


	return chErr;
}


/**
  * @brief  Convert RGB to Gray blocks pixels
  * @param  pInBuffer  : pointer to input RGB888/ARGB8888 blocks.
  * @param  pOutBuffer : pointer to output Gray blocks buffer.
  * @param  BlockIndex : index of the input buffer first block in the final image.
  * @param  DataCount  : number of bytes in the input buffer .
  * @param  ConvertedDataCount  : number of converted bytes from input buffer.
  * @retval Number of blocks converted from RGB to Gray
  */
/*static uint32_t JPEG_ARGB_MCU_Gray_ConvertBlocks(uint8_t *pInBuffer, uint8_t *pOutBuffer, uint32_t BlockIndex, uint32_t DataCount, uint32_t *ConvertedDataCount)
{
  uint32_t numberMCU;
  uint32_t i,j, currentMCU, xRef,yRef, colones;

  uint32_t refline;
  int32_t offset;

  uint32_t red, green, blue;

  uint8_t *pOutAddr;
  uint8_t *pInAddr;
  uint8_t ycomp;

  numberMCU = (DataCount / (JPEG_BYTES_PER_PIXEL * GRAY_444_BLOCK_SIZE));

  currentMCU = BlockIndex;
  *ConvertedDataCount = numberMCU * GRAY_444_BLOCK_SIZE;

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

    for(i= 0; i <  JPEG_ConvertorParams.V_factor; i++)
    {
      pInAddr = &pInBuffer[0] ;

      for(j=0; j < colones; j++)
      {
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
        ycomp  = (uint8_t)((int32_t)(*(RED_Y_LUT + red)) + (int32_t)(*(GREEN_Y_LUT + green)) + (int32_t)(*(BLUE_Y_LUT + blue)));

        (*(pOutAddr + offset)) = (ycomp);

        pInAddr += JPEG_BYTES_PER_PIXEL;
        offset++;
      }
      offset += (JPEG_ConvertorParams.H_factor - colones);
      refline += JPEG_ConvertorParams.ScaledWidth;
    }
    pOutAddr +=  JPEG_ConvertorParams.BlockSize;
  }

  return numberMCU;
}*/



//przykład z chata
#include <stdint.h>
#include <string.h>

#define WIDTH   480
#define HEIGHT  320

#define MCU_W   16
#define MCU_H   16

#define Y_BLOCK_SIZE   64   // 8x8
#define C_BLOCK_SIZE   64



// Funkcja: kopiowanie bloku 8x8 z płaszczyzny
void copyBlock8x8(uint8_t *dst, const uint8_t *src, int stride, int startX, int startY)
{
    for (int y = 0; y < 8; y++) {
        memcpy(&dst[y * 8], &src[(startY + y) * stride + startX], 8);
    }
}

// Funkcja: przygotowanie jednego MCU (YUV420)
void prepareMCU(int mcuX, int mcuY, uint8_t *Yplane, uint8_t *Cbplane, uint8_t *Crplane, uint8_t *dstBuffer)
{
    int baseX = mcuX * MCU_W;
    int baseY = mcuY * MCU_H;

    // 4 bloki Y (każdy 8x8)
    copyBlock8x8(&dstBuffer[0 * 64], Yplane, WIDTH, baseX, baseY);       // Y0
    copyBlock8x8(&dstBuffer[1 * 64], Yplane, WIDTH, baseX + 8, baseY);   // Y1
    copyBlock8x8(&dstBuffer[2 * 64], Yplane, WIDTH, baseX, baseY + 8);   // Y2
    copyBlock8x8(&dstBuffer[3 * 64], Yplane, WIDTH, baseX + 8, baseY + 8); // Y3

    // Blok Cb (skalowany 8x8 → indeksy dzielone przez 2)
    int cbBaseX = mcuX * 8;
    int cbBaseY = mcuY * 8;
    copyBlock8x8(&dstBuffer[4 * 64], Cbplane, WIDTH / 2, cbBaseX, cbBaseY);

    // Blok Cr
    copyBlock8x8(&dstBuffer[5 * 64], Crplane, WIDTH / 2, cbBaseX, cbBaseY);
}

// Funkcja: przygotowanie całego obrazu do podania DMA encoderowi
void prepareImage(uint8_t *Yplane, uint8_t *Cbplane, uint8_t *Crplane, void (*feedMCU)(uint8_t *mcuData))
{
    //int mcuPerRow = WIDTH / MCU_W;   // 480 / 16 = 30
    //int mcuPerCol = HEIGHT / MCU_H;  // 320 / 16 = 20

	int mcuPerRow = 32 / MCU_W;
	int mcuPerCol = 16 / MCU_H;

    for (int my = 0; my < mcuPerCol; my++) {
        for (int mx = 0; mx < mcuPerRow; mx++) {
            prepareMCU(mx, my, Yplane, Cbplane, Crplane, chBuforMCU);

            // Przekazanie MCU do enkodera JPEG
            //feedMCU(chBuforMCU);
        }
    }
}
