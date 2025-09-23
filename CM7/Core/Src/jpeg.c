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

uint32_t nRozmiarObrazuJPEG;	//w bajtach
uint8_t chObrazSkompresowany;


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
// Przygotuj pojedynczy zestaw MCU 8x8 dla sprzętowego kompresora w formacie YUV420: Y, Y, Y, Y, Cb, Cr
// Parametry: *chObrazY8 - wskaźnik na wejsciowy obraz Y8 czarno-biały
//  *chBlokWyj - wskaźnik na zestaw danych idący do kompresora skonfigurowanego na YUV420
// sSzerokosc, sWysokosc - rozmiary obrazu. sSzerokosc musi być podzielna przez 32, sWysokosc podzielna przez 8
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t KompresujY8(uint8_t *chObrazY8, uint8_t *chBlokWyj, uint16_t sSzerokosc, uint16_t sWysokosc)
{
	//uint16_t sIndeksBlokuMCU;
	uint8_t chErr = BLAD_OK;

	//Do kompresji wybieram format YUV420 jako najbardziej efektywny. Minimalna jednostka zawiera 4 kolejne bloki luminancji i 2 bloki chrominancji: niebieski Cb i czerwony Cr
	//Każdy z bloków nazywanych MCU jest fragmentem obrazu 8x8 pikseli. Taki zestaw danych (chBlokWyj) podawany jest na kompresor sprzetowy
	for (uint16_t y=0; y<WYSOKOSC_MCU; y++)
	{
		//pierwszy blok luminancji Y
		for (uint16_t x=0; x<SZEROKOSC_MCU; x++)
		{
			*(chBlokWyj + x + (0 * ROZMIAR_MCU) + (y * SZEROKOSC_MCU)) = *(chObrazY8 + x + (0 * SZEROKOSC_MCU) + (y * 4 * SZEROKOSC_MCU));
		}
		//drugi blok luminancji Y
		for (uint16_t x=0; x<SZEROKOSC_MCU; x++)
		{
			*(chBlokWyj + x + (1 * ROZMIAR_MCU) + (y * SZEROKOSC_MCU)) = *(chObrazY8 + x + (1 * SZEROKOSC_MCU) + (y * 4 * SZEROKOSC_MCU));
		}
		//trzeci blok luminancji Y
		for (uint16_t x=0; x<SZEROKOSC_MCU; x++)
		{
			*(chBlokWyj + x + (2 * ROZMIAR_MCU) + (y * SZEROKOSC_MCU)) = *(chObrazY8 + x + (2 * SZEROKOSC_MCU) + (y * 4 * SZEROKOSC_MCU));
		}
		//czwarty blok luminancji Y
		for (uint16_t x=0; x<SZEROKOSC_MCU; x++)
		{
			*(chBlokWyj + x + (3 * ROZMIAR_MCU) + (y * SZEROKOSC_MCU)) = *(chObrazY8 + x + (3 * SZEROKOSC_MCU) + (y * 4 * SZEROKOSC_MCU));
		}
		//blok chrominancji niebieskiej Cb
		for (uint16_t x=0; x<SZEROKOSC_MCU; x++)
		{
			*(chBlokWyj + x + (4 * ROZMIAR_MCU) + (y * SZEROKOSC_MCU)) = 128;	//obraz jest szary, więc brak zróżnicowania składowych chrominancji
		}
		//blok chrominancji czerwonej Cr
		for (uint16_t x=0; x<SZEROKOSC_MCU; x++)
		{
			*(chBlokWyj + x + (5 * ROZMIAR_MCU) + (y * SZEROKOSC_MCU)) = 128;	//obraz jest szary, więc brak zróżnicowania składowych chrominancji
		}
	}
	//teraz taki zesyaw danych można podać na kompresor


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

