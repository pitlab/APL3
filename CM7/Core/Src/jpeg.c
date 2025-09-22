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






