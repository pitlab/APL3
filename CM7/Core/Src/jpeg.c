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
uint8_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) chBuforJPEG[ROZMIAR_BUF_JPEG] = {0};	//SekcjaZewnSRAM ma wyłączony cache w MPU, więc może pracować z MDMA bez konieczności czyszczenia cache
uint32_t nRozmiarObrazuJPEG;	//w bajtach
uint8_t chWynikKompresji;		//flagi ustawiane w callbackach, określajace postęp przepływu danych przez enkoder JPEG
uint8_t chBuforMCU[6 * ROZMIAR_BLOKU];  //Bufor wyjściowy dla jednego MCU UVY420: [Y0][Y1][Y2][Y3][Cb][Cr]


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
	HAL_JPEG_Pause(hjpeg, JPEG_PAUSE_RESUME_INPUT);	//zatrzymaj obróbkę danych wejściowych. Zostanie wznowiona w pętli formatowania danych
	chWynikKompresji |= KOMPR_PUSTE_WE;		//na wejście enkodera można podać nowe dane
}



////////////////////////////////////////////////////////////////////////////////
//skompresowane dane sa gotowe
// Callback HAL_JPEG_DataReadyCallback is asserted when the HAL JPEG driver has filled the given output buffer with the given size.
////////////////////////////////////////////////////////////////////////////////
void HAL_JPEG_DataReadyCallback(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength)
{
	nRozmiarObrazuJPEG = ROZMIAR_BUF_JPEG - OutDataLength;
	chWynikKompresji |= KOMPR_MCU_GOTOWY;	//podany MCU jest już skompresowany
	//printf("Rozmiar: %ld, kompresja: %.2f\r\n",  nRozmiarObrazuJPEG, (float)(480*320) / nRozmiarObrazuJPEG);
	printf("Rozmiar: %ld\r\n",  nRozmiarObrazuJPEG);
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
uint8_t KompresujYUV420(uint8_t *chObrazWe, uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t *chDaneSkompresowane, uint32_t nRozmiarBuforaJPEG)
{
	uint8_t chErr = BLAD_OK;
	uint32_t nOffsetMCU_wzgObrazu;	//przesuniecie bloku wzgledem początku obrazu
	uint32_t nOffsetMCU_wzgWiersza;	//przesunięcie obrazu wejsciowego wzgledem początku bieżącego wiersza MCU
	uint32_t nOffsetObrazuWe;		//przesunęcie obrazu wejściowego względem początku MCU
	uint8_t chLiczbaMCU_Szer = sSzerokosc / (2 * SZEROKOSC_BLOKU);	//liczba MCU luminancji (czwórek bloków) po szerokosci obrazu
	uint8_t chLiczbaMCU_Wys = sWysokosc / (2 * WYSOKOSC_BLOKU);	//liczba MCU luminancji (czwórek bloków) po wysokosci obrazu
	uint8_t chLicznikTimeoutu;

	chErr = KonfigurujKompresjeJpeg(sSzerokosc, sWysokosc, JPEG_GRAYSCALE_COLORSPACE, 60);	//JPEG_YCBCR_COLORSPACE,  JPEG_GRAYSCALE_COLORSPACE
	if (chErr)
	{
		if (hjpeg.State == HAL_JPEG_STATE_BUSY_ENCODING)
			HAL_JPEG_Abort(&hjpeg);
		return chErr;
	}

	//for (uint32_t n=0; n<ROZMIAR_BUF_JPEG; n++)
		//chBuforJPEG[n] = 0;

	//wypełnij oba bloki chrominancji neutralną wartoscią, niezmienną dla całego obrazu
	for (uint16_t n=(4 * ROZMIAR_BLOKU); n<(6 * ROZMIAR_BLOKU); n++)
		chBuforMCU[n] = 128;

	//konfiguruj bufor na dane skompresowane
	HAL_JPEG_ConfigOutputBuffer(&hjpeg, chDaneSkompresowane, nRozmiarBuforaJPEG);
	chWynikKompresji |= KOMPR_PUSTE_WE;	//proces startuje z flagą gotowości do przyjęcia danych

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
					chBuforMCU[x + (0 * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);

				//następny z prawej blok luminancji Y
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (1 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (1 * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);

				//trzeci blok luminancji Y poniżej pierwszego, przesuniety o 2 bloki i liczbę MCU w szerokości obrazu
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (chLiczbaMCU_Szer * 2 * ROZMIAR_BLOKU)		//przesunięcie o pierwszy wiersz bloków
								+ (0 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (2 * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);

				//czwarty blok luminancji Y
				nOffsetObrazuWe = (y * chLiczbaMCU_Szer * 2 * SZEROKOSC_BLOKU)	//przesunięcie pełnych wierszy obrazu
								+ (chLiczbaMCU_Szer * 2 * ROZMIAR_BLOKU)		//przesunięcie o pierwszy wiersz bloków
								+ (1 * SZEROKOSC_BLOKU);						//przesunięcie wzgledem początku MCU
				for (uint16_t x=0; x<SZEROKOSC_BLOKU; x++)
					chBuforMCU[x + (3 * ROZMIAR_BLOKU) + (y * SZEROKOSC_BLOKU)] = *(chObrazWe + x + nOffsetObrazuWe + nOffsetMCU_wzgWiersza + nOffsetMCU_wzgObrazu);
			}
			//teraz 2 bloki chrominancji
			// na razie pomijam chrominancję

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
					return BLAD_TIMEOUT;
			}
			//printf("timeout MCU: %02X ", 100 - chLicznikTimeoutu);
			chWynikKompresji &= ~KOMPR_PUSTE_WE;	//kasuj flagę po sprawdzeniu

			HAL_JPEG_ConfigInputBuffer(&hjpeg, chBuforMCU, 256);	//przekaź bufor z nowymi danymi
			HAL_JPEG_Resume(&hjpeg, JPEG_PAUSE_RESUME_INPUT);		//wzów kompresję

			//kompresja jednego MCU na bieżąco aby nie przechowywać danych i nie czekać później na konwersję, niech się robi w tle
			if (hjpeg.State == HAL_JPEG_STATE_READY)
			{
				chErr = HAL_JPEG_Encode_DMA(&hjpeg, chBuforMCU, 4 * ROZMIAR_BLOKU, chBuforJPEG, ROZMIAR_BUF_JPEG);	//dla Y8 jest mniej bloków
				chWynikKompresji &= ~KOMPR_PUSTE_WE;	//zdejmij flagę pustego  enkodera
			}
		}
	}

	//czekaj aż ostatnie MCU zostanie skompresowane
	chLicznikTimeoutu = 100;
	while((chWynikKompresji & KOMPR_MCU_GOTOWY) == 0)	//czekaj aż HAL_JPEG_DataReadyCallback ustawi flagę końca kompresji i zaktualizuje rozmiar danych
	{
		osDelay(1);
		chLicznikTimeoutu--;
		if (!chLicznikTimeoutu)
			return BLAD_TIMEOUT;
	}
	//printf("Finalny timeout: %02X ", 100 - chLicznikTimeoutu);
	return chErr;
}

