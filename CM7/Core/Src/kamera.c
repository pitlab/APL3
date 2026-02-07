//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł osługi kamery DCMI na układzie OV5642 o rozdzielczości 2592x1944
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
// KAMERA		APL3	Port uC
//  1 VCC		2		---
//  2 GND	1, 11, 20	---
//  3 SCL		3		PH4,  I2C2_SCL
//  4 SDA		4		PH5,  I2C2_SDA
//  5 VSYNC		5		PB7,  DCMI_VSYNC
//  6 HREF		6		PA4,  DCMI_HSYNC
//  7 PCLK		7		PA6,  DCMI_PIXCLK
//  8 XCLK		8		PH6,  TIM14_CH1
//  9 DOUT9 	12		PI7,  DCMI_D7
// 10 DOUT8		13		PI6,  DCMI_D6
// 11 DOUT7		14		PI4,  DCMI_D5
// 12 DOUT6		15		PH14, DCMI_D4
// 13 DOUT5		16		PG11, DCMI_D3
// 14 DOUT4 	17		PG10, DCMI_D2
// 15 DOUT3		18		PC7,  DCMI_D1
// 16 DOUT2		19		PH9,  DCMI_DO
// 17 PWDN				---
// 18 RSV				---
// 19 DOUT1				---
// 20 DOUT0				---
//////////////////////////////////////////////////////////////////////////////
#include "kamera.h"
#include "polecenia_komunikacyjne.h"
#include "moduly_SPI.h"
#include "protokol_kom.h"
#include "display.h"
#include "jpeg.h"
#include "analiza_obrazu.h"
#include "cmsis_os.h"
#include "osd.h"

//uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamerySRAM[SZER_ZDJECIA * WYS_ZDJECIA / 2] = {0};
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) sBuforKamery[SZER_ZDJECIA * WYS_ZDJECIA] = {0};
//uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaDRAM"))) sBuforKameryDRAM[ROZM_BUF16_KAM] = {0};
extern uint8_t chBuforJpeg[ROZM_BUF_WY_JPEG];
extern uint8_t chBuforLCD[DISP_X_SIZE * DISP_Y_SIZE * 3];
extern uint8_t chBuforOSD[DISP_X_SIZE * DISP_Y_SIZE * ROZMIAR_KOLORU_OSD];	//pamięć obrazu OSD
stKonfKam_t stKonfKam;
static stKonfKam_t stPoprzKonfig;
struct sensor_reg stListaRejestrow[ROZMIAR_STRUKTURY_REJESTROW_KAMERY];
volatile uint16_t sLicznikLiniiKamery;
volatile uint16_t sLicznikRamekKamery;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;
extern DMA2D_HandleTypeDef hdma2d;
extern TIM_HandleTypeDef htim12;
extern I2C_HandleTypeDef hi2c2;
extern const struct sensor_reg OV5642_RGB_QVGA[];
extern const uint8_t chAdres_expandera[];
extern uint8_t chPort_exp_wysylany[];
extern JPEG_HandleTypeDef hjpeg;
extern uint32_t nRozmiarObrazuJPEG;	//w bajtach
uint32_t nRozmiarObrazuKamery;	//w bajtach
volatile uint8_t chObrazKameryGotowy;	//flaga gotowości obrazu, ustawiana w callbacku
stDiagKam_t stDiagKam;	//diagnostyka stanu kamery
uint8_t chWskNapBufKam;	//wskaźnik napełnaniania bufora kamery
volatile uint8_t chBladKamery;	//1=HAL_DCMI_ERROR_OVR, 2=DCMI_ERROR_SYNC, 3=HAL_DCMI_ERROR_TIMEOUT, 4=HAL_DCMI_ERROR_DMA
extern uint32_t nCzasBlend, nCzasLCD;
extern stKonfOsd_t stKonfOSD;

////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja pracy kamery
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjalizujKamere(void)
{
	uint8_t chErr;
	uint32_t nHCLK = HAL_RCC_GetHCLKFreq();
	TIM_OC_InitTypeDef sConfigOC = {0};

	//ustaw i włącz przerwania
	HAL_NVIC_SetPriority(DCMI_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(DCMI_IRQn);

	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

	//Zresetuj kamerę. Power Down jest cały czas nieaktywny = L
	chPort_exp_wysylany[0] &= ~EXP03_CAM_RESET;		//CAM_RES - reset kamery ustaw aktywny niski
	chErr = WyslijDaneExpandera(chAdres_expandera[0], chPort_exp_wysylany[0]);	//wyślij dane do expandera I/O
	if (chErr)
		return chErr;

	//ustawienie timera generującego PWM 20MHz jako zegar dla przetwornika kamery
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 0;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = (nHCLK / KAMERA_ZEGAR) - 1;		//częstotliwość PWM
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	chErr = HAL_TIM_OC_Init(&htim12);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse =  (nHCLK / KAMERA_ZEGAR) / 2;	//wypełnienie PWM
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	chErr |= HAL_TIM_OC_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
	chErr = HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

	HAL_Delay(30);	//power on period. Nie używać osdelay ponieważ w czasie inicjalizacji system jeszcze nie wstał
	chErr = SprawdzKamere();	//wewnątrz funkcji ustawia reset = H
	if (chErr)
	{
		HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);	//zatrzymaj taktowanie skoro nie ma kamery
		HAL_NVIC_DisableIRQ(DCMI_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		return chErr;
	}

	Wyslij_I2C_Kamera(0x3103, 0x93);	//PLL clock select: [1] PLL input clock: 1=from pre-divider
	Wyslij_I2C_Kamera(0x3008, 0x82);	//system control 00: [7] software reset mode, [6] software power down mode {def=0x02}
	HAL_Delay(2);						//Reset settling time <1ms

	chErr = Wyslij_Blok_Kamera(OV5642_RGB_QVGA);					//150ms @ 20MHz
	if (chErr)
		return chErr;

	//ustaw domyślne parametry pracy kamery
	stKonfKam.chSzerWe = KAM_SZEROKOSC_OBRAZU / KROK_ROZDZ_KAM * KAM_ZOOM_CYFROWY;
	stKonfKam.chWysWe = KAM_WYSOKOSC_OBRAZU / KROK_ROZDZ_KAM * KAM_ZOOM_CYFROWY;
	stKonfKam.chSzerWy = KAM_SZEROKOSC_OBRAZU / KROK_ROZDZ_KAM;
	stKonfKam.chWysWy = KAM_WYSOKOSC_OBRAZU / KROK_ROZDZ_KAM;
	stKonfKam.chPrzesWyPoz = 0;
	stKonfKam.chPrzesWyPio = 0;
	stKonfKam.chObracanieObrazu = 0xc1;		//TIMING TC REG18: [6] mirror, [5] Vertial flip, [4] 1=thumbnail mode, [3] 1=compression, [1] vertical subsample 1/4, [0] vertical subsample 1/2  <def:0x80>
	stKonfKam.chFormatObrazu = 0x6F;		//format control [7..4] 6=RGB656, [3..0] 1={G[2:0}, B[4:0]},{R[4:0], G[5:3]} - OK
	stKonfKam.sWzmocnienieR = 0x0400;		//AWB red gain[11:0] / 0x400;
	stKonfKam.sWzmocnienieG = 0x0400;		//AWB green gain[11:0] / 0x400;
	stKonfKam.sWzmocnienieB = 0x0400;		//AWB blue gain[11:0] / 0x400;
	stKonfKam.chKontrBalBieli = 0x00;		//AWB Manual enable[0]: 0=Auto, 1=manual
	stKonfKam.sCzasEkspozycji = 0x03E8;		//AEC Long Channel Exposure [19:0]: 0x3500..02
	stKonfKam.chKontrolaExpo = 0x00;		//AEC PK MANUAL 0x3503: AEC Manual Mode Control: [2]-VTS manual, [1]-AGC manual, [0]-AEC manual: pełna kontrola automatyczna
	stKonfKam.chTrybyEkspozycji = 0x7C;		//AEC CTRL0 0x3A00: [7]-not used, [6]-less one line mode, [5]-band function, [4]-band low limit mode, [3]-reserved, [2]-Night mode (ekspozycja trwa 1..8 ramek) [1]-not used, [0]-Freeze
	stKonfKam.sAGCLong = 0;					//AEC PK Long Gain 0x3508..09
	stKonfKam.sAGCAdjust = 0;				//AEC PK AGC ADJ 0x350A..0B
	stKonfKam.sAGCVTS = 0;					//AEC PK VTS Output 0x350C..0D
	stKonfKam.chKontrolaISP0 = 0xDF;		//ISP Control 00 default value
	stKonfKam.chKontrolaISP1 = 0x4F;		//ISP Control 01 default value
	stKonfKam.chProgUsuwania = 0x40;		//Even CTRL 00
	stKonfKam.chNasycenie = 0x40;			//SDE Control3 Saturation U i V: 0x5583 i 0x5584
	stKonfKam.chPoziomEkspozycji = SKALA_POZIOMU_EKSPOZYCJI / 2;	//ustawiana jest grupa rejestrów z czego podstawowy to 0x3A10

	//memcpy(&stPoprzKonfig, &stKonfKam, sizeof(stKonfKam));	//wykonuje kopię konfiguracji
	return UstawKamere(&stKonfKam);
}



////////////////////////////////////////////////////////////////////////////////
// Odczytaj dane z rejestru kamery
// Parametry:
//  rejestr - 16 bitowy adres rejestru kamery
//  dane - dane zapisywane do rejestru
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t Czytaj_I2C_Kamera(uint16_t rejestr, uint8_t *dane)
{
	uint8_t dane_wy[2];
	uint8_t chErr;

	dane_wy[0] = (rejestr & 0xFF00) >> 8;
	dane_wy[1] = (rejestr & 0x00FF);
	chErr = HAL_I2C_Master_Transmit(&hi2c2, OV5642_I2C_ADR, dane_wy, 2, KAMERA_TIMEOUT);
	if (chErr == 0)
		chErr = HAL_I2C_Master_Receive(&hi2c2, OV5642_I2C_ADR, dane, 1, KAMERA_TIMEOUT);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Wyślij polecenie konfiguracyjne do kamery przez I2C4. Funkcja wysyłajaca jest blokująca, kończy się po wysłaniu danych.
// Parametry:
//  rejestr - 16 bitowy adres rejestru kamery
//  dane - dane zapisywane do rejestru
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t Wyslij_I2C_Kamera(uint16_t rejestr, uint8_t dane)
{
	uint8_t dane_wy[3];

	dane_wy[0] = (rejestr & 0xFF00) >> 8;
	dane_wy[1] = (rejestr & 0x00FF);
	dane_wy[2] = dane;
	return HAL_I2C_Master_Transmit(&hi2c2, OV5642_I2C_ADR, dane_wy, 3, KAMERA_TIMEOUT);
}



////////////////////////////////////////////////////////////////////////////////
// Wyślij polecenie konfiguracyjne do kamery przez I2C2. Funkcja wysyłajaca jest blokująca, kończy się po wysłaniu danych.
// Parametry:
//  rejestr - 16 bitowy adres rejestru kamery
//  dane - dane zapisywane do rejestru
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t Wyslij_Blok_Kamera(const struct sensor_reg reglist[])
{
	const struct sensor_reg *next = reglist;
	uint8_t chErr;

	while ((next->sRejestr != 0xFFFF) && (chErr == 0))
	{
		chErr = Wyslij_I2C_Kamera(next->sRejestr, next->chWartosc);
		next++;
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Sprawdź czy mamy kontakt z kamerą
// Parametry: brak
// Zwraca: systemowy kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t	SprawdzKamere(void)
{
	uint8_t chErr = ERR_BRAK_KAMERY;
	uint16_t sDaneH;
	uint8_t chDaneL, chPowtorz = 10;

	do
	{
		//pnieważ moduły kamer OV5042 i OV5040 mają unaczej ułożone piny RESET i POWER_DOWN a mamy możliwość sterowania tylko jednym pinem, sprawdź obecność kamery w obu stanach
		chPort_exp_wysylany[0] ^= EXP03_CAM_RESET;		//CAM_RES - zmień stan linii resetu kamery
		chErr = WyslijDaneExpandera(chAdres_expandera[0], chPort_exp_wysylany[0]);	//wyślij dane do expandera I/O
		if (chErr)
			return chErr;
		HAL_Delay(2);	//nie używać osDelay ponieważ w czasie inicjalizacji system jeszcze nie działa

		Czytaj_I2C_Kamera(0x300A, (uint8_t*)&sDaneH);	//Chip ID High Byte = 0x56
		Czytaj_I2C_Kamera(0x300B, &chDaneL);	//Chip ID Low Byte = 0x42
		chPowtorz--;
		sDaneH <<= 8;
		sDaneH |= chDaneL;
	}
	while ((sDaneH != OV5642_ID) && (sDaneH != OV5640_ID) && chPowtorz);	//na raze OV5640 nie działa, ponieważ linia PDOWN jest na APL3 stale == L a na module kamery wypada na linii RESET, więc kamera jest na stale zresetowana
	if (chPowtorz == 0)
		return chErr;
	else
	{
		nZainicjowanoCM7 |= INIT_KAMERA;
		return chErr;
	}
}



////////////////////////////////////////////////////////////////////////////////
// uruchamia DCMI w trybie pojedyńczego zdjęcia jako aparat lub ciagłej pracy jako kamera
// Parametry: chAparat - 1 = KAM_ZDJECIE, tryb pojedyńczego zdjęcia, 0 = KAM_FILM, tryb filmu
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijPraceDCMI(stKonfKam_t* konfig, uint16_t* sBufor, uint32_t nRozmiarObrazu32bit)
{
	uint8_t chErr;
	//uint32_t nRozmiarObrazu32bit;

	//konfiguracja DMA do DCMI
	hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	if (konfig->chTrybPracy == KAM_ZDJECIE)
		hdma_dcmi.Init.Mode = DMA_NORMAL;
	else
		hdma_dcmi.Init.Mode = DMA_CIRCULAR;
	hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_dcmi.Init.Priority = DMA_PRIORITY_VERY_HIGH;
	hdma_dcmi.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_dcmi.Init.MemBurst = DMA_MBURST_INC4;
	hdma_dcmi.Init.PeriphBurst = DMA_PBURST_SINGLE;
	chErr = HAL_DMA_Init(&hdma_dcmi);
	if (chErr)
		return chErr;

	HAL_DMA_RegisterCallback(&hdma_dcmi, HAL_DMA_XFER_CPLT_CB_ID, &DCMI_DMAXferCplt);
	HAL_DMA_RegisterCallback(&hdma_dcmi, HAL_DMA_XFER_HALFCPLT_CB_ID, &DCMI_DMAXferHalfCplt);
	HAL_DMA_RegisterCallback(&hdma_dcmi, HAL_DMA_XFER_HALFCPLT_CB_ID, &DCMI_DMAError);

	//bez względu na format danych RGB565 lub YCbCr, kamera zwraca tą samą liczbę danych: 16 bitów na piksel
	//nRozmiarObrazu32bit = (konfig.chSzerWy * KROK_ROZDZ_KAM) * (konfig.chWysWy * KROK_ROZDZ_KAM) / 2;
	chObrazKameryGotowy = 0;	//flaga jest ustawiana w callbacku: HAL_DCMI_FrameEventCallback

	//Konfiguracja transferu DMA z DCMI do pamięci
	if (konfig->chTrybPracy == KAM_ZDJECIE)		//KAM_ZDJECIE=1 lub KAM_FILM=0
		chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBufor, nRozmiarObrazu32bit);
	else
	{
		chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)sBufor, nRozmiarObrazu32bit);
		//chErr = HAL_DMAEx_MultiBufferStart_IT(hdcmi.DMA_Handle, (uint32_t)&DCMI->DR, (uint32_t)sBufor, (uint32_t)(sBufor + nRozmiarObrazu32bit), nRozmiarObrazu32bit);
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// uruchamia DCMI w trybie pojedyńczego zdjęcia jako aparat lub ciagłej pracy jako kamera
// Parametry: chAparat - 1 = KAM_ZDJECIE, tryb pojedyńczego zdjęcia, 0 = KAM_FILM, tryb filmu
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t ZakonczPraceDCMI(void)
{
	return HAL_DCMI_Stop(&hdcmi);
}



////////////////////////////////////////////////////////////////////////////////
// Czeka z timeoutem aż DCMI w trybie snapshot przechwyci cały obraz
// Cała ramka obrazu trwa 21,6ms, wiec czekam max 6 ramek
// Parametry: brak
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t CzekajNaKoniecPracyDCMI(uint16_t sWysokoscZdjecia)
{
	uint8_t chErr = BLAD_OK;
	uint8_t chLicznikTimeoutu = 250;
	do
	{
		osDelay(1);		//przełącz na inny wątek
		chLicznikTimeoutu--;
	}
	while ((sLicznikLiniiKamery <= sWysokoscZdjecia) && chLicznikTimeoutu);
	chErr = HAL_DCMI_Stop(&hdcmi);
	if (!chLicznikTimeoutu)
		chErr = BLAD_TIMEOUT;
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// wykonuje jedno zdjęcie kamerą i trzyma je w sBuforZdjecia
// Parametry: *sBufor - wskaźnik na bufor obrazu
//  nRozmiarObrazu32bit - rozmiar bufora widzianego jako 32-bitowy, ponieważ DMA pracuje na pełnej magistrali
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZrobZdjecie(uint16_t* sBufor, uint32_t nRozmiarObrazu32bit)
{
	uint8_t chErr;
	uint8_t chStatusDCMI;

	chStatusDCMI = HAL_DCMI_GetState(&hdcmi);
	switch (chStatusDCMI)
	{
	case HAL_DCMI_STATE_READY:	break;	//jest OK kontynuuj pracę
	case HAL_DCMI_STATE_BUSY:			//jeżeli trwa praca kamery to ją zatrzymaj i zrób zdjęcie
		chErr = HAL_DCMI_Stop(&hdcmi);
		if (chErr)
			return chErr;
		break;

	default:
		chErr = HAL_DCMI_Stop(&hdcmi);
		return ERR_BRAK_KAMERY;	//jeżeli nie typowy stan to zwróc bład
	}

	//wyczyść obraz
	for (uint32_t n=0; n<nRozmiarObrazu32bit; n++)
		*((uint32_t*)sBufor + n) = 0;

	chObrazKameryGotowy = 0;
	sLicznikLiniiKamery = 0;
	sLicznikRamekKamery = 0;

	//Konfiguracja transferu MDMA z DCMI do pamięci
	chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBufor, nRozmiarObrazu32bit);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Callback od końca wiersza kamery
// Parametry:
//  hdcmi - wskaźnik na interfejs DCMI
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	sLicznikLiniiKamery++;
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
	chBladKamery = hdcmi->ErrorCode;	//1=HAL_DCMI_ERROR_OVR, 2=DCMI_ERROR_SYNC, 3=HAL_DCMI_ERROR_TIMEOUT, 4=HAL_DCMI_ERROR_DMA
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
}



////////////////////////////////////////////////////////////////////////////////
// Callback gotowej klatki obrazu
// Uruchami DMA2D, które pobiera obraz do dalszej obróbki. Na razie tylko do OSD
// Parametry:
//  hdcmi - wskaźnik na interfejs DCMI
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	chObrazKameryGotowy = 1;
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	if (stKonfOSD.chOSDWlaczone)	//czy OSD jest włączone?
	{
		if (hdma2d.State == HAL_DMA2D_STATE_BUSY)
			HAL_DMA2D_Abort(&hdma2d);
		nCzasBlend = DWT->CYCCNT;
		HAL_DMA2D_BlendingStart_IT(&hdma2d, (uint32_t)chBuforOSD, (uint32_t)sBuforKamery, (uint32_t)chBuforLCD, stKonfKam.chSzerWy * KROK_ROZDZ_KAM, stKonfKam.chWysWy * KROK_ROZDZ_KAM);
	}
}



////////////////////////////////////////////////////////////////////////////////
// Callback gotowej linii obrazu - nieużyteczny
// Parametry:
//  hdcmi - wskaźnik na interfejs DCMI
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	sLicznikRamekKamery++;
}

//callback
void DCMI_DMAXferCplt(struct __DMA_HandleTypeDef * hdma)
{
	sLicznikRamekKamery++;
}

void DCMI_DMAXferHalfCplt(DMA_HandleTypeDef * hdma)
{
	sLicznikRamekKamery++;
}

void DCMI_DMAError(struct __DMA_HandleTypeDef * hdma)
{
	sLicznikRamekKamery++;
}

////////////////////////////////////////////////////////////////////////////////
// Zbiera ustawienie wielu rejestrów w jedną z 4 grup w pamięci kamery aby odpalić je pojedynczym poleceniem w obrębie jednej klatki obrazu
// Parametry: chNrGrupy - numer grupy rejestrów [0..3]
//   stListaRejestrow - wskaźnika na tablicę struktur zawierajaca kolejne rejestry do zgrupowania
//   chLiczbaRejestrow - liczba rejestrów w strukturze do zgrupowania max: ROZMIAR_STRUKTURY_REJESTROW_KAMERY
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UtworzGrupeRejestrowKamery(uint8_t chNrGrupy, struct sensor_reg *stListaRejestrow, uint8_t chLiczbaRejestrow)
{
	uint8_t chPolecenie;
	uint8_t chErr;

	chPolecenie = 0x00 + (chNrGrupy & 0x03);	//zbuduj polecenie utworzenia grupy rejestrów
	chErr = Wyslij_I2C_Kamera(0x3212, chPolecenie);
	if (chErr)
		return chErr;	//jeżeli na początku jest już błąd to nie brnij dalej

	if (chLiczbaRejestrow >= ROZMIAR_STRUKTURY_REJESTROW_KAMERY)
		return ERR_ZLE_DANE;

	for (uint8_t n=0; n<chLiczbaRejestrow; n++)	//zapisz kolejne rejestry
		chErr |= Wyslij_I2C_Kamera(stListaRejestrow[n].sRejestr, stListaRejestrow[n].chWartosc);

	chPolecenie = 0x10 + (chNrGrupy & 0x03);	//zbuduj polecenie zakończenia grupy rejestrów
	chErr |= Wyslij_I2C_Kamera(0x3212, chPolecenie);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Uruchamia wykonanie grupy rejestrów
// Parametry: chNrGrupy - numer grupy rejestrów [0..3]
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UruchomGrupeRejestrowKamery(uint8_t chNrGrupy)
{
	uint8_t chPolecenie;

	chPolecenie = 0xA0 + (chNrGrupy & 0x03);	//zbuduj polecenie uruchomienia grupy rejestrów
	return Wyslij_I2C_Kamera(0x3212, chPolecenie);
}



////////////////////////////////////////////////////////////////////////////////
// funkcja umożliwia ręczne wgranie paramerów z menu
// Parametry:
//  sSzerokosc - szerokość wyjściowa obrazu kamery
//  sWysokosc - wysokość wyjściowa obrazu kamery
//  chZoom - mnożnik powiekszenia cyfrowego, powiększa tylukrotnie obraz wejściowy
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawRozdzielczoscKamery(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chZoom)
{
	//wyczyść zmienną obrazu aby zmiany były widoczne
	for (uint32_t n=0; n<ROZM_BUF16_KAM; n++)
		sBuforKamery[n] = 0;

	stKonfKam.chSzerWy = (uint8_t)(sSzerokosc / KROK_ROZDZ_KAM);
	stKonfKam.chWysWy = (uint8_t)(sWysokosc / KROK_ROZDZ_KAM);
	return UstawKamere(&stKonfKam);
}



////////////////////////////////////////////////////////////////////////////////
// konfiguruje wybrane parametry kamery
//
// Parametry: konf - struktura konfiguracji kamery
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawKamere(stKonfKam_t *konf)
{
	uint8_t chErr = 0;
	un8_32_t un8_32;

	chErr = HAL_DCMI_Suspend(&hdcmi);	//zatrzymaj pracę na czas konfiguracji
	if (konf->chKontrolaISP0 != stPoprzKonfig.chKontrolaISP0)	//zapisz tylko gdy była zmiana
	{
		stPoprzKonfig.chKontrolaISP0 = konf->chKontrolaISP0;
		chErr |= Wyslij_I2C_Kamera(0x5000, konf->chKontrolaISP0);	//0x5000
	}

	if (konf->chKontrolaISP1 != stPoprzKonfig.chKontrolaISP1)
	{
		stPoprzKonfig.chKontrolaISP1 = konf->chKontrolaISP1;
		chErr |= Wyslij_I2C_Kamera(0x5001, konf->chKontrolaISP1);	//ISP control 01: [7] Special digital effects, [6] UV adjust enable, [5]1=Vertical scaling enable, [4]1=Horizontal scaling enable, [3] Line stretch enable, [2] UV average enable, [1] color matrix enable, [0] auto white balance AWB
	}
	if (chErr)
		return chErr;

	//ustaw położenie względem początku obrazu w poziomie
	if (konf->chPrzesWyPoz != stPoprzKonfig.chPrzesWyPoz)
	{
		un8_32.dane16[0] = (uint16_t)konf->chPrzesWyPoz * KROK_ROZDZ_KAM;
		stPoprzKonfig.chPrzesWyPoz = konf->chPrzesWyPoz;
		//ustaw początek obrazu w X
		chErr |= Wyslij_I2C_Kamera(0x3800, un8_32.dane8[1]);	//Timing HS: [11:8] HREF Horizontal start point high byte
		chErr |= Wyslij_I2C_Kamera(0x3801, un8_32.dane8[0]);	//Timing HS: [7:0] HREF Horizontal start point low byte
		//ustaw współrzędną X początku okna uśredniania jasności
		chErr |= Wyslij_I2C_Kamera(0x5680, un8_32.dane8[1]);	//AVG X start [11:8] Horizontal start position for averaging window
		chErr |= Wyslij_I2C_Kamera(0x5681, un8_32.dane8[0]);	//AVG X start [7:0]
	}

	//ustaw położenie względem początku obrazu w pionie
	if (konf->chPrzesWyPio != stPoprzKonfig.chPrzesWyPio)
	{
		un8_32.dane16[0] = (uint16_t)konf->chPrzesWyPio * KROK_ROZDZ_KAM;
		stPoprzKonfig.chPrzesWyPio = konf->chPrzesWyPio;
		//ustaw początek obrazu w Y
		chErr |= Wyslij_I2C_Kamera(0x3802, un8_32.dane8[1]);	//Timing VS: [11:8] HREF Vertical start point high byte
		chErr |= Wyslij_I2C_Kamera(0x3803, un8_32.dane8[0]);	//Timing VS: [7:0] HREF Vertical start point low byte
		//ustaw współrzędną Y początku okna uśredniania jasności
		chErr |= Wyslij_I2C_Kamera(0x5684, un8_32.dane8[1]);	//AVG Y start [10:8] Vertical start point for averaging window
		chErr |= Wyslij_I2C_Kamera(0x5685, un8_32.dane8[0]);	//AVG Y start [7:0]
	}

	//ustaw szerokość wejściową
	if (konf->chSzerWe != stPoprzKonfig.chSzerWe)
	{
		stPoprzKonfig.chSzerWe = konf->chSzerWe;
		//ustaw szerokość X okna obrazu
		un8_32.dane16[0] = (uint16_t)konf->chSzerWe * KROK_ROZDZ_KAM;
		chErr |= Wyslij_I2C_Kamera(0x3804, un8_32.dane8[1]);	//Timing HW: [3:0] Horizontal width high byte 0x500=1280,  0x280=640, 0x140=320 (scale input}
		chErr |= Wyslij_I2C_Kamera(0x3805, un8_32.dane8[0]);	//Timing HW: [7:0] Horizontal width low byte

		//ustaw współrzędną X końca okna uśredniania jasności = poczatek obrazu X + szerokość obrazu
		un8_32.dane16[0] = ((uint16_t)konf->chPrzesWyPoz * KROK_ROZDZ_KAM) + ((uint16_t)konf->chSzerWe * KROK_ROZDZ_KAM);
		chErr |= Wyslij_I2C_Kamera(0x5682, un8_32.dane8[1]);	//AVG X end [11:8] Horizontal start position for averaging window
		chErr |= Wyslij_I2C_Kamera(0x5683, un8_32.dane8[0]);	//AVG X end [7:0]
	}

	//ustaw wysokość wejściową
	if (konf->chWysWe != stPoprzKonfig.chWysWe)
	{
		stPoprzKonfig.chWysWe = konf->chWysWe;
		//ustaw wysokość Y okna obrazu
		un8_32.dane16[0] = (uint16_t)konf->chWysWe * KROK_ROZDZ_KAM;
		chErr |= Wyslij_I2C_Kamera(0x3806, un8_32.dane8[1]);	//Timing VH: [3:0] HREF vertical height high byte 0x3C0=960, 0x1E0=480, 0x0F0=240
		chErr |= Wyslij_I2C_Kamera(0x3807, un8_32.dane8[0]);	//Timing VH: [7:0] HREF vertical height low byte
		//ustaw współrzędną Y końca okna uśredniania jasności = poczatek obrazu Y + wysokość obrazu
		un8_32.dane16[0] = ((uint16_t)konf->chPrzesWyPio * KROK_ROZDZ_KAM) + ((uint16_t)konf->chWysWe * KROK_ROZDZ_KAM);
		chErr |= Wyslij_I2C_Kamera(0x5686, un8_32.dane8[1]);	//AVG Y end [10:8] Vertical end point for averaging window
		chErr |= Wyslij_I2C_Kamera(0x5687, un8_32.dane8[0]);	//AVG Y end [7:0]
	}

	//ustaw szerokość wyjściową
	if (konf->chSzerWy != stPoprzKonfig.chSzerWy)
	{
		stPoprzKonfig.chSzerWy = konf->chSzerWy;
		un8_32.dane16[0] = (uint16_t)konf->chSzerWy * KROK_ROZDZ_KAM;
		chErr |= Wyslij_I2C_Kamera(0x3808,  un8_32.dane8[1]);	//Timing DVPHO: [3:0] output horizontal width high byte [11:8]
		chErr |= Wyslij_I2C_Kamera(0x3809,  un8_32.dane8[0]);	//Timing DVPHO: [7:0] output horizontal width low byte [7:0]
	}

	//ustaw wysokość wyjściową
	if (konf->chWysWy != stPoprzKonfig.chWysWy)
	{
		stPoprzKonfig.chWysWy = konf->chWysWy;
		un8_32.dane16[0] = (uint16_t)konf->chWysWy * KROK_ROZDZ_KAM;
		chErr |= Wyslij_I2C_Kamera(0x380A, un8_32.dane8[1]);	//Timing DVPVO: [3:0] output vertical height high byte [11:8]
		chErr |= Wyslij_I2C_Kamera(0x380B, un8_32.dane8[0]);	//Timing DVPVO: [7:0] output vertical height low byte [7:0]
	}

	if (konf->chObracanieObrazu != stPoprzKonfig.chObracanieObrazu)
	{
		stPoprzKonfig.chObracanieObrazu = konf->chObracanieObrazu;
		chErr |= Wyslij_I2C_Kamera(0x3818, konf->chObracanieObrazu);	//Timing Control: 0x3818 (ustaw rotację w poziomie i pionie), //for the mirror function it is necessary to set registers 0x3621 [5:4] and 0x3801
	}

	if (konf->chFormatObrazu != stPoprzKonfig.chFormatObrazu)
	{
		stPoprzKonfig.chFormatObrazu = konf->chFormatObrazu;
		chErr |= Wyslij_I2C_Kamera(0x4300, konf->chFormatObrazu);		//Format Control 0x4300
	}

	//regulacja balansu bieli
	if (konf->sWzmocnienieR != stPoprzKonfig.sWzmocnienieR)
	{
		stPoprzKonfig.sWzmocnienieR = konf->sWzmocnienieR;
		un8_32.dane16[0] = konf->sWzmocnienieR & 0xFFF;
		chErr |= Wyslij_I2C_Kamera(0x3400, un8_32.dane8[1]);	//AWB R Gain [11:0]: 0x3400..01
		chErr |= Wyslij_I2C_Kamera(0x3401, un8_32.dane8[0]);
	}
	if (konf->sWzmocnienieG != stPoprzKonfig.sWzmocnienieG)
	{
		stPoprzKonfig.sWzmocnienieG = konf->sWzmocnienieG;
		un8_32.dane16[0] = konf->sWzmocnienieG & 0xFFF;
		chErr |= Wyslij_I2C_Kamera(0x3402, un8_32.dane8[1]);	//AWB G Gain [11:0]: 0x3402..03
		chErr |= Wyslij_I2C_Kamera(0x3403, un8_32.dane8[0]);
	}
	if (konf->sWzmocnienieB != stPoprzKonfig.sWzmocnienieB)
	{
		stPoprzKonfig.sWzmocnienieB = konf->sWzmocnienieB;
		un8_32.dane16[0] = konf->sWzmocnienieB & 0xFFF;
		chErr |= Wyslij_I2C_Kamera(0x3404, un8_32.dane8[1]);	//AWB B Gain [11:0]: 0x3404..05
		chErr |= Wyslij_I2C_Kamera(0x3405, un8_32.dane8[0]);
	}

	if (konf->chKontrBalBieli != stPoprzKonfig.chKontrBalBieli)
	{
		stPoprzKonfig.chKontrBalBieli = konf->chKontrBalBieli;
		chErr |= Wyslij_I2C_Kamera(0x3406, konf->chKontrBalBieli);	//AWB Manual: 0x3406
	}

	//nasycenie koloru wysyłane jest do obu rejestrów: U i V
	if (konf->chNasycenie != stPoprzKonfig.chNasycenie)
	{
		stPoprzKonfig.chNasycenie = konf->chNasycenie;
		chErr |= Wyslij_I2C_Kamera(0x5583, konf->chNasycenie);	//SDE Control3 Saturation U: 0x5583
		chErr |= Wyslij_I2C_Kamera(0x5584, konf->chNasycenie);	//SDE Control3 Saturation V: 0x5584
	}

	//regulacja czułosci
	if (konf->chKontrolaExpo != stPoprzKonfig.chKontrolaExpo)
	{
		stPoprzKonfig.chKontrolaExpo = konf->chKontrolaExpo;
		chErr |= Wyslij_I2C_Kamera(0x3503, konf->chKontrolaExpo);		//AEC Manual Mode Control: 0x3503
	}

	if (konf->chTrybyEkspozycji != stPoprzKonfig.chTrybyEkspozycji)
	{
		stPoprzKonfig.chTrybyEkspozycji = konf->chTrybyEkspozycji;
		chErr |= Wyslij_I2C_Kamera(0x3A00, konf->chTrybyEkspozycji);	//AEC System Control 0: 0x3A00
	}

	if (konf->sCzasEkspozycji != stPoprzKonfig.sCzasEkspozycji)
	{
		stPoprzKonfig.sCzasEkspozycji = konf->sCzasEkspozycji;
		un8_32.dane32 = (uint32_t)konf->sCzasEkspozycji << 4;
		chErr |= Wyslij_I2C_Kamera(0x3500, un8_32.dane8[2]);	//AEC Long Channel Exposure [19:0]: 0x3500..02
		chErr |= Wyslij_I2C_Kamera(0x3501, un8_32.dane8[1]);
		chErr |= Wyslij_I2C_Kamera(0x3502, un8_32.dane8[0]);
	}

	if (konf->sAGCLong != stPoprzKonfig.sAGCLong)
	{
		stPoprzKonfig.sAGCLong = konf->sAGCLong;
		un8_32.dane16[0] = konf->sAGCLong & 0x1FF;
		chErr |= Wyslij_I2C_Kamera(0x3508, un8_32.dane8[1]);	//AEC PK Long Gain 0x3508..09
		chErr |= Wyslij_I2C_Kamera(0x3509, un8_32.dane8[0]);
	}

	if (konf->sAGCAdjust != stPoprzKonfig.sAGCAdjust)
	{
		stPoprzKonfig.sAGCAdjust = konf->sAGCAdjust;
		un8_32.dane16[0] = konf->sAGCAdjust & 0x1FF;
		chErr |= Wyslij_I2C_Kamera(0x350A, un8_32.dane8[1]);	//AEC PK AGC ADJ 0x350A..0B
		chErr |= Wyslij_I2C_Kamera(0x350B, un8_32.dane8[0]);
	}

	if (konf->sAGCVTS != stPoprzKonfig.sAGCVTS)
	{
		stPoprzKonfig.sAGCVTS = konf->sAGCVTS;
		un8_32.dane16[0] = konf->sAGCVTS & 0x1FF;
		chErr |= Wyslij_I2C_Kamera(0x350C, un8_32.dane8[1]);	//AEC PK VTS Output 0x350C..0D
		chErr |= Wyslij_I2C_Kamera(0x350D, un8_32.dane8[0]);
	}

	if (konf->chProgUsuwania != stPoprzKonfig.chProgUsuwania)
	{
		stPoprzKonfig.chProgUsuwania = konf->chProgUsuwania;
		chErr |= Wyslij_I2C_Kamera(0x5080, konf->chProgUsuwania);	//0x5080 Even CTRL 00 Treshold for even odd  cancelling
	}

	//ustaw poziom ekspozycji opisany w OV5642 Camera Module Software Application Notes str 27 gdzie ustawiane jest 6 rejestrów
	if (konf->chPoziomEkspozycji != stPoprzKonfig.chPoziomEkspozycji)
	{
		stPoprzKonfig.chPoziomEkspozycji = konf->chPoziomEkspozycji;
		if (konf->chPoziomEkspozycji >= SKALA_POZIOMU_EKSPOZYCJI)	//walidacja wartości ze wzgledu na użyte poniżej funkcje
			konf->chPoziomEkspozycji = SKALA_POZIOMU_EKSPOZYCJI / 2;

		//najbardziej charakterystyczny rejestr 0x3A10 zmienia się od 8 do 88, więc przyjmuję poszerzony zakres zmiany od 0 do 96 co odpowiada jego liniowej zmianie a wartość 96 => SKALA_POZIOMU_EKSPOZYCJI
		chErr |= Wyslij_I2C_Kamera(0x3A0F, konf->chPoziomEkspozycji + 8);	//zaczyna jak 0x3A10, potem o 8 większy. Upraszczając przyjmuję że jest cały czas większy o 8
		chErr |= Wyslij_I2C_Kamera(0x3A10, konf->chPoziomEkspozycji);		//rejestr bazowy
		//parametr 0x3A11 zachowuje się dosyć nieregularnie, ale można przyjąć że zaczyna się o 8 większy rośnie średnio 1,67 raza szybciej niż 0x3A10
		float fTemp = (float)konf->chPoziomEkspozycji * 1.667f;
		chErr |= Wyslij_I2C_Kamera(0x3A11, (uint8_t)fTemp + 8);
		//chErr |= Wyslij_I2C_Kamera(0x3A1B, konf->chPoziomEkspozycji + 8);	//jest o 8 większy niż 0x3A10
		//chErr |= Wyslij_I2C_Kamera(0x3A1E, konf->chPoziomEkspozycji);	//zachowuje się jak bazowy 0x3A10 z korekta na początku. Dla uproszczenia pomijam korektę

		//Sprawdzić czy jest porpzwnie: według global init granica stabilnej pracy jest o 2 jednostki szersza
		chErr |= Wyslij_I2C_Kamera(0x3A1B, konf->chPoziomEkspozycji + 10);	//jest o 8 większy niż 0x3A10
		chErr |= Wyslij_I2C_Kamera(0x3A1E, konf->chPoziomEkspozycji - 2);	//zachowuje się jak bazowy 0x3A10 z korekta na początku. Dla uproszczenia pomijam korektę

		//parametr 0x3A1F ma wartość 0x10 dla wartosci <=0 i 0x20 dla >0. W tym przypadku 0 oznacza połowę skali
		if (konf->chPoziomEkspozycji > SKALA_POZIOMU_EKSPOZYCJI/2)
			chErr |= Wyslij_I2C_Kamera(0x3A1F, 0x20);
		else
			chErr |= Wyslij_I2C_Kamera(0x3A1F, 0x10);
	}
	chErr |= HAL_DCMI_Resume(&hdcmi);	//wznów pracę po zakończonej konfiguracji
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// konfiguruje wybrane parametry kamery
// Buduje struktury rejestrów i zapisuje je do grupy rejestrów w kamerze, tak aby móc je odpalić jednym poleceniem. Na razie nie działa
// Parametry: konf - struktura konfiguracji kamery
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawKamere2(stKonfKam_t *konf)
{
	uint8_t chErr;
	un8_32_t un8_32;

	//1. Najpierw rejestry kontrolne do grupy 0, bo one właczają ustawiane później funkcjonalności
	stListaRejestrow[0].sRejestr = 0x5000;				//ISP control 00: [0] Color InterPolation enable
	stListaRejestrow[0].chWartosc = konf->chKontrolaISP0;
	stListaRejestrow[1].sRejestr = 0x5001;				//ISP control 01: [7] Special digital effects, [6] UV adjust enable, [5]1=Vertical scaling enable, [4]1=Horizontal scaling enable, [3] Line stretch enable, [2] UV average enable, [1] color matrix enable, [0] auto white balance AWB
	stListaRejestrow[1].chWartosc = konf->chKontrolaISP1;
	stListaRejestrow[2].sRejestr = 0x3818;				//Timing Control: 0x3818 (ustaw rotację w poziomie i pionie), //for the mirror function it is necessary to set registers 0x3621 [5:4] and 0x3801
	stListaRejestrow[2].chWartosc = konf->chObracanieObrazu;
	stListaRejestrow[3].sRejestr = 0x4300;				//Format Control 0x4300
	stListaRejestrow[3].chWartosc = konf->chFormatObrazu;

	//ustaw rozdzielczość wejściową
	un8_32.dane16[0] = ((uint16_t)konf->chSzerWe * KROK_ROZDZ_KAM) & 0xFFF;
	stListaRejestrow[4].sRejestr = 0x3804;				//Timing HW: [3:0] Horizontal width high byte 0x500=1280,  0x280=640, 0x140=320 (scale input}
	stListaRejestrow[4].chWartosc = un8_32.dane8[1];
	stListaRejestrow[5].sRejestr = 0x3805;				//Timing HW: [7:0] Horizontal width low byte
	stListaRejestrow[5].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = ((uint16_t)konf->chWysWe * KROK_ROZDZ_KAM) & 0xFFF;
	stListaRejestrow[6].sRejestr = 0x3806;				//Timing VH: [3:0] HREF vertical height high byte 0x3C0=960, 0x1E0=480, 0x0F0=240
	stListaRejestrow[6].chWartosc = un8_32.dane8[1];
	stListaRejestrow[7].sRejestr = 0x3807;				//Timing VH: [7:0] HREF vertical height low byte
	stListaRejestrow[7].chWartosc = un8_32.dane8[0];
	chErr = UtworzGrupeRejestrowKamery(0, stListaRejestrow, 8);
	if (chErr)
		return chErr;

	//2. Grupa odpowiedzialna za obszar obrazowania
	//ustaw położenie względem początku obrazu w poziomie
	un8_32.dane16[0] = ((uint16_t)konf->chPrzesWyPoz * KROK_ROZDZ_KAM) & 0xFFF;
	stListaRejestrow[0].sRejestr = 0x3800;				//Timing HS: [3:0] HREF Horizontal start point high byte [11:8]
	stListaRejestrow[0].chWartosc = un8_32.dane8[1];
	stListaRejestrow[1].sRejestr = 0x3801;				//Timing HS: [7:0] HREF Horizontal start point low byte [7:0]
	stListaRejestrow[1].chWartosc = un8_32.dane8[0];

	//ustaw położenie względem początku obrazu w pionie
	un8_32.dane16[0] = ((uint16_t)konf->chPrzesWyPio * KROK_ROZDZ_KAM) & 0xFFF;
	stListaRejestrow[2].sRejestr = 0x3802;				//Timing VS: [3:0] HREF Vertical start point high byte [11:8]
	stListaRejestrow[2].chWartosc = un8_32.dane8[1];
	stListaRejestrow[3].sRejestr = 0x3803;				//Timing VS: [7:0] HREF Vertical start point low byte [7:0]
	stListaRejestrow[3].chWartosc = un8_32.dane8[0];

	//ustaw rozdzielczość wyjściową
	un8_32.dane16[0] = (uint16_t)konf->chSzerWy * KROK_ROZDZ_KAM;
	stListaRejestrow[4].sRejestr = 0x3808;				//Timing DVPHO: [3:0] output horizontal width high byte [11:8]
	stListaRejestrow[4].chWartosc = un8_32.dane8[1];
	stListaRejestrow[5].sRejestr = 0x3809;				//Timing DVPHO: [7:0] output horizontal width low byte [7:0]
	stListaRejestrow[5].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = (uint16_t)konf->chWysWy * KROK_ROZDZ_KAM;
	stListaRejestrow[6].sRejestr = 0x380A;				//Timing DVPVO: [3:0] output vertical height high byte [11:8]
	stListaRejestrow[6].chWartosc = un8_32.dane8[1];
	stListaRejestrow[7].sRejestr = 0x380B;				//Timing DVPVO: [7:0] output vertical height low byte [7:0]
	stListaRejestrow[7].chWartosc = un8_32.dane8[0];
	chErr = UtworzGrupeRejestrowKamery(1, stListaRejestrow, 8);
	if (chErr)
		return chErr;

	//3. Grupa odpowiedzialna za balans bieli
	stListaRejestrow[0].sRejestr = 0x3406;				//AWB Manual: 0x3406
	stListaRejestrow[0].chWartosc = konf->chKontrBalBieli;

	un8_32.dane16[0] = konf->sWzmocnienieR & 0xFFF;
	stListaRejestrow[1].sRejestr = 0x3400;				//AWB R Gain [11:8]
	stListaRejestrow[1].chWartosc = un8_32.dane8[1];
	stListaRejestrow[2].sRejestr = 0x3401;				//AWB R Gain [7:0]
	stListaRejestrow[2].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = konf->sWzmocnienieG & 0xFFF;
	stListaRejestrow[3].sRejestr = 0x3402;				//AWB G Gain [11:8]
	stListaRejestrow[3].chWartosc = un8_32.dane8[1];
	stListaRejestrow[4].sRejestr = 0x3403;				//AWB G Gain [7:0]
	stListaRejestrow[4].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = konf->sWzmocnienieB & 0xFFF;
	stListaRejestrow[5].sRejestr = 0x3404;				//AWB B Gain [11:8]
	stListaRejestrow[5].chWartosc = un8_32.dane8[1];
	stListaRejestrow[6].sRejestr = 0x3405;				//AWB B Gain [7:0]
	stListaRejestrow[6].chWartosc = un8_32.dane8[0];

	stListaRejestrow[7].sRejestr = 0x5583;				//SDE Control3 Saturation U: 0x5583
	stListaRejestrow[7].chWartosc = konf->chNasycenie;
	stListaRejestrow[8].sRejestr = 0x5584;				//SDE Control3 Saturation V: 0x5584
	stListaRejestrow[8].chWartosc = konf->chNasycenie;
	chErr = UtworzGrupeRejestrowKamery(2, stListaRejestrow, 9);
	if (chErr)
		return chErr;

	//4. Grupa odpowiedzialna czas naświetlania
	stListaRejestrow[0].sRejestr = 0x3A00;				//AEC System Control 0: 0x3A00
	stListaRejestrow[0].chWartosc = konf->chTrybyEkspozycji;
	stListaRejestrow[1].sRejestr = 0x3503;				//AEC Manual Mode Control: 0x3503
	stListaRejestrow[1].chWartosc = konf->chKontrolaExpo;

	un8_32.dane32 = (uint32_t)konf->sCzasEkspozycji << 4;
	stListaRejestrow[2].sRejestr = 0x3500;				//AEC Long Channel Exposure [19:16]
	stListaRejestrow[2].chWartosc = un8_32.dane8[2];
	stListaRejestrow[3].sRejestr = 0x3501;				//AEC Long Channel Exposure [15:8]
	stListaRejestrow[3].chWartosc = un8_32.dane8[1];
	stListaRejestrow[4].sRejestr = 0x3502;				//AEC Long Channel Exposure [7:0]
	stListaRejestrow[4].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = konf->sAGCLong & 0x1FF;
	stListaRejestrow[5].sRejestr = 0x3508;				//AEC PK Long Gain high bit 0x3508 [0]
	stListaRejestrow[5].chWartosc = un8_32.dane8[2];
	stListaRejestrow[6].sRejestr = 0x3509;				//AEC PK Long Gain 0x3509 [7:0]
	stListaRejestrow[6].chWartosc = un8_32.dane8[1];

	un8_32.dane16[0] = konf->sAGCAdjust & 0x1FF;
	stListaRejestrow[7].sRejestr = 0x3508;				//AEC PK AGC ADJ 0x350A..0B
	stListaRejestrow[7].chWartosc = un8_32.dane8[2];
	stListaRejestrow[8].sRejestr = 0x3509;				//AEC PK AGC ADJ 0x350A..0B
	stListaRejestrow[8].chWartosc = un8_32.dane8[1];

	un8_32.dane16[0] = konf->sAGCVTS & 0x1FF;
	stListaRejestrow[9].sRejestr = 0x3508;				//AEC PK VTS Output 0x350C..0D
	stListaRejestrow[9].chWartosc = un8_32.dane8[2];
	stListaRejestrow[10].sRejestr = 0x3509;				//AEC PK VTS Output 0x350C..0D
	stListaRejestrow[10].chWartosc = un8_32.dane8[1];
	chErr = UtworzGrupeRejestrowKamery(3, stListaRejestrow, 11);
	if (chErr)
		return chErr;

	//teraz uruchom wszystko
	chErr |= UruchomGrupeRejestrowKamery(0);
	chErr |= UruchomGrupeRejestrowKamery(1);
	chErr |= UruchomGrupeRejestrowKamery(2);
	chErr |= UruchomGrupeRejestrowKamery(3);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Ustawia kamerę w trybie kolorowym RGB656 generujący obraz na wwyświetlacz LCD
// Parametry: brak
// Zwraca: kod błędu HAL
/*///////////////////////////////////////////////////////////////////////////////
uint8_t UstawObrazKameryRGB565(uint16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr;

	stKonfKam.chTrybPracy = KAM_FILM;
	stKonfKam.chFormatObrazu = 0x6F;	//obraz RGB565
	stKonfKam.chSzerWe = sSzerokosc / KROK_ROZDZ_KAM * 3;
	stKonfKam.chWysWe = sWysokosc / KROK_ROZDZ_KAM * 3;
	stKonfKam.chSzerWy = sSzerokosc / KROK_ROZDZ_KAM;
	stKonfKam.chWysWy = sWysokosc / KROK_ROZDZ_KAM;
	chErr = UstawKamere(&stKonfKam);
	return chErr;
}*/



////////////////////////////////////////////////////////////////////////////////
// Ustawia rozdzielczość, format obrazu i tryb pracy kamery
// Parametry: sSzerokosc, sSzerokosc - rozmiar obrazu w pikselach
//  chFormatObrazu - konfiguracja dla rejestru 0x4300 FormatControl, ustala rodazj danych wychodzących z kamery
//  chTrybPracy - tryb pracy DMA: pojedyńczy (zdjecie) lub ciagły (film)
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawObrazKamery(uint16_t sSzerokosc, uint16_t sWysokosc, uint8_t chFormatObrazu, uint8_t chTrybPracy)
{
	uint8_t chErr;
	uint8_t chZoom, chZoomX, chZoomY;

	nRozmiarObrazuKamery = (uint32_t)sSzerokosc * sWysokosc;

	//znajdź największy zoom cyfrowy dla danego obrazu
	chZoomX = MAX_SZER_KAM / sSzerokosc;
	chZoomY = MAX_WYS_KAM / sWysokosc;
	if (chZoomX > chZoomY)
		chZoom = chZoomY;
	else
		chZoom = chZoomX;

	stKonfKam.chTrybPracy = chTrybPracy;
	stKonfKam.chFormatObrazu = chFormatObrazu;
	stKonfKam.chSzerWe = sSzerokosc / KROK_ROZDZ_KAM * chZoom;
	stKonfKam.chWysWe = sWysokosc / KROK_ROZDZ_KAM * chZoom;
	stKonfKam.chSzerWy = sSzerokosc / KROK_ROZDZ_KAM;
	stKonfKam.chWysWy = sWysokosc / KROK_ROZDZ_KAM;

	//centruj w obrazie
	stKonfKam.chPrzesWyPoz = (MAX_SZER_KAM - (sSzerokosc * chZoom)) / (2 * KROK_ROZDZ_KAM);
	stKonfKam.chPrzesWyPio = (MAX_WYS_KAM - (sWysokosc * chZoom)) / (2 * KROK_ROZDZ_KAM);

	chErr = UstawKamere(&stKonfKam);
	return chErr;
}



void CzyscBufory(void)
{
	for (uint32_t n=0; n<ROZM_BUF16_KAM; n++)
		sBuforKamery[n] = 0;

	for (uint32_t n=0; n<(DISP_X_SIZE * DISP_Y_SIZE * 3); n++)
		chBuforLCD[n] = 0;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytuje z kamery grupę rejestrów pozwalajacą ocenić stan automatyki kamery
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t WykonajDiagnostykeKamery(stDiagKam_t* stDiagKam)
{
	uint8_t chErr, chRej1, chRej2;

	//Odczytaj 0x5690 — to bieżąca średnia jasność (AVG). Jeśli wartość skacze gwałtownie, AEC reaguje za agresywnie
	chErr = Czytaj_I2C_Kamera(0x5690, &stDiagKam->chSredniaJasnoscAVG);	//AVG R10 - average value
	if (chErr)
		return chErr;

	chErr |= Czytaj_I2C_Kamera(0x3A0F, &stDiagKam->chProgAEC_H);	//AEC CTRL0F - high treshold value
	chErr |= Czytaj_I2C_Kamera(0x3A10, &stDiagKam->chProgAEC_L);	//AEC CTRL10 - low treshold value
	chErr |= Czytaj_I2C_Kamera(0x3A1B, &stDiagKam->chProgStabAEC_H);	//high treshold value from image change from stable state to unstable state
	chErr |= Czytaj_I2C_Kamera(0x3A1E, &stDiagKam->chProgStabAEC_L);	//low treshold value from image change from stable state to unstable state

	chErr |= Czytaj_I2C_Kamera(0x3503, &stDiagKam->chTrybEAC_EAG);	//tryb EAC/AEG
	chErr |= Czytaj_I2C_Kamera(0x350C, &chRej1);	//VTSH
	chErr |= Czytaj_I2C_Kamera(0x350D, &chRej2);	//VTSL
	stDiagKam->sMaxCzasEkspoVTS = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x5680, &chRej1);	//okno AVG X start H
	chErr |= Czytaj_I2C_Kamera(0x5681, &chRej2);	//start L
	stDiagKam->sPoczatOknaAVG_X = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x5682, &chRej1);	//end H
	chErr |= Czytaj_I2C_Kamera(0x5683, &chRej2);	//end L
	stDiagKam->sKoniecOknaAVG_X = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x5684, &chRej1);	//okno AVG Y start H
	chErr |= Czytaj_I2C_Kamera(0x5685, &chRej2);	//start L
	stDiagKam->sPoczatOknaAVG_Y = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x5686, &chRej1);	//end H
	chErr |= Czytaj_I2C_Kamera(0x5687, &chRej2);	//end L
	stDiagKam->sKoniecOknaAVG_Y = (uint16_t)chRej1<<8 | chRej2;
	if (chErr)
		return chErr;

	chErr |= Czytaj_I2C_Kamera(0x3800, &chRej1);	//Timing HS
	chErr |= Czytaj_I2C_Kamera(0x3801, &chRej2);
	stDiagKam->sPoczatOknaObrazu_X = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x3802, &chRej1);	//Timing VS
	chErr |= Czytaj_I2C_Kamera(0x3803, &chRej2);
	stDiagKam->sPoczatOknaObrazu_Y = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x3804, &chRej1);	//Timing VW
	chErr |= Czytaj_I2C_Kamera(0x3805, &chRej2);
	stDiagKam->sRozmiarOknaObrazu_X = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x3806, &chRej1);	//Timing VH
	chErr |= Czytaj_I2C_Kamera(0x3807, &chRej2);
	stDiagKam->sRozmiarOknaObrazu_Y = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x380C, &chRej1);	//Timing HTS - total horizontal size
	chErr |= Czytaj_I2C_Kamera(0x380D, &chRej2);
	stDiagKam->sRozmiarPoz_HTS = (uint16_t)chRej1<<8 | chRej2;

	chErr |= Czytaj_I2C_Kamera(0x380E, &chRej1);	//Timing VTS - total vertical size
	chErr |= Czytaj_I2C_Kamera(0x380F, &chRej2);
	stDiagKam->sRozmiarPio_VTS = (uint16_t)chRej1<<8 | chRej2;
	return chErr;
}



/*///////////////////////////////////////////////////////////////////////////////
// Przechwytuje obraz kamery w trybie ciagłym
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t PrzechwycObrazKamery(st_KonfKam* stKonf)
{
	uint8_t chErr;
	chErr = UstawObrazKamery(stKonf->sSzerokosc, stKonf->sWysokosc, OBR_RGB565, KAM_FILM);		//kolor
	if (chErr)
		break;
	chErr = RozpocznijPraceDCMI(stKonfKam, sBuforKamerySRAM, stKonf->sSzerokosc * stKonf->sWysokosc / 2);	//kolor
	if (chErr)
		break;
}*/
