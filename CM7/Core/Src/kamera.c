//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł osługi kamery DCMI na układzie OV5642
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


ALIGN_32BYTES(uint16_t __attribute__((section(".SekcjaZewnSRAM")))	sBuforKamery[ROZM_BUF16_KAM]);
struct st_KonfKam strKonfKam;

uint16_t sLicznikLiniiKamery;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;
extern TIM_HandleTypeDef htim12;
extern I2C_HandleTypeDef hi2c2;
extern const struct sensor_reg OV5642_RGB_QVGA[];
extern const uint8_t chAdres_expandera[];
extern uint8_t chPort_exp_wysylany[];

////////////////////////////////////////////////////////////////////////////////
// Inicjalizacja pracy kamery
// Parametry: brak
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjalizujKamere(void)
{
	uint8_t chErr;
	uint32_t nHCLK = HAL_RCC_GetHCLKFreq();		//200MHz

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

	//ustawienie timera generującego PWM 20MHz
	chErr = HAL_TIM_Base_Start_IT(&htim12);
	htim12.Instance->ARR = (nHCLK / KAMERA_ZEGAR) - 1;		//częstotliwość PWM
	htim12.Instance->CCR1 = (nHCLK / KAMERA_ZEGAR) / 2;	//wypełnienie PWM
	chErr = HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

	HAL_Delay(10);	//power on period
	chPort_exp_wysylany[0] |= EXP03_CAM_RESET;		//CAM_RES - reset kamery ustaw nieaktywny wysoki
		chErr = WyslijDaneExpandera(chAdres_expandera[0], chPort_exp_wysylany[0]);	//wyślij dane do expandera I/O
		if (chErr)
			return chErr;
	HAL_Delay(30);

	chErr = SprawdzKamere();
	if (chErr)
		return chErr;

	Wyslij_I2C_Kamera(0x3103, 0x93);	//PLL clock select: [1] PLL input clock: 1=from pre-divider
	Wyslij_I2C_Kamera(0x3008, 0x82);	//system control 00: [7] software reset mode, [6] software power down mode {def=0x02}
	HAL_Delay(30);
	/*err = Wyslij_Blok_Kamera(ov5642_dvp_fmt_global_init);		//174ms @ 20MHz
	if (err)
		return err;*/
	chErr = Wyslij_Blok_Kamera(OV5642_RGB_QVGA);					//150ms @ 20MHz
	if (chErr)
		return chErr;

	//ustaw domyślne parametry pracy kamery
	//strKonfKam.sSzerWe = 1280;
	//strKonfKam.sWysWe = 960;
	//strKonfKam.sSzerWy = 320;
	//strKonfKam.sWysWy = 240;
	//strKonfKam.sSzerWe = 1920;
	//strKonfKam.sWysWe = 1280;
	strKonfKam.sSzerWe = 960;
	strKonfKam.sWysWe = 640;
	strKonfKam.sSzerWy = 480;
	strKonfKam.sWysWy = 320;
	strKonfKam.chTrybDiagn = 0;	//brak trybu diagnostycznego
	strKonfKam.chFlagi = 1;

	chErr = UstawKamere(&strKonfKam);
	if (chErr)
		return chErr;

	Wyslij_I2C_Kamera(0x4300, 0x6F);	//format control [7..4] 6=RGB656, [3..0] 1={R[4:0], G[5:3]},{G[2:0}, B[4:0]}

	chErr = RozpocznijPraceDCMI(strKonfKam.chFlagi & FUK1_ZDJ_FILM);	//1 = zdjecie, 0 = film (tylko ten jeden bit)
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Odczytaj dane z rejestru kamery
// Parametry:
//  rejestr - 16 bitowy adres rejestru kamery
//  dane - dane zapisywane do rejestru
// Zwraca: kod błędu
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

	while ((next->reg != 0xFFFF) && (chErr == 0))
	{
		chErr = Wyslij_I2C_Kamera(next->reg, next->val);
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
	uint16_t sDaneH;
	uint8_t chDaneL, powtorz = 10;

	do
	{
		Czytaj_I2C_Kamera(0x300A, (uint8_t*)&sDaneH);	//Chip ID High Byte = 0x56
		Czytaj_I2C_Kamera(0x300B, &chDaneL);	//Chip ID Low Byte = 0x42
		powtorz--;
		HAL_Delay(1);
		sDaneH <<= 8;
		sDaneH |= chDaneL;
	}
	while ((sDaneH != OV5642_ID) && powtorz);
	if (powtorz == 0)
		return ERR_BRAK_KAMERY;
	else
	{
		nZainicjowanoCM7 |= INIT_KAMERA;
		return ERR_OK;
	}
}



////////////////////////////////////////////////////////////////////////////////
// konfiguruje wybrane parametry kamery
// Parametry: konf - struktura konfiguracji kamery
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawKamere(stKonfKam_t *konf)
{
	//uint8_t chReg;

	Wyslij_I2C_Kamera(0x5001, 0x7F);	//ISP control 01: [7] Special digital effects, [6] UV adjust enable, [5]1=Vertical scaling enable, [4]1=Horizontal scaling enable, [3] Line stretch enable, [2] UV average enable, [1] color matrix enable, [0] auto white balance AWB

	//ustaw rozdzielczość wejściową
	Wyslij_I2C_Kamera(0x3804, (uint8_t)(konf->sSzerWe>>8));		//Timing HW: [3:0] Horizontal width high byte 0x500=1280,  0x280=640, 0x140=320 (scale input}
	Wyslij_I2C_Kamera(0x3805, (uint8_t)(konf->sSzerWe & 0xFF));	//Timing HW: [7:0] Horizontal width low byte
	Wyslij_I2C_Kamera(0x3806, (uint8_t)(konf->sWysWe>>8));			//Timing VH: [3:0] HREF vertical height high byte 0x3C0=960, 0x1E0=480, 0x0F0=240
	Wyslij_I2C_Kamera(0x3807, (uint8_t)(konf->sWysWe & 0xFF));		//Timing VH: [7:0] HREF vertical height low byte

	//ustaw rozdzielczość wyjściową
	Wyslij_I2C_Kamera(0x3808, (uint8_t)(konf->sSzerWy>>8));		//Timing DVPHO: [3:0] output horizontal width high byte [11:8]
	Wyslij_I2C_Kamera(0x3809, (uint8_t)(konf->sSzerWy & 0xFF));	//Timing DVPHO: [7:0] output horizontal width low byte [7:0]
	Wyslij_I2C_Kamera(0x380a, (uint8_t)(konf->sWysWy>>8));			//Timing DVPVO: [3:0] output vertical height high byte [11:8]
	Wyslij_I2C_Kamera(0x380b, (uint8_t)(konf->sWysWy & 0xFF));		//Timing DVPVO: [7:0] output vertical height low byte [7:0]

	//wzór testowy
	switch(konf->chTrybDiagn)
	{
	case TDK_KRATA_CB:	//czarnobiała karata
		Wyslij_I2C_Kamera(0x503d , 0x85);	//test pattern: B/W square
		Wyslij_I2C_Kamera(0x503e, 0x1a);	//PRE ISP TEST SETTING2 [7] reserved, [6:4] 1=random data pattern seed enable, [3] 1=test pattern square b/w mode, [2] 1=add test pattern on image data, [1:0] 0=color bar, 1=random data, 2=square data, 3=black image
		break;

	case TDK_PASKI:		//7 pionowych pasków
		Wyslij_I2C_Kamera(0x503d , 0x80);	//test pattern: color bar
		Wyslij_I2C_Kamera(0x503e, 0x00);	//PRE ISP TEST SETTING2 [7] reserved, [6:4] 1=random data pattern seed enable, [3] 1=test pattern square b/w mode, [2] 1=add test pattern on image data, [1:0] 0=color bar, 1=random data, 2=square data, 3=black image
		break;

	case TDK_PRACA:		//normalna praca
	default:
	}

	return 0;
	//ustaw rotację w poziomie i pionie
	//chReg = 0x80 + ((konf->chFlagi && FUK1_OBR_PION) << 6) +  ((konf->chFlagi && FUK1_OBR_POZ) << 5);
	//return Wyslij_I2C_Kamera(0x3818, chReg);	//TIMING TC REG18: [6] mirror, [5] Vertial flip, [4] 1=thumbnail mode,  [3] 1=compression, [1] vertical subsample 1/4, [0] vertical subsample 1/2  <def:0x80>
	//for the mirror function it is necessary to set registers 0x3621 [5:4] and 0x3801
}


////////////////////////////////////////////////////////////////////////////////
// uruchamia DCMI w trybie pojedyńczego zdjęcia jako aparat lub ciagłej pracy jako kamera
// Parametry: chAparat - 1 = tryb pojedyńczego zdjęcia, 0 = tryb filmu
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t RozpocznijPraceDCMI(uint8_t chAparat)
{
	uint8_t chErr;

	hdcmi.Instance = DCMI;
	hdcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
	hdcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_FALLING;
	hdcmi.Init.VSPolarity = DCMI_VSPOLARITY_LOW;
	hdcmi.Init.HSPolarity = DCMI_HSPOLARITY_HIGH;
	hdcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
	hdcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
	hdcmi.Init.JPEGMode = DCMI_JPEG_DISABLE;
	hdcmi.Init.ByteSelectMode = DCMI_BSM_ALL;
	hdcmi.Init.ByteSelectStart = DCMI_OEBS_ODD;
	hdcmi.Init.LineSelectMode = DCMI_LSM_ALL;
	hdcmi.Init.LineSelectStart = DCMI_OELS_ODD;
	hdcmi.Instance->IER = DCMI_IT_FRAME | DCMI_IT_OVR | DCMI_IT_ERR | DCMI_IT_VSYNC | DCMI_IT_LINE;
	chErr = HAL_DCMI_Init(&hdcmi);
	if (chErr)
		return chErr;

	//konfiguracja DMA do DCMI
	hdma_dcmi.Instance = DMA2_Stream1;
	hdma_dcmi.Init.Request = DMA_REQUEST_DCMI;
	hdma_dcmi.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_dcmi.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_dcmi.Init.MemInc = DMA_MINC_ENABLE;
	hdma_dcmi.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_dcmi.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	if (chAparat)		//1 = zdjecie, 0 = film
		hdma_dcmi.Init.Mode = DMA_NORMAL;
	else
		hdma_dcmi.Init.Mode = DMA_CIRCULAR;
	hdma_dcmi.Init.Priority = DMA_PRIORITY_HIGH;
	hdma_dcmi.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	chErr = HAL_DMA_Init(&hdma_dcmi);
	if (chErr)
		return chErr;

	//Konfiguracja transferu DMA z DCMI do pamięci
	if (chAparat)		//1 = zdjecie, 0 = film
		chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBuforKamery, ROZM_BUF16_KAM);
		//chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBuforKamery, strKonfKam.sSzerWy * strKonfKam.sWysWy / 2);

	else
		chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, (uint32_t)sBuforKamery, ROZM_BUF16_KAM);
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// wykonuje jedno zdjęcie kamerą i trzyma je w buforze kamery
// Parametry:
// [i] - sSzerokosc - szerokość zdjecia w pikselach
// [i] - sWysokosc - wysokość zdjęcia w pikselach
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t ZrobZdjecie(int16_t sSzerokosc, uint16_t sWysokosc)
{
	uint8_t chErr;

	chErr = HAL_DCMI_Stop(&hdcmi);
	if (chErr)
		return chErr;

	//skalowanie obrazu
	Wyslij_I2C_Kamera(0x5001, 0x7F);	//ISP control 01: [7] Special digital effects, [6] UV adjust enable, [5]1=Vertical scaling enable, [4]1=Horizontal scaling enable, [3] Line stretch enable, [2] UV average enable, [1] color matrix enable, [0] auto white balance AWB
	Wyslij_I2C_Kamera(0x3804, 0x05);	//Timing HW: [3:0] Horizontal width high byte 0x500=1280,  0x280=640, 0x140=320 (scale input}
	Wyslij_I2C_Kamera(0x3805, 0x00);	//Timing HW: [7:0] Horizontal width low byte
	Wyslij_I2C_Kamera(0x3806, 0x03);	//Timing VH: [3:0] HREF vertical height high byte 0x3C0=960, 0x1E0=480, 0x0F0=240
	Wyslij_I2C_Kamera(0x3807, 0xC0);	//Timing VH: [7:0] HREF vertical height low byte

	//ustaw rozmiar obrazu
	Wyslij_I2C_Kamera(0x3808, (sSzerokosc & 0xFF00)>>8);	//Timing DVPHO: [3:0] output horizontal width high byte [11:8]
	Wyslij_I2C_Kamera(0x3809, (sSzerokosc & 0x00FF));		//Timing DVPHO: [7:0] output horizontal width low byte [7:0]
	Wyslij_I2C_Kamera(0x380a, (sWysokosc & 0xFF00)>>8);		//Timing DVPVO: [3:0] output vertical height high byte [11:8]
	Wyslij_I2C_Kamera(0x380b, (sWysokosc & 0x00FF));		//Timing DVPVO: [7:0] output vertical height low byte [7:0]

	//Konfiguracja transferu DMA z DCMI do pamięci
	//chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBuforKamery, ROZM_BUF16_KAM / 2);
	chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBuforKamery, (uint32_t)sSzerokosc * sWysokosc / 2);
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

