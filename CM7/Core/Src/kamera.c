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

uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforKamery[ROZM_BUF16_KAM] = {0};
uint16_t __attribute__ ((aligned (32))) __attribute__((section(".SekcjaZewnSRAM"))) sBuforObrazu[ROZM_BUF16_KAM] = {0};

struct st_KonfKam stKonfKam;
struct sensor_reg stListaRejestrow[ROZMIAR_STRUKTURY_REJESTROW_KAMERY];
uint16_t sLicznikLiniiKamery;
extern uint32_t nZainicjowanoCM7;		//flagi inicjalizacji sprzętu
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;
extern TIM_HandleTypeDef htim12;
extern I2C_HandleTypeDef hi2c2;
extern const struct sensor_reg OV5642_RGB_QVGA[];
extern const struct sensor_reg ov5642_dvp_fmt_global_init[];
extern const struct sensor_reg OV5642_VGA_preview_setting[];
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

	//ustawienie timera generującego PWM 20MHz jako zegar dla przetwornika makery
	htim12.Instance = TIM12;
	htim12.Init.Prescaler = 0;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim12.Init.Period = (nHCLK / KAMERA_ZEGAR) - 1;		//częstotliwość PWM
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	chErr = HAL_TIM_OC_Init(&htim12);

	//sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse =  (nHCLK / KAMERA_ZEGAR) / 2;	//wypełnienie PWM
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	chErr |= HAL_TIM_OC_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);
	chErr = HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);

	HAL_Delay(10);	//power on period. Nie używać osdelay ponieważ w czasie inicjalizacji system jeszcze nie wstał
	chErr = SprawdzKamere();
	if (chErr)
	{
		HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1);	//zatrzymaj taktowanie skoro nie ma kamery
		HAL_NVIC_DisableIRQ(DCMI_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		return chErr;
	}

	Wyslij_I2C_Kamera(0x3103, 0x93);	//PLL clock select: [1] PLL input clock: 1=from pre-divider
	Wyslij_I2C_Kamera(0x3008, 0x82);	//system control 00: [7] software reset mode, [6] software power down mode {def=0x02}
	HAL_Delay(30);
	/*chErr = Wyslij_Blok_Kamera(ov5642_dvp_fmt_global_init);		//174ms @ 20MHz
	if (chErr)
		return chErr;*/

	//chErr = Wyslij_Blok_Kamera(OV5642_VGA_preview_setting);
	chErr = Wyslij_Blok_Kamera(OV5642_RGB_QVGA);					//150ms @ 20MHz
	if (chErr)
		return chErr;

	//ustaw domyślne parametry pracy kamery
	stKonfKam.chSzerWe = KAM_SZEROKOSC_OBRAZU  / KROK_ROZDZ_KAM * KAM_ZOOM_CYFROWY;
	stKonfKam.chWysWe = KAM_WYSOKOSC_OBRAZU  / KROK_ROZDZ_KAM * KAM_ZOOM_CYFROWY;
	stKonfKam.chSzerWy = KAM_SZEROKOSC_OBRAZU / KROK_ROZDZ_KAM;
	stKonfKam.chWysWy = KAM_WYSOKOSC_OBRAZU / KROK_ROZDZ_KAM;
	//stKonfKam.chPrzesWyPoz = (stKonfKam.chSzerWe - stKonfKam.chSzerWy) / 2;	//środek obrazu
	//stKonfKam.chPrzesWyPio = (stKonfKam.chWysWe - stKonfKam.chWysWy) / 2;	//środek obrazu
	stKonfKam.chPrzesWyPoz = 0;
	stKonfKam.chPrzesWyPio = 0;

	stKonfKam.chTrybDiagn = 0;	//brak trybu diagnostycznego
	//stKonfKam.chTrybDiagn = TDK_KRATA_CB;	//czarnobiała krata
	//stKonfKam.chTrybDiagn = TDK_PASKI;		//kolorowe paski


	//naturalny układ pikseli kamery to: wiersze parzyte B, G, B, G...; wiersze nieparzyste G, R, G, R...
	//chErr = Wyslij_I2C_Kamera(0x4300, 0x61);	//format control [7..4] 6=RGB656, [3..0] 1={R[4:0], G[5:3]},{G[2:0}, B[4:0]} - źle
	//chErr = Wyslij_I2C_Kamera(0x4300, 0x62);	//format control [7..4] 6=RGB656, [3..0] 2={G[4:0], R[5:3]},{R[2:0}, B[4:0]} - źle
	//chErr = Wyslij_I2C_Kamera(0x4300, 0x6F);

	//ustaw rejestry konfiguracyjne
	stKonfKam.chObracanieObrazu = 0xc1;		//TIMING TC REG18: [6] mirror, [5] Vertial flip, [4] 1=thumbnail mode, [3] 1=compression, [1] vertical subsample 1/4, [0] vertical subsample 1/2  <def:0x80>
	stKonfKam.chFormatObrazu = 0x6F;		//format control [7..4] 6=RGB656, [3..0] 1={G[2:0}, B[4:0]},{R[4:0], G[5:3]} - OK
	stKonfKam.sWzmocnienieR = 0x0400;		//AWB red gain[11:0] / 0x400;
	stKonfKam.sWzmocnienieG = 0x0400;		//AWB green gain[11:0] / 0x400;
	stKonfKam.sWzmocnienieB = 0x0400;		//AWB blue gain[11:0] / 0x400;
	stKonfKam.chKontrBalBieli = 0x00;		//AWB Manual enable[0]: 0=Auto, 1=manual
	//stKonfKam.nEkspozycjaReczna = 0x000000;	//AEC Long Channel Exposure [19:0]: 0x3500..02
	stKonfKam.sCzasEkspozycji = 0x0000;		//AEC Long Channel Exposure [19:0]: 0x3500..02

	stKonfKam.chKontrolaExpo = 0x00;		//AEC PK MANUAL 0x3503: AEC Manual Mode Control: [2]-VTS manual, [1]-AGC manual, [0]-AEC manual: pełna kontrola automatyczna

	stKonfKam.chTrybyEkspozycji = 0x7C;		//AEC CTRL0 0x3A00: [7]-not used, [6]-less one line mode, [5]-band function, [4]-band low limit mode, [3]-reserved, [2]-Night mode (ekspozycja trwa 1..8 ramek) [1]-not used, [0]-Freeze
	stKonfKam.chGranicaMinExpo = 0x04; 		//Minimum Exposure Output Limit [7..0] 0x3A01:
	stKonfKam.nGranicaMaxExpo = 0x03D800;	//Maximum Exposure Output Limit [19..0]: 0x3A02..04

	stKonfKam.chKontrolaISP0 = 0xDF;		//ISP Control 00 default value
	stKonfKam.chKontrolaISP1 = 0x4F;		//ISP Control 01 default value
	stKonfKam.chProgUsuwania = 0x40;		//Even CTRL 00
	return UstawKamere(&stKonfKam);
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
		HAL_Delay(30);	//nie używać osDelay ponieważ w czasie inicjalizacji system jeszcze nie działa

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
		chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBuforKamery, stKonfKam.chSzerWy * stKonfKam.chWysWy * KROK_ROZDZ_KAM / 2);

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
uint8_t ZrobZdjecie(void)
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

	//Konfiguracja transferu DMA z DCMI do pamięci
	chErr = HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_SNAPSHOT, (uint32_t)sBuforKamery, ROZM_BUF16_KAM / 2);
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



////////////////////////////////////////////////////////////////////////////////
// Zbiera ustawienie wielu rejestrów w jedną z 4 grup w pamięci kamery aby uruchomić je pojedynczym poleceniem w obrębie jednej klatki obrazu
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
	stKonfKam.chTrybDiagn = 0;			//brak trybu diagnostycznego
	return UstawKamere(&stKonfKam);
}



////////////////////////////////////////////////////////////////////////////////
// konfiguruje wybrane parametry kamery
// Buduje struktury rejestrów i zapisuje je do grupy rejestrów, tak aby móc je odpalić jednym poleceniem
// Parametry: konf - struktura konfiguracji kamery
// Zwraca: kod błędu HAL
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawKamere(stKonfKam_t *konf)
{
	uint8_t chErr = 0;
	un8_32_t un8_32;

	chErr = HAL_DCMI_Suspend(&hdcmi);	//zatrzymaj pracę na czas konfiguracji
	chErr |= Wyslij_I2C_Kamera(0x5000, konf->chKontrolaISP0);	//0x5000
	chErr |= Wyslij_I2C_Kamera(0x5001, konf->chKontrolaISP1);	//ISP control 01: [7] Special digital effects, [6] UV adjust enable, [5]1=Vertical scaling enable, [4]1=Horizontal scaling enable, [3] Line stretch enable, [2] UV average enable, [1] color matrix enable, [0] auto white balance AWB
	if (chErr)
		return chErr;

	//ustaw położenie względem początku obrazu w poziomie
	un8_32.dane16[0] = (uint16_t)konf->chPrzesWyPoz * KROK_ROZDZ_KAM;
	chErr |= Wyslij_I2C_Kamera(0x3800, un8_32.dane8[1]);	//Timing HS: [3:0] HREF Horizontal start point high byte [11:8]
	chErr |= Wyslij_I2C_Kamera(0x3801, un8_32.dane8[0]);	//Timing HS: [7:0] HREF Horizontal start point low byte [7:0]

	//ustaw położenie względem początku obrazu w pionie
	un8_32.dane16[0] = (uint16_t)konf->chPrzesWyPio * KROK_ROZDZ_KAM;
	chErr |= Wyslij_I2C_Kamera(0x3802, un8_32.dane8[1]);	//Timing VS: [3:0] HREF Vertical start point high byte [11:8]
	chErr |= Wyslij_I2C_Kamera(0x3803, un8_32.dane8[0]);	//Timing VS: [7:0] HREF Vertical start point low byte [7:0]

	//ustaw rozdzielczość wejściową
	un8_32.dane16[0] = (uint16_t)konf->chSzerWe * KROK_ROZDZ_KAM;
	chErr |= Wyslij_I2C_Kamera(0x3804, un8_32.dane8[1]);	//Timing HW: [3:0] Horizontal width high byte 0x500=1280,  0x280=640, 0x140=320 (scale input}
	chErr |= Wyslij_I2C_Kamera(0x3805, un8_32.dane8[0]);	//Timing HW: [7:0] Horizontal width low byte

	un8_32.dane16[0] = (uint16_t)konf->chWysWe * KROK_ROZDZ_KAM;
	chErr |= Wyslij_I2C_Kamera(0x3806, un8_32.dane8[1]);	//Timing VH: [3:0] HREF vertical height high byte 0x3C0=960, 0x1E0=480, 0x0F0=240
	chErr |= Wyslij_I2C_Kamera(0x3807, un8_32.dane8[0]);	//Timing VH: [7:0] HREF vertical height low byte

	//ustaw rozdzielczość wyjściową
	un8_32.dane16[0] = (uint16_t)konf->chSzerWy * KROK_ROZDZ_KAM;
	chErr |= Wyslij_I2C_Kamera(0x3808,  un8_32.dane8[1]);	//Timing DVPHO: [3:0] output horizontal width high byte [11:8]
	chErr |= Wyslij_I2C_Kamera(0x3809,  un8_32.dane8[0]);	//Timing DVPHO: [7:0] output horizontal width low byte [7:0]

	un8_32.dane16[0] = (uint16_t)konf->chWysWy * KROK_ROZDZ_KAM;
	chErr |= Wyslij_I2C_Kamera(0x380A, un8_32.dane8[1]);	//Timing DVPVO: [3:0] output vertical height high byte [11:8]
	chErr |= Wyslij_I2C_Kamera(0x380B, un8_32.dane8[0]);	//Timing DVPVO: [7:0] output vertical height low byte [7:0]

	chErr |= Wyslij_I2C_Kamera(0x3818, konf->chObracanieObrazu);	//Timing Control: 0x3818 (ustaw rotację w poziomie i pionie), //for the mirror function it is necessary to set registers 0x3621 [5:4] and 0x3801
	chErr |= Wyslij_I2C_Kamera(0x4300, konf->chFormatObrazu);		//Format Control 0x4300

	un8_32.dane16[0] = konf->sWzmocnienieR & 0xFFF;
	chErr |= Wyslij_I2C_Kamera(0x3400, un8_32.dane8[1]);	//AWB R Gain [11:0]: 0x3400..01
	chErr |= Wyslij_I2C_Kamera(0x3401, un8_32.dane8[0]);

	un8_32.dane16[0] = konf->sWzmocnienieG & 0xFFF;
	chErr |= Wyslij_I2C_Kamera(0x3402, un8_32.dane8[1]);	//AWB G Gain [11:0]: 0x3402..03
	chErr |= Wyslij_I2C_Kamera(0x3403, un8_32.dane8[0]);

	un8_32.dane16[0] = konf->sWzmocnienieB & 0xFFF;
	chErr |= Wyslij_I2C_Kamera(0x3404, un8_32.dane8[1]);	//AWB B Gain [11:0]: 0x3404..05
	chErr |= Wyslij_I2C_Kamera(0x3405, un8_32.dane8[0]);

	chErr |= Wyslij_I2C_Kamera(0x3406, konf->chKontrBalBieli);	//AWB Manual: 0x3406

	//un8_32.dane32 = konf->nEkspozycjaReczna & 0xFFFFF;
	un8_32.dane32 = (uint32_t)konf->sCzasEkspozycji << 4;

	chErr |= Wyslij_I2C_Kamera(0x3500, un8_32.dane8[2]);	//AEC Long Channel Exposure [19:0]: 0x3500..02
	chErr |= Wyslij_I2C_Kamera(0x3501, un8_32.dane8[1]);
	chErr |= Wyslij_I2C_Kamera(0x3502, un8_32.dane8[0]);

	chErr |= Wyslij_I2C_Kamera(0x3503, konf->chKontrolaExpo);		//AEC Manual Mode Control: 0x3503
	chErr |= Wyslij_I2C_Kamera(0x3A00, konf->chTrybyEkspozycji);	//AEC System Control 0: 0x3A00
	chErr |= Wyslij_I2C_Kamera(0x3A01, konf->chGranicaMinExpo);		//Minimum Exposure Output Limit [7..0]: 0x3A01

	un8_32.dane32 = konf->nGranicaMaxExpo & 0xFFFFF;
	chErr |= Wyslij_I2C_Kamera(0x3A14, un8_32.dane8[2]);	//Maximum Exposure Output Limit 50Hz [19..0]: 0x3A02..04
	chErr |= Wyslij_I2C_Kamera(0x3A15, un8_32.dane8[1]);
	chErr |= Wyslij_I2C_Kamera(0x3A16, un8_32.dane8[0]);

	chErr |= Wyslij_I2C_Kamera(0x5080, konf->chProgUsuwania);	//0x5080 Even CTRL 00 Treshold for even odd  cancelling
	chErr |= HAL_DCMI_Resume(&hdcmi);	//wznów pracę po zakończonej konfiguracji
	return chErr;
}


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
	chErr = UtworzGrupeRejestrowKamery(2, stListaRejestrow, 7);
	if (chErr)
		return chErr;

	//4. Grupa odpowiedzialna czas naświetlania
	stListaRejestrow[0].sRejestr = 0x3A00;				//AEC System Control 0: 0x3A00
	stListaRejestrow[0].chWartosc = konf->chTrybyEkspozycji;
	stListaRejestrow[1].sRejestr = 0x3503;				//AEC Manual Mode Control: 0x3503
	stListaRejestrow[1].chWartosc = konf->chKontrolaExpo;
	stListaRejestrow[2].sRejestr = 0x3A01;				//Minimum Exposure Output Limit [7..0]: 0x3A01
	stListaRejestrow[2].chWartosc = konf->chGranicaMinExpo;

	//un8_32.dane32 = konf->nEkspozycjaReczna & 0xFFFFF;
	un8_32.dane32 = (uint32_t)konf->sCzasEkspozycji << 4;
	stListaRejestrow[3].sRejestr = 0x3500;				//AEC Long Channel Exposure [19:16]
	stListaRejestrow[3].chWartosc = un8_32.dane8[2];
	stListaRejestrow[4].sRejestr = 0x3501;				//AEC Long Channel Exposure [15:8]
	stListaRejestrow[4].chWartosc = un8_32.dane8[1];
	stListaRejestrow[5].sRejestr = 0x3502;				//AEC Long Channel Exposure [7:0]
	stListaRejestrow[5].chWartosc = un8_32.dane8[0];

	un8_32.dane32 = konf->nGranicaMaxExpo & 0xFFFFF;
	stListaRejestrow[6].sRejestr = 0x3A14;				//Maximum Exposure Output Limit 50Hz [19:16]
	stListaRejestrow[6].chWartosc = un8_32.dane8[2];
	stListaRejestrow[7].sRejestr = 0x3A15;				//Maximum Exposure Output Limit 50Hz [15:8]
	stListaRejestrow[7].chWartosc = un8_32.dane8[1];
	stListaRejestrow[8].sRejestr = 0x3A16;				//Maximum Exposure Output Limit 50Hz [7:0]
	stListaRejestrow[8].chWartosc = un8_32.dane8[0];
	chErr = UtworzGrupeRejestrowKamery(3, stListaRejestrow, 9);
	if (chErr)
		return chErr;

	//teraz uruchom wszystko
	chErr |= UruchomGrupeRejestrowKamery(0);
	chErr |= UruchomGrupeRejestrowKamery(1);
	chErr |= UruchomGrupeRejestrowKamery(2);
	chErr |= UruchomGrupeRejestrowKamery(3);
	return chErr;
}

void UstawDomyslny(void)
{
	UstawKamere(&stKonfKam);
}


void Ustaw1(void)
{
	UstawKamere2(&stKonfKam);
}

void Ustaw2(void)
{
	un8_32_t un8_32;
	//3. Grupa odpowiedzialna za balans bieli
	stListaRejestrow[0].sRejestr = 0x3406;				//AWB Manual: 0x3406
	stListaRejestrow[0].chWartosc = stKonfKam.chKontrBalBieli;

	un8_32.dane16[0] = stKonfKam.sWzmocnienieR & 0xFFF;
	stListaRejestrow[1].sRejestr = 0x3400;				//AWB R Gain [11:8]
	stListaRejestrow[1].chWartosc = un8_32.dane8[1];
	stListaRejestrow[2].sRejestr = 0x3401;				//AWB R Gain [7:0]
	stListaRejestrow[2].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = stKonfKam.sWzmocnienieG & 0xFFF;
	stListaRejestrow[3].sRejestr = 0x3402;				//AWB G Gain [11:8]
	stListaRejestrow[3].chWartosc = un8_32.dane8[1];
	stListaRejestrow[4].sRejestr = 0x3403;				//AWB G Gain [7:0]
	stListaRejestrow[4].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = stKonfKam.sWzmocnienieB & 0xFFF;
	stListaRejestrow[5].sRejestr = 0x3404;				//AWB B Gain [11:8]
	stListaRejestrow[5].chWartosc = un8_32.dane8[1];
	stListaRejestrow[6].sRejestr = 0x3405;				//AWB B Gain [7:0]
	stListaRejestrow[6].chWartosc = un8_32.dane8[0];
	UtworzGrupeRejestrowKamery(2, stListaRejestrow, 7);
	UruchomGrupeRejestrowKamery(2);
}



void Ustaw3(void)
{
	//un8_32_t un8_32;

	//1. Najpierw rejestry kontrolne do grupy 0, bo one właczają ustawiane później funkcjonalności
	stListaRejestrow[0].sRejestr = 0x5000;				//ISP control 00: [0] Color InterPolation enable
	stListaRejestrow[0].chWartosc = stKonfKam.chKontrolaISP0;
	stListaRejestrow[1].sRejestr = 0x5001;				//ISP control 01: [7] Special digital effects, [6] UV adjust enable, [5]1=Vertical scaling enable, [4]1=Horizontal scaling enable, [3] Line stretch enable, [2] UV average enable, [1] color matrix enable, [0] auto white balance AWB
	stListaRejestrow[1].chWartosc = stKonfKam.chKontrolaISP1;
	stListaRejestrow[2].sRejestr = 0x3818;				//Timing Control: 0x3818 (ustaw rotację w poziomie i pionie), //for the mirror function it is necessary to set registers 0x3621 [5:4] and 0x3801
	stListaRejestrow[2].chWartosc = stKonfKam.chObracanieObrazu;
	stListaRejestrow[3].sRejestr = 0x4300;				//Format Control 0x4300
	stListaRejestrow[3].chWartosc = stKonfKam.chFormatObrazu;

	/*/ustaw rozdzielczość wejściową
	un8_32.dane16[0] = ((uint16_t)stKonfKam.chSzerWe * KROK_ROZDZ_KAM) & 0xFFF;
	stListaRejestrow[4].sRejestr = 0x3804;				//Timing HW: [3:0] Horizontal width high byte 0x500=1280,  0x280=640, 0x140=320 (scale input}
	stListaRejestrow[4].chWartosc = un8_32.dane8[1];
	stListaRejestrow[5].sRejestr = 0x3805;				//Timing HW: [7:0] Horizontal width low byte
	stListaRejestrow[5].chWartosc = un8_32.dane8[0];

	un8_32.dane16[0] = ((uint16_t)stKonfKam.chWysWe * KROK_ROZDZ_KAM) & 0xFFF;
	stListaRejestrow[6].sRejestr = 0x3806;				//Timing VH: [3:0] HREF vertical height high byte 0x3C0=960, 0x1E0=480, 0x0F0=240
	stListaRejestrow[6].chWartosc = un8_32.dane8[1];
	stListaRejestrow[7].sRejestr = 0x3807;				//Timing VH: [7:0] HREF vertical height low byte
	stListaRejestrow[7].chWartosc = un8_32.dane8[0];*/

	UtworzGrupeRejestrowKamery(0, stListaRejestrow, 4);
	UruchomGrupeRejestrowKamery(0);
}
