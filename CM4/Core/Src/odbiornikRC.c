//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł ubsługi odbiorników RC
//
// (c) PitLab 2025
// https://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "odbiornikRC.h"
#include "konfig_fram.h"
#include "petla_glowna.h"
#include "fram.h"

stRC_t stRC;	//struktura danych odbiorników RC
extern unia_wymianyCM4_t uDaneCM4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart2_rx;
DMA_HandleTypeDef hdma_uart4_rx;
uint8_t chBuforOdbioruSBus1[ROZMIAR_BUF_ODB_SBUS];
uint8_t chBuforOdbioruSBus2[ROZMIAR_BUF_ODB_SBUS];

////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację odbiorników RC
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOdbiornikiRC(void)
{
	uint8_t chTyp, chErr = ERR_OK;
	TIM_IC_InitTypeDef sConfigIC = {0};

	//czytaj konfigurację i ustaw Port PB8 jako UART4_RX lub TIM4_CH3
	chTyp = CzytajFRAM(FAU_KONF_ODB_RC1);
	chTyp = ODB_RC_SBUS;	//tymczasowo nadpisz konfigurację
	if ((chTyp & MASKA_TYPU_RC) == ODB_RC_PPM)
	{
		//ustaw alternatywną funkcję portu B8
		GPIOB->AFR[1] &= ~0x0000000F;	//wyczyść 4 bity AFR8[3:0]
		GPIOB->AFR[1] |= GPIO_AF2_TIM4;	//AF2 = TIM4_CH3

		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 7;
		chErr = HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);
	}
	else
	if ((chTyp & MASKA_TYPU_RC) == ODB_RC_SBUS)
	{
		//ustaw alternatywną funkcję portu B8
		GPIOB->AFR[1] &= ~0x0000000F;	//wyczyść 4 bity AFR8[3:0]
		GPIOB->AFR[1] |= GPIO_AF8_UART4;	//AF8 = UART4_RX

		huart4.Instance = UART4;
		huart4.Init.BaudRate = 100000;
		huart4.Init.WordLength = UART_WORDLENGTH_8B;
		huart4.Init.StopBits = UART_STOPBITS_1;
		huart4.Init.Parity = UART_PARITY_NONE;
		huart4.Init.Mode = UART_MODE_RX;
		huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart4.Init.OverSampling = UART_OVERSAMPLING_16;
		huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
		huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
		huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
		huart4.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
		chErr = HAL_UART_Init(&huart4);
		chErr = HAL_UARTEx_DisableFifoMode(&huart4);

    	//HAL_UART_Receive_DMA(&huart4, chBuforOdbioruSBus1, ROZMIAR_BUF_ODB_SBUS);
		HAL_UART_Receive_IT(&huart4, chBuforOdbioruSBus1, ROZMIAR_BUF_ODB_SBUS);
	}

	//czytaj konfigurację i ustaw Port PA3 jako UART2_RX lub TIM2_CH4
	chTyp = CzytajFRAM(FAU_KONF_ODB_RC2);
	chTyp = ODB_RC_SBUS;	//tymczasowo nadpisz konfigurację
	if ((chTyp & MASKA_TYPU_RC) == ODB_RC_PPM)
	{
		//ustaw alternatywną funkcję portu PA3
		GPIOA->AFR[0] &= ~0x0000F000;	//wyczyść 4 bity AFR3[3:0]
		GPIOA->AFR[0] |= GPIO_AF1_TIM2;	//AF1 = TIM2_CH4


		sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
		sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
		sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
		sConfigIC.ICFilter = 7;
		chErr = HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4);
	}
	else
	if ((chTyp & MASKA_TYPU_RC) == ODB_RC_SBUS)
	{
		//ustaw alternatywną funkcję portu PA3
		GPIOA->AFR[0] &= ~0x0000F000;	//wyczyść 4 bity AFR3[3:0]
		GPIOA->AFR[0] |= GPIO_AF7_USART2;	//AF7 = USART2_RX



		huart2.Instance = UART4;
		huart2.Init.BaudRate = 100000;
		huart2.Init.WordLength = UART_WORDLENGTH_8B;
		huart2.Init.StopBits = UART_STOPBITS_1;
		huart2.Init.Parity = UART_PARITY_NONE;
		huart2.Init.Mode = UART_MODE_RX;
		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
		huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
		huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
		chErr = HAL_UART_Init(&huart2);
		chErr = HAL_UARTEx_DisableFifoMode(&huart2);

		//HAL_UART_Receive_DMA(&huart2, chBuforOdbioruSBus2, ROZMIAR_BUF_ODB_SBUS);
		HAL_UART_Receive_IT(&huart2, chBuforOdbioruSBus2, ROZMIAR_BUF_ODB_SBUS);
	}
	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Porównuje dane z obu odbiorników RC i wybiera ten lepszy przepisując jego dane do struktury danych CM4
// Parametry:
// [we] *psRC - wskaźnik na strukturę danych odbiorników RC
// [wy] *psDaneCM4 - wskaźnik na strukturę danych CM4
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t DywersyfikacjaOdbiornikowRC(stRC_t* psRC, stWymianyCM4_t* psDaneCM4)
{
	uint8_t chErr = ERR_OK;
	uint32_t nCzasBiezacy = PobierzCzas();
	uint32_t nCzasRC1, nCzasRC2;
	uint8_t n;

	//Sprawdź kiedy przyszły ostatnie dane RC
	nCzasRC1 = MinalCzas2(psRC->nCzasWe1, nCzasBiezacy);
	nCzasRC2 = MinalCzas2(psRC->nCzasWe2, nCzasBiezacy);

	if ((nCzasRC1 < 2*CZAS_RAMKI_PPM_RC) && (nCzasRC2 > 2*CZAS_RAMKI_PPM_RC))	//działa odbiornik 1, nie działa 2
	{
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			if (psRC->sZdekodowaneKanaly1 & (1<<n))
			{
				psDaneCM4->sKanalRC[n] = psRC->sOdb1[n]; 	//przepisz zdekodowane kanały
				psRC->sZdekodowaneKanaly1 &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
	}
	else
	if ((nCzasRC1 > 2*CZAS_RAMKI_PPM_RC) && (nCzasRC2 < 2*CZAS_RAMKI_PPM_RC))	//nie działa odbiornik 1, działa 2
	{
		for (n=0; n<KANALY_ODB_RC; n++)
		{
			if (psRC->sZdekodowaneKanaly2 & (1<<n))
			{
				psDaneCM4->sKanalRC[n] = psRC->sOdb2[n]; 	//przepisz zdekodowane kanały
				psRC->sZdekodowaneKanaly2 &= ~(1<<n);		//kasuj bit obrobionego kanału
			}
		}
	}
	else		//działają oba odbiorniki, określ który jest lepszy
	{

	}

	return chErr;
}
