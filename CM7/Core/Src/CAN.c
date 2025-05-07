//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi magistrali CAN
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "CAN.h"
#include "main.h"

//konfiguracja prametrów czasowych według https://kvaser.com/support/calculators/can-fd-bit-timing-calculator/
//Frequency 50000 [kHz], tolerance 4687 [ppm], node delay 210 [ns]
extern FDCAN_HandleTypeDef hfdcan2;

uint32_t RxLocation;
uint32_t BufferIndex;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t chDaneCanWych[12];
uint8_t chDaneCanPrzych[12];


extern uint8_t chPorty_exp_wysylane[];



////////////////////////////////////////////////////////////////////////////////
// Inicjalizuje proces komunikacji po CAN
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void InicjujCAN(void)
{
	chPorty_exp_wysylane[1] &= ~EXP12_CAN_STANDBY;	//wyłącz standby ustawiając stan niski na wejściu STB
	if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}

	TxHeader.Identifier = ID_MAGNETOMETR;		//identyfikator ramki z danymi magnetometru
	TxHeader.IdType = FDCAN_STANDARD_ID;		//ramka standardowa
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;	//FDCAN_DATA_FRAME lub FDCAN_REMOTE_FRAME
	TxHeader.DataLength = FDCAN_DLC_BYTES_5;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;	//FDCAN_ESI_ACTIVE lub FDCAN_ESI_PASSIVE
	TxHeader.BitRateSwitch =  FDCAN_BRS_OFF;			//FDCAN frames transmitted/received without bit rate switching
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;				//Frame transmitted/received in Classic CAN format
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;	//Do not store Tx events
	TxHeader.MessageMarker = 0;       					//Specifies the message marker to be copied into Tx Event FIFO element for identification of Tx message status. This parameter must be a number between 0 and 0xFF
}

void TestCanTx(void)
{
	//bufor danych pomiarowych osi X
	chDaneCanWych[0] = 1;
	chDaneCanWych[1] = 0x00;
	chDaneCanWych[2] = 0x00;
	chDaneCanWych[3] = 0x12;
	chDaneCanWych[4] = 0x34;


	//HAL_FDCAN_EnableTxBufferRequest(&hfdcan2,  BufferIndex);
	//HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, chDaneCanWych);
	//HAL_FDCAN_AddMessageToTxBuffer(&hfdcan2, &TxHeader, chDaneCanWych, BufferIndex);


	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, chDaneCanWych) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_Delay(100);
}


void TestCanRx(void)
{
	//RxLocation = CAN_RX_FIFO0;
	HAL_FDCAN_GetRxMessage(&hfdcan2,  RxLocation, &RxHeader, chDaneCanPrzych);
}



////////////////////////////////////////////////////////////////////////////////
// callback opróżnienia bufora nadawczego CAN
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan == &hfdcan2)
	{

	}
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{

}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{

}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{

}



////////////////////////////////////////////////////////////////////////////////
// callback napełnienia FIFO odbiorczego CAN
// Parametry: nic
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, chDaneCanPrzych) != HAL_OK)
		{
		  Error_Handler();
		}

		if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
		{
		  Error_Handler();
		}
	}
}

