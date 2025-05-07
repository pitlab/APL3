//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi magistrali CAN
//
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include "LCD.h"

extern FDCAN_HandleTypeDef hfdcan2;

uint32_t RxLocation;
uint32_t BufferIndex;
FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t chDaneCanWych[20];
uint8_t chDaneCanPrzych[20];


void TestCanTx(void)
{
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_EnableTxBufferRequest(&hfdcan2,  BufferIndex);
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, chDaneCanWych);
	HAL_FDCAN_AddMessageToTxBuffer(&hfdcan2, &TxHeader, chDaneCanWych, BufferIndex);


	HAL_FDCAN_Stop(&hfdcan2);
}


void TestCanRx(void)
{
	//RxLocation = CAN_RX_FIFO0;
	HAL_FDCAN_GetRxMessage(&hfdcan2,  RxLocation, &RxHeader, chDaneCanPrzych);
}


/* Callback functions *********************************************************/
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{

}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{

}

