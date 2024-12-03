//////////////////////////////////////////////////////////////////////////////
//
// Moduł obsługi pętli głównej programu autopilota na rdzeniu CM4
//
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////

#include "petla_glowna.h"
#include "main.h"
#include "fram.h"

////////////////////////////////////////////////////////////////////////////////
// Ustawia sygnału ChipSelect dla modułów i układów pamięci FRAM i Expandera IO
// Adres jesyt ustawiony na portach PK2, PK1 i PI8
// Parametry: modul - adres modułu
// Zwraca: kod błędu
////////////////////////////////////////////////////////////////////////////////
uint8_t UstawDekoderModulow(uint8_t modul)
{
	uint8_t chErr = ERR_OK;

	switch (modul)
	{
	case ADR_MOD1:	//0
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_MOD2:	//1
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_SET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_MOD3:	//2
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_SET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_MOD4:	//3
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_SET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_SET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_RESET);	//ADR2
		break;

	case ADR_NIC:	//4
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_SET);	//ADR2
		break;

	case ADR_EXPIO:	//5
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_SET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_RESET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_SET);	//ADR2
		break;

	case ADR_FRAM:	//6
		HAL_GPIO_WritePin(MODW_ADR0_GPIO_Port, MODW_ADR0_Pin, GPIO_PIN_RESET);	//ADR0
		HAL_GPIO_WritePin(MODW_ADR1_GPIO_Port, MODW_ADR1_Pin, GPIO_PIN_SET);	//ADR1
		HAL_GPIO_WritePin(MODW_ADR2_GPIO_Port, MODW_ADR2_Pin, GPIO_PIN_SET);	//ADR2
		break;

	default:	chErr = ERR_ZLY_ADRES;	break;
	}

	return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// Pętla główna programu autopilota
// Parametry: brak
// Zwraca: nic, program nigdy z niej nie wychodzi
////////////////////////////////////////////////////////////////////////////////
void PetlaGlowna(void)
{
	uint8_t chDane[1024];
	uint16_t n;
	uint32_t nCzas;

	CzytajIdFRAM(chDane);

	nCzas = HAL_GetTick();
	CzytajBuforFRAM(0x1000, chDane, 1024);
	nCzas = MinalCzas(nCzas);

	for (n=0; n<1024; n++)
		chDane[n] = (uint16_t)(n & 0xFF);

	nCzas = HAL_GetTick();
	ZapiszBuforFRAM(0x1000, chDane, 1024);
	nCzas = MinalCzas(nCzas);
}



////////////////////////////////////////////////////////////////////////////////
// Liczy upływ czasu
// Parametry: nStart - licznik czasu na na początku pomiaru
// Zwraca: ilość czasu w milisekundach jaki upłynął do podanego czasu startu
////////////////////////////////////////////////////////////////////////////////
uint32_t MinalCzas(uint32_t nStart)
{
	uint32_t nCzas, nCzasAkt;

	nCzasAkt = HAL_GetTick();
	if (nCzasAkt >= nStart)
		nCzas = nCzasAkt - nStart;
	else
		nCzas = 0xFFFFFFFF - nStart + nCzasAkt;
	return nCzas;
}
