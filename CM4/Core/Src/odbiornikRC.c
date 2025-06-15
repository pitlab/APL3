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

stRC_t stRC;	//struktura danych odbiorników RC
extern unia_wymianyCM4_t uDaneCM4;

////////////////////////////////////////////////////////////////////////////////
// Funkcja wczytuje z FRAM konfigurację odbiorników RC
// Parametry:
// Zwraca: kod błędu
// Czas wykonania:
////////////////////////////////////////////////////////////////////////////////
uint8_t InicjujOdbiornikiRC(void)
{
	uint8_t chErr = ERR_OK;



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
