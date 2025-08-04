/*
 * audio.h
 *
 *  Created on: Jan 5, 2025
 *      Author: PitLab
 */

#ifndef INC_AUDIO_H_
#define INC_AUDIO_H_

#include "sys_def_CM7.h"

#define CZESTOTLIWOSC_AUDIO		16000
#define ROZMIAR_BUFORA_AUDIO	256
#define ROZMIAR_BUFORA_AUDIO_WE	480		//tyle danych aby wykres można było wrzucić na ekran
#define ROZMIAR_BUFORA_PAPUGI	(uint32_t)(CZESTOTLIWOSC_AUDIO * 10)	//16kHz * 10 sekund
#define SKALA_GLOSNOSCI_AUDIO	128		//rozpiętość głosnośco od zera do tej wartosci
#define SKALA_GLOSNOSCI_TONU	128		//aby uzyskać finalną amplitud
#define LICZBA_TONOW_WARIO		50		//liczba tonów sygnału wario, musi być liczbą parzystą
#define MIN_OKRES_TONU			4		//minimalny rozmiar okresu tonu dla najwyższej częstotliwości
#define SKOK_TONU				1		//zmiana długości okresu między kolejnymi tonami
#define ROZMIAR_BUFORA_TONU		MIN_OKRES_TONU + (SKOK_TONU * LICZBA_TONOW_WARIO)
#define PRZERW_TONU_NA_SEK		5		//tyle przerw na sekundę chcemy dla najwyższego tonu
#define LICZBA_PROBEK_PRZERWY	(CZESTOTLIWOSC_AUDIO / PRZERW_TONU_NA_SEK)		//tyle próbek trzeba odczekać aby przerwać dżwięk
#define PRZERWA_TONU_WZNOSZ		(LICZBA_PROBEK_PRZERWY * (LICZBA_TONOW_WARIO/2))	//ponieważ zwiększamy o (LICZBA_TONOW_WARIO/2) to pomnóż o tyle
#define ROZM_KOLEJKI_KOMUNIKATOW	16
#define ROZM_MALEJ_KOLEJKI_KOMUNIK	4


//definicje próbek głosowych audio odpowiadająca kolejności w pliku komunikaty*.txt
#define	PRGA_00				0	//00.wav
#define	PRGA_01				1	//01.wav
#define	PRGA_02				2	//02.wav
#define	PRGA_03				3	//03.wav
#define	PRGA_04				4	//04.wav
#define	PRGA_05				5	//05.wav
#define	PRGA_06				6	//06.wav
#define	PRGA_07				7	//07.wav
#define	PRGA_08				8	//08.wav
#define	PRGA_09				9	//09.wav
#define	PRGA_10				10	//10.wav
#define	PRGA_11				11	//11.wav
#define	PRGA_12				12	//12.wav
#define	PRGA_13				13	//13.wav
#define	PRGA_14				14	//14.wav
#define	PRGA_15				15	//15.wav
#define	PRGA_16				16	//16.wav
#define	PRGA_17				17	//17.wav
#define	PRGA_18				18	//18.wav
#define	PRGA_19				19	//19.wav
#define	PRGA_20				20	//20.wav
#define	PRGA_30				21	//30.wav
#define	PRGA_40				22	//40.wav
#define	PRGA_50				23	//50.wav
#define	PRGA_60				24	//60.wav
#define	PRGA_70				25	//70.wav
#define	PRGA_80				26	//80.wav
#define	PRGA_90				27	//90.wav
#define	PRGA_100			28	//100.wav
#define	PRGA_200			29	//200.wav
#define	PRGA_300			30	//300.wav
#define	PRGA_400			31	//400.wav
#define	PRGA_500			32	//500.wav
#define	PRGA_600			33	//600.wav
#define	PRGA_700			34	//700.wav
#define	PRGA_800			35	//800.wav
#define	PRGA_900			36	//900.wav
#define	PRGA_TYSIAC			37	//tysiac.wav
#define	PRGA_TYSIACE		38	//tysiace.wav
#define	PRGA_TYSIECY		39	//tysiecy.wav
#define	PRGA_I				40	//i.wav
#define	PRGA_JEDNA			41	//jedna.wav
#define	PRGA_DWIE			42	//dwie.wav
#define	PRGA_DZIESIATA		43	//dziesiata.wav
#define	PRGA_DZIESIATE		44	//dziesiate.wav
#define	PRGA_DZIESIATYCH	45	//dziesiatych.wav
#define	PRGA_MINUS			46	//minus.wav
#define	PRGA_NAPIECIE		47	//napiecie.wav
#define	PRGA_WOLTA			48	//wolta.wav
#define	PRGA_WOLT			49	//wolt.wav
#define	PRGA_WOLTY			50	//wolty.wav
#define	PRGA_WOLTOW			51	//woltow.wav
#define	PRGA_TEMPERATURA	52	//temperatura.wav
#define	PRGA_STOPNIA		53	//stopnia.wav
#define	PRGA_STOPIEN		54	//stopien.wav
#define	PRGA_STOPNIE		55	//stopnie.wav
#define	PRGA_STOPNI			56	//stopni.wav
#define	PRGA_WYSOKOSC		57	//wysokosc.wav
#define	PRGA_METRA			58	//metra.wav
#define	PRGA_METR			59	//metr.wav
#define	PRGA_METRY			60	//metry.wav
#define	PRGA_METROW			61	//metrow.wav
#define	PRGA_PREDKOSC		62	//predkosc.wav
#define	PRGA_NA_SEKUNDE		63	//na_sekunde.wav
#define	PRGA_KIERUNEK		64	//kierunek.wav
#define	PRGA_ALLELUJA		65	//Alleluja_i_do_gory.wav
#define	PRGA_NIECHAJ_NARODO	66	//niechaj_narodowie.wav
#define PRGA_OS				67
#define PRGA_X				68
#define PRGA_Y				69
#define PRGA_Z				70
#define PRGA_MIN			71
#define PRGA_MAX			72
#define PRGA_GOTOWE			73
#define PRGA_PRZYCISK		74
#define PRGA_GOTOWY_SLUZYC	75
#define PRGA_MAX_PROBEK		76	//maksymalna liczba dostępnych próbek
#define PRGA_PUSTE_MIEJSCE	255	//poste miejsce bez żadnej próbki


//obliczenia: Maksymalna finalna amplituda to: (sin(1harm) * AMPLITUDA_1HARM + sin(3harm) * AMPLITUDA_3HARM) * SKALA_GLOSNOSCI_TONU
//Należy tak dobrać amplitudy harmonicznych aby mieściły się w 16 bitach (+-32768)i dawały dodać do komunikatu dźwiękowego bez większych zniekstałceń
#define AMPLITUDA_1HARM			2048	//amplituda pierwszej harmonicznej sygnału
#define AMPLITUDA_3HARM			2048	//amplituda trzeciej harmonicznej sygnału

uint8_t InicjujAudio(void);
uint8_t ObslugaWymowyKomunikatu(void);
uint8_t OdtworzProbkeAudio(uint32_t nAdres, uint32_t nRozmiar);
uint8_t OdtworzProbkeAudioZeSpisu(uint8_t chNrProbki);
void UstawTon(uint8_t chNrTonu, uint8_t chGlosnosc);
void ZatrzymajTon(void);
uint8_t InicjujRejestracjeDzwieku(void);
uint8_t InicjujOdtwarzanieDzwieku(void);
uint8_t NapelnijBuforDzwieku(int16_t *sBufor, uint16_t sRozmiar);
uint8_t DodajProbkeDoKolejki(uint8_t chNumerProbki);
uint8_t DodajProbkeDoMalejKolejki(uint8_t chNumerProbki, uint8_t chRozmiarKolejki);
uint8_t PrzygotujKomunikat(uint8_t chTypKomunikatu, float fWartosc);
void TestKomunikatow(void);
uint8_t DlugoscKolejkiKomunikatow(void);
uint8_t PrzepiszProbkeDoDRAM(uint8_t chNrProbki);
void NormalizujDzwiek(int16_t* sBufor, uint32_t nRozmiar, uint8_t chGlosnosc);
#endif /* INC_AUDIO_H_ */
