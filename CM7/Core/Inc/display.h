


#ifndef Display_H_
#define Display_H_



#include "sys_def_CM7.h"
//definicje kolorów
#define	BLACK   	0x0000
#define	BLUE    	0x001F
#define	RED     	0xF800
#define	GREEN   	0x07E0
#define CYAN    	0x07FF
#define MAGENTA 	0xF81F
#define YELLOW  	0xFFE0
#define WHITE 		0xFFFF
#define LIBELLA1	0xAEAA	//p�yn
#define LIBELLA2	0x95EA	//obw�dka b�belka
#define LIBELLA3	0xCFAA	//b�belek
#define GRAY10 		0x18C3	//najciemniejszy
#define GRAY20 		0x3186
#define GRAY30 		0x4A69
#define GRAY40 		0x632C
#define GRAY50 		0x7BEF
#define GRAY60 		0x9CD3
#define GRAY70 		0xB596
#define GRAY80 		0xCE79
#define SILVER		0xC618
#define MAROON		0x8000	//czerwono-br�zowy
#define OLIVE		0x8400	//jasny zgni�ozielony
#define TEAL		0x0410	//ciemny zgni�ozielony
#define NAVY		0x0010	//granatowy
#define PURPLE		0x8010	//fioletowy

#define MENU_TLO_BAR	GRAY40	//kolor belki menu g�rnego
#define MENU_TLO_WYB	GRAY70	//pozycja menu edytowana lub uruchamiana
#define MENU_TLO_AKT	GRAY50	//pozycja menu pod�wietlona
#define MENU_TLO_NAK	GRAY30	//pozycja nieaktywna

#define MENU_RAM_WYB	GREEN
#define MENU_RAM_AKT	WHITE
#define MENU_RAM_NAK	GRAY50
#define TRANSPARENT	0xFFFE	//specjalny kolor t�a



//definicje orientacji
#define POZIOMO 0
#define PIONOWO 1
#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

//rozmiar ekranu
#define DISP_X_SIZE		480
#define DISP_Y_SIZE		320
#define DISP_HX_SIZE	320
#define DISP_HY_SIZE	480
#define DISP_VX_SIZE	480
#define DISP_VY_SIZE	320



#define FONT_SH		12	//wysoko�c ma�ej czcionki
#define FONT_BH		16	//wysoko�c du�ej czcionki
#define FONT_SL		8	//szeroko�� ma�ej czcionki
#define FONT_BL		16	//szeroko�� du�ej czcionki
#define FONT_SLEN	8	//szeroko�� ma�ej czcionki
#define FONT_BLEN	16	//szeroko�� du�ej czcionki
#define UP_SPACE	2	//wolna przestrze� nad tekstem
#define DW_SPACE	4	//wolna przestrze� pod tekstem




//menu obrazkowe na �rodku ekranu
typedef struct tmenu
{
	const char* chOpis;				//wska�nik na opis pod menu
	const char* chPomoc;			//wska�nika na tekst pomocy wy�wietlan� w pasku statusu
	unsigned char chMode;			//numer trybu pracy obs�uguj�cy funkcj�
	const unsigned short* sIkona;	//wska�nika na ikon� menu
} tmenu;


#endif /* Display_H_ */
