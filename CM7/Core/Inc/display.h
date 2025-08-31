


#ifndef Display_H_
#define Display_H_



#include "sys_def_CM7.h"
//definicje kolorów: https://rgbcolorpicker.com/565
#define	BLACK   	0x0000
#define	CZARNY   	0x0000
#define	BLUE    	0x001F
#define	NIEBIESKI  	0x001F
#define	RED     	0xF800
#define	CZERWONY   	0xF800
#define	GREEN   	0x07E0
#define	ZIELONY   	0x07E0
#define CYJAN    	0x07FF
#define CYAN    	0x07FF
#define MAGENTA 	0xF81F
#define ZOLTY	  	0xFFE0
#define YELLOW  	0xFFE0
//#define ORANGE		0xFDA0
#define POMARANCZ	0xFDA0
#define WHITE 		0xFFFF
#define BIALY 		0xFFFF
#define LIBELLA1	0xAEAA	//płyn
#define LIBELLA2	0x95EA	//obwódka bąbelka
#define LIBELLA3	0xCFAA	//bąbelek
#define GRAY10 		0x18C3	//najciemniejszy
#define GRAY20 		0x3186
#define GRAY30 		0x4A69
#define GRAY40 		0x632C
#define GRAY50 		0x7BEF
#define GRAY60 		0x9CD3
#define GRAY70 		0xB596
#define GRAY80 		0xCE79
#define TEAL		0x0410	//ciemny zgniłozielony


#define	KOLOR_X    	0xFA8A	//FF5151
#define	KOLOR_Y	   	0x07E0
#define	KOLOR_Z   	0x7C7F	//7E8EFE

#define MENU_TLO_BAR	GRAY40	//kolor belki menu górnego
#define MENU_TLO_WYB	GRAY70	//pozycja menu edytowana lub uruchamiana
#define MENU_TLO_AKT	GRAY50	//pozycja menu podświetlona
#define MENU_TLO_NAK	GRAY30	//pozycja nieaktywna

#define MENU_RAM_WYB	GREEN
#define MENU_RAM_AKT	WHITE
#define MENU_RAM_NAK	GRAY50
#define TRANSPARENT		0xFFFE	//specjalny kolor tła



//definicje orientacji
#define POZIOMO 0
#define PIONOWO 1
#define LEFT 0
#define RIGHT 9999
#define CENTER 9998

//rozmiar ekranu
#define DISP_X_SIZE		480
#define DISP_Y_SIZE		320
#define DISP_VX_SIZE	320		//wertykalnie
#define DISP_VY_SIZE	480

#define SZER_WYKR_MAG	260
#define PROMIEN_RZUTU_MAGN	90


#define FONT_SH		12	//wysokośc małej czcionki
#define FONT_BH		16	//wysokośc dużej czcionki
#define FONT_SL		8	//szerokość małej czcionki
#define FONT_BL		16	//szerokość dużej czcionki
#define FONT_SLEN	8	//szerokość małej czcionki
#define FONT_BLEN	16	//szerokość dużej czcionki
#define UP_SPACE	2	//wolna przestrzeń nad tekstem
#define DW_SPACE	4	//wolna przestrzeń pod tekstem

//definicje menu głównego
#define MENU_WIERSZE	2								//liczba rzędów menu
#define MENU_KOLUMNY	5								//liczba kolumn
#define MENU_ICO_WYS	62										//wysokość ikonki
#define MENU_ICO_DLG	66										//szerokość ikonki
#define MENU_NAG_WYS	18										//wyskość nagłówka
#define MENU_OPIS_WYS	8										//wyskość opisu ikon
#define MENU_PASOP_WYS	(FONT_SH+2*UP_SPACE)					// wysokość paska opisu funkcji


#define WYS_PASKA_POSTEPU	5

//definicje polskich znaków dla MidFont
#define ą	127
#define ć	128
#define ę	129
#define ł	130
#define ń	131
#define ó	132
#define ś	133
#define ż	134
#define ź	135
#define ZNAK_STOPIEN	139

typedef struct current_font
{
	unsigned char* font;
	unsigned char x_size;
	unsigned char y_size;
	unsigned char offset;
	unsigned char numchars;
} current_font_t;


//menu obrazkowe na środku ekranu
typedef struct tmenu
{
	const char* chOpis;				//wskaźnik na opis pod menu
	const char* chPomoc;			//wskaźnik na tekst pomocy wyświetlany w pasku statusu
	unsigned char chMode;			//numer trybu pracy obsługujący funkcję
	const unsigned short* sIkona;	//wskaźnika na ikonę menu
} tmenu;


#define BABEL			24		//promień bąbelka powietrza w libelli
#define LIBELLA_BOK		230		//szerokość i wysokość libelli



#endif /* Display_H_ */
