//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Biblioteka bbliczeń na kwaternionach
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "kwaterniony.h"
//Lektura obowiązkowa:
//https://youtu.be/HMDb9zJ2BdA?si=NFm44BkfRmKqpYiF
//https://youtu.be/ZgOmCYfw6os?si=fDKRwn95x-n1le50


//Ten i poniższy wzór daja takie same wyniki i zajmują tyle samo czasu procesora. Różnią się formatem podawanych danych
////////////////////////////////////////////////////////////////////////////////
// Wzór na mnożenie kwaternionów (a1 + ib1 + jc1 + kd1) * (a2 + ib2 + jc2 + kd2) =
// = (a1a2 - b1b2 - c1c2 -d1d2)
// +i(a1b2 + b1a2 + c1d2 - d1c2)
// +j(a1c2 + c1a2 + d1b2 - b1d2)
// +k(a1d2 + d1a2 + b1c2 - c1b2)
// Na podstawie: https://www.youtube.com/watch?v=HMDb9zJ2BdA&ab_channel=Copernicus
// Parametry: (abcd)1 i (abcd)2 - elementy mnożonych kwaternionów
// *wynik - wskaźnik na kwaternion będący wynikiem mnożenia
// Zwraca: nic
// Czas trwania: 1,05us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void MnozenieKwaternionow1(float a1, float b1, float c1, float d1, float a2, float b2, float c2, float d2, float *wynik)
{
	*(wynik + 0) = a1*a2 - b1*b2 - c1*c2 - d1*d2;
	*(wynik + 1) = a1*b2 + b1*a2 + c1*d2 - d1*c2;
	*(wynik + 2) = a1*c2 + c1*a2 + d1*b2 - b1*d2;
	*(wynik + 3) = a1*c2 + c1*a2 + d1*b2 - b1*d2;
}



// Dla porównania bardziej czytelny wzór, łatwiejszy do implemntacji na wskaźnikach. Mnożenie q*p =
////////////////////////////////////////////////////////////////////////////////
// Wzór na mnożenie kwaternionów (q0 + iq1 + jq2 + kq3) * (p0 + ip1 + jp2 + kp3)
// (q0p0 - q1p1 - q2p2 + q3p3)
// (q1p0 + q0p1 - q3p2 + q2p3)
// (q2p0 + q3p1 + q0p2 - q1p3)
// (q3p0 - q2p1 + q1p2 + q0p3)
// Źródło: https://www.youtube.com/watch?v=VNIszCO4azs&ab_channel=ngoduong
// Parametry: *q i *p - wskaźniki na mnożone kwaterniony
// *wynik - wskaźnik na kwaternion będący wynikiem mnożenia
// Zwraca: nic
// Czas trwania: 1,05us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void MnozenieKwaternionow2(float *q, float *p, float *wynik)
{
	*(wynik + 0) = *(q+0) * *(p+0) - *(q+1) * *(p+1) - *(q+2) * *(p+2) - *(q+3) * *(p+3);
	*(wynik + 1) = *(q+1) * *(p+0) + *(q+0) * *(p+1) + *(q+3) * *(p+2) - *(q+2) * *(p+3);
	*(wynik + 2) = *(q+2) * *(p+0) + *(q+3) * *(p+1) + *(q+0) * *(p+2) - *(q+1) * *(p+3);
	*(wynik + 3) = *(q+3) * *(p+0) + *(q+2) * *(p+1) + *(q+1) * *(p+2) - *(q+0) * *(p+3);
}



////////////////////////////////////////////////////////////////////////////////
// Przepisanie kwaternionu z postaci algebraicznaj do macierzowej na podstawie: https://www.youtube.com/watch?v=ZgOmCYfw6os&t=1621s&ab_channel=MateuszKowalski
// Parametry:
// [we] *q - wskaźnik na kwaternion q = (q0 + iq1 + jq2 + kq3)
// [wy] *m - wskaźnik na macierz m[4 wiersze]x[4 kolumny]
// Zwraca: nic
// Czas trwania: ?us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KwaternionNaMacierz(float *q, float *m)
{
	*(m+0*4+0) = *(q+0);		//wiersz 1, kol 1 = s
	*(m+0*4+1) = *(q+1);		//wiersz 1, kol 2 = x
	*(m+0*4+2) = *(q+2);		//wiersz 1, kol 3 = y
	*(m+0*4+3) = *(q+3);		//wiersz 1, kol 4 = z
	*(m+1*4+0) = *(q+1) * -1;	//wiersz 2, kol 1 = -x
	*(m+1*4+1) = *(q+0);		//wiersz 2, kol 2 = s
	*(m+1*4+2) = *(q+3) * -1;	//wiersz 2, kol 3 = -z
	*(m+1*4+3) = *(q+2);		//wiersz 2, kol 4 = y
	*(m+2*4+0) = *(q+2) * -1;	//wiersz 3, kol 1 = -y
	*(m+2*4+1) = *(q+3);		//wiersz 3, kol 2 = z
	*(m+2*4+2) = *(q+0);		//wiersz 3, kol 3 = s
	*(m+2*4+3) = *(q+1) * -1;	//wiersz 3, kol 4 = -x
	*(m+3*4+0) = *(q+3) * -1;	//wiersz 4, kol 1 = -z
	*(m+3*4+1) = *(q+2) * -1;	//wiersz 3, kol 2 = -y
	*(m+3*4+2) = *(q+1);		//wiersz 1, kol 3 = x
	*(m+3*4+3) = *(q+0);		//wiersz 3, kol 4 = s
}



////////////////////////////////////////////////////////////////////////////////
// Przepisanie kwaternionu z postaci macierzowej do algebraicznaj na podstawie: https://www.youtube.com/watch?v=ZgOmCYfw6os&t=1621s&ab_channel=MateuszKowalski
// Parametry:
// [we] *m - wskaźnik na macierz m[4 wiersze]x[4 kolumny]
// [wy] *q - wskaźnik na kwaternion q = (q0 + iq1 + jq2 + kq3)
// Zwraca: nic
// Czas trwania: ?us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void MacierzNaKwaternion(float  *m, float *q)
{
	*(q+0) = *(m+0*4+0);		//wiersz 1, kol 1 = s
	*(q+1) = *(m+0*4+1);		//wiersz 1, kol 2 = x
	*(q+2) = *(m+0*4+2);		//wiersz 1, kol 3 = y
	*(q+3) = *(m+0*4+3);		//wiersz 1, kol 4 = z
}

////////////////////////////////////////////////////////////////////////////////
// Mnożenie macierzy 4x4. Wiersz pierwszej mnożymy razy kolumnę drugiej
// Parametry:
// [we] *a - wskaźnik na macierz A
// [we] *b - wskaźnik na macierz B
// [wy] *m - wskaźnik na macierz m[4 wiersze]x[4 kolumny]
// Zwraca: nic
// Czas trwania: ?us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void MnozenieMacierzy4x4(float *a, float *b, float *m)
{
	*(m+0*4+0) = *(a+0*4+0) * *(b+0*4+0) + *(a+0*4+1) * *(b+1*4+0) + *(a+0*4+2) * *(b+2*4+0) + *(a+0*4+3) * *(b+3*4+0);	//wiersz 1, kol 1 = 1 wiersz A * 1 kolumna B
	*(m+0*4+1) = *(a+0*4+0) * *(b+0*4+1) + *(a+0*4+1) * *(b+1*4+1) + *(a+0*4+2) * *(b+2*4+1) + *(a+0*4+3) * *(b+3*4+1);	//wiersz 1, kol 2
	*(m+0*4+2) = *(a+0*4+0) * *(b+0*4+2) + *(a+0*4+1) * *(b+1*4+2) + *(a+0*4+2) * *(b+2*4+2) + *(a+0*4+3) * *(b+3*4+2);	//wiersz 1, kol 3
	*(m+0*4+3) = *(a+0*4+0) * *(b+0*4+3) + *(a+0*4+1) * *(b+1*4+3) + *(a+0*4+2) * *(b+2*4+3) + *(a+0*4+3) * *(b+3*4+3);	//wiersz 1, kol 4
	*(m+1*4+0) = *(a+1*4+0) * *(b+0*4+0) + *(a+1*4+1) * *(b+1*4+0) + *(a+1*4+2) * *(b+2*4+0) + *(a+1*4+3) * *(b+3*4+0);	//wiersz 2, kol 1
	*(m+1*4+1) = *(a+1*4+0) * *(b+0*4+1) + *(a+1*4+1) * *(b+1*4+1) + *(a+1*4+2) * *(b+2*4+1) + *(a+1*4+3) * *(b+3*4+1);	//wiersz 2, kol 2
	*(m+1*4+2) = *(a+1*4+0) * *(b+0*4+2) + *(a+1*4+1) * *(b+1*4+2) + *(a+1*4+2) * *(b+2*4+2) + *(a+1*4+3) * *(b+3*4+2);	//wiersz 2, kol 3
	*(m+1*4+3) = *(a+1*4+0) * *(b+0*4+3) + *(a+1*4+1) * *(b+1*4+3) + *(a+1*4+2) * *(b+2*4+3) + *(a+1*4+3) * *(b+3*4+3);	//wiersz 2, kol 4
	*(m+2*4+0) = *(a+2*4+0) * *(b+0*4+0) + *(a+2*4+1) * *(b+1*4+0) + *(a+2*4+2) * *(b+2*4+0) + *(a+2*4+3) * *(b+3*4+0);	//wiersz 3, kol 1
	*(m+2*4+1) = *(a+2*4+0) * *(b+0*4+1) + *(a+2*4+1) * *(b+1*4+1) + *(a+2*4+2) * *(b+2*4+1) + *(a+2*4+3) * *(b+3*4+1);	//wiersz 3, kol 2
	*(m+2*4+2) = *(a+2*4+0) * *(b+0*4+2) + *(a+2*4+1) * *(b+1*4+2) + *(a+2*4+2) * *(b+2*4+2) + *(a+2*4+3) * *(b+3*4+2);	//wiersz 3, kol 3
	*(m+2*4+3) = *(a+2*4+0) * *(b+0*4+3) + *(a+2*4+1) * *(b+1*4+3) + *(a+2*4+2) * *(b+2*4+3) + *(a+2*4+3) * *(b+3*4+3);	//wiersz 3, kol 4
	*(m+3*4+0) = *(a+3*4+0) * *(b+0*4+0) + *(a+3*4+1) * *(b+1*4+0) + *(a+3*4+2) * *(b+2*4+0) + *(a+3*4+3) * *(b+3*4+0);	//wiersz 4, kol 1
	*(m+3*4+1) = *(a+3*4+0) * *(b+0*4+0) + *(a+3*4+1) * *(b+1*4+0) + *(a+3*4+2) * *(b+2*4+0) + *(a+3*4+3) * *(b+3*4+0);	//wiersz 3, kol 2
	*(m+3*4+2) = *(a+3*4+0) * *(b+0*4+0) + *(a+3*4+1) * *(b+1*4+0) + *(a+3*4+2) * *(b+2*4+0) + *(a+3*4+3) * *(b+3*4+0);	//wiersz 1, kol 3
	*(m+3*4+3) = *(a+3*4+0) * *(b+0*4+0) + *(a+3*4+1) * *(b+1*4+0) + *(a+3*4+2) * *(b+2*4+0) + *(a+3*4+3) * *(b+3*4+0);	//wiersz 3, kol 4
}

//[wiersz][kolumna]
/*void MnozenieMacierzy4x4(float a[4][4], float b[4][4], float (*m)[4][4])
{
	*m[0][0] = a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0] + a[0][3] * b[3][0];	//wiersz 1, kol 1 = wiersz 1A * kolumna 1B
	*m[0][1] = a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1] + a[0][3] * b[3][1];	//wiersz 1, kol 2
	*m[0][2] = a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2] + a[0][3] * b[3][2];	//wiersz 1, kol 3
	*m[0][1] = a[0][0] * b[0][3] + a[0][1] * b[1][3] + a[0][2] * b[2][3] + a[0][3] * b[3][3];	//wiersz 1, kol 4

	*m[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0] + a[1][3] * b[3][0];	//wiersz 2, kol 1
	*m[1][1] = a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1] + a[1][3] * b[3][1];	//wiersz 2, kol 2
	*m[1][2] = a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2] + a[1][3] * b[3][2];	//wiersz 2, kol 3
	*m[1][3] = a[1][0] * b[0][3] + a[1][1] * b[1][3] + a[1][2] * b[2][3] + a[1][3] * b[3][3];	//wiersz 2, kol 4

	*m[2][0] = a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0] + a[2][3] * b[3][0];	//wiersz 3, kol 1
	*m[2][1] = a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1] + a[2][3] * b[3][1];	//wiersz 3, kol 2
	*m[2][2] = a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2] + a[2][3] * b[3][2];	//wiersz 3, kol 3
	*m[2][3] = a[2][0] * b[0][3] + a[2][1] * b[1][3] + a[2][2] * b[2][3] + a[2][3] * b[3][3];	//wiersz 3, kol 4

	*m[3][0] = a[3][0] * b[0][0] + a[3][1] * b[1][0] + a[3][2] * b[2][0] + a[3][3] * b[3][0];	//wiersz 4, kol 1
	*m[3][1] = a[3][0] * b[0][1] + a[3][1] * b[1][1] + a[3][2] * b[2][1] + a[3][3] * b[3][1];	//wiersz 4, kol 2
	*m[3][2] = a[3][0] * b[0][2] + a[3][1] * b[1][2] + a[3][2] * b[2][2] + a[3][3] * b[3][2];	//wiersz 4, kol 3
	*m[3][3] = a[3][0] * b[0][3] + a[3][1] * b[1][3] + a[3][2] * b[2][3] + a[3][3] * b[3][3];	//wiersz 4, kol 4
}*/


////////////////////////////////////////////////////////////////////////////////
// Obrót wektora wokół osi i o kąt opisane przez kwaternion obortu v
// Na podstawie https://youtu.be/ZgOmCYfw6os?si=W9FL7V7r92M1e8Zf
// Parametry:
// [we] *wektor_we - wskaźnik na wektor wejsciowy [x, y, z],
// [we] *v - wskaźnik na kwaternion obrotu v = (v0 + ivx + jvy + kvz)
// [wy] *wektor_we - wskaźnik na wektor po obrocie [x', y', z'],
// Zwraca: nic
// Czas trwania: ?us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void ObrotWektoraKwaternionem(float *wektor_we, float *v, float *wektor_wy)
{
	//Macierz obrotu należy pomnożyć przez obracany punkt zapisany jako wektor [Px, Py, Pz] w kolumnie
	//[2*(s0^2 + x0^2) - 1		2*(x0*y0 - s0*z0)		2*(s0*y0 + x0*z0)]		[Px]
	//[2*(s0*z0 + x0*y0)		2*(s0^2 + y0^2) - 1		2*(y0*z0 - s0*x0)]	* 	[Py]
	//[2*(x0*z0 - s0*y0)		2*(s0*x0 + y0*z0)		2*(s0^2 + z0^2) - 1]	[Pz]

	//mnożemie macierzy przez wektor odpowiada mnożeniu kolumn macierzy przez składowe wektora
	 *(wektor_wy+0) = *(wektor_we+0) * (2*(*(v+0) * *(v+0) + *(v+1) * *(v+1)) - 1) 	+ *(wektor_we+1) * 2*(*(v+1) * *(v+2) - *(v+0) * *(v+3)) 			+ *(wektor_we+2) * 2*(*(v+0) * *(v+2) + *(v+1) * *(v+3));
	 *(wektor_wy+1) = *(wektor_we+0) * 2*(*(v+0) * *(v+3) + *(v+1) * *(v+2)) 		+ *(wektor_we+1) * (2*(*(v+0) * *(v+0) + *(v+2) * *(v+2)) - 1) 		+ *(wektor_we+2) * 2*(*(v+2) * *(v+3) - *(v+0) * *(v+1));
	 *(wektor_wy+2) = *(wektor_we+0) * 2*(*(v+1) * *(v+3) - *(v+0) * *(v+2)) 		+ *(wektor_we+1) * 2*(*(v+0) * *(v+1) + *(v+2) * *(v+3)) 			+ *(wektor_we+2) * (2*(*(v+0) * *(v+0) + *(v+3) * *(v+3)) - 1);
}



////////////////////////////////////////////////////////////////////////////////
// Wstawia wektor do kwaternionu
// Parametry:
// [we] *wektor - wskaźnik na wektor wejsciowy [x, y, z],
// [wy] *q - wskaźnik na kwaternion w = (q0 + iqx + jqy + kqz)
// Zwraca: nic
// Czas trwania: ?us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void WektorNaKwaternion(float *wektor, float *q)
{
	*(q+0) = 0;
	*(q+1) = *(wektor+0);
	*(q+2) = *(wektor+1);
	*(q+3) = *(wektor+2);
}



////////////////////////////////////////////////////////////////////////////////
// Wyodrębnia wektor z kwaternionu
// Parametry:
// [we] *q - wskaźnik na kwaternion w = (q0 + iqx + jqy + kqz)
// [wy] *wektor - wskaźnik na wektor wejsciowy [x, y, z],
// Zwraca: nic
// Czas trwania: ?us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KwaternionNaWektor(float *q, float *wektor)
{
	*(wektor+0) = *(q+1);
	*(wektor+1) = *(q+2);
	*(wektor+2) = *(q+3);
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza kwaternion sprzężony
// Parametry:
// [we] *q - wskaźnik na kwaternion w = (q0 + iqx + jqy + kqz)
// [wy] *sprzezony - wskaźnik na kwaternion sprzęzony z wejsciowym
// Zwraca: nic
// Czas trwania: ?us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KwaternionSprzezony(float *q, float *sprzezony)
{
	*(sprzezony+0) = *(q+0);
	for (uint8_t n=1; n<4; n++)
		*(sprzezony+n) = *(q+n) * -1.0f;
}
