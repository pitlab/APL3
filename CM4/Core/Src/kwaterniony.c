//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Biblioteka obliczeń na kwaternionach
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "kwaterniony.h"
//Lektura obowiązkowa:
//https://youtu.be/HMDb9zJ2BdA?si=NFm44BkfRmKqpYiF - dr. Tomasz Miller Historia i wyjaśnienie właściwości kwaternionów
//https://youtu.be/ZgOmCYfw6os?si=fDKRwn95x-n1le50 - Mateusz Kowalski - Algortym obrotu w przestrzeni 3D przy użyciu kwaternionów
//https://youtu.be/j5m71_TaQwE?si=XlQSB3M4NHd1mqvY - Mateusz Kowalski - Macierz obrotu wokół dowolnej osi o dowolny kąt



// Obliczenia obrotu wektora grawitacji o kąt z żyroskopów na podstawie instrukcji Mateusza Kowalskiego: https://youtu.be/XjFq3Slo2wo?si=5JBCD7lqXvaLayjN
// Trzeba utworzyć kwaterniony q i v gdzie v będzie zawierał współrzędne obracanego wektora a q będzie definiował obrót i jest kwaternionem jednostkowym
// Następnie trzeba wykonać mnożenie q * v * q* gdzie q* jest kwaternionem sprzężonym.
// Kwaternion sprzeżony ma części urojone ze znakiem minus. Jeżeli q = s + xi + yj + zk to q* = s - xi - yj - zk
// Mnożenie przez kwaternion sprzężony wykonuje się po to aby pozbyć się części rzeczywistej, której nie potrafimy zinterpretować gdyż nasz wektor siedzi w części urojonej.
// mnożenie przez q i następnie przez sprzezone q* spowoduje wyzerowanie części rzeczywistej. Skutkiem ubocznym jest obrót o podwójny kąt, dlatego obracamy o połowę kąta
// Wektorem obracamym jest wektor przyspieszenie ziemskiego, będący odczytem początkowym z akcelerometru. Wektor nie musi być znormalizowany więc jest to po prostu uDaneCM4.dane.fAkcel1 lub uDaneCM4.dane.fAkcel2
// Mnożenie kwaternionów wykonujemy w postaci algebraicznej a podstawianie danych w postaci trygonometrycznej, więc potrzebne jest przejscie miedzy tymi dwiema formami
// Przejscie z formy trygonometrycznej na algebraiczną:
//	q = |q| * (cos fi + (uxi + uyj + uzk) * sin fi)  ->  q = s + xi + yj + zk
//	s = |q| * cos fi
//	x = |q| * ux * sin fi
//	y = |q| * uy * sin fi
//	z = |q| * uz * sin fi
// Przejście z formy algebraicznej na trygonometryczną:
//	q = s + xi + yj + zk  ->  q = |q| * (cos fi + (uxi + uyj + uzk) * sin fi)
//	|q| = sqrt(s^2 + x^2 + y^2 + z^2)
//	ux = x / sqrt(x^2 + y^2 + z^2)		//bez s^2
//	uy = y / sqrt(x^2 + y^2 + z^2)
//	uz = z / sqrt(x^2 + y^2 + z^2)
//	cos fi = s / sqrt(s^2 + x^2 + y^2 + z^2)
//	sin fi = sqrt(x^2 + y^2 + z^2) / sqrt(s^2 + x^2 + y^2 + z^2)
//	fi = arcos(s / sqrt(s^2 + x^2 + y^2 + z^2))
//	Jeżeli przyjmiemy że moduł z q: |q| = 1 to mamy uproszczenie:
//	cos fi = s
//	sin fi = sqrt(x^2 + y^2 + z^2)
//	fi = arcos(s)




////////////////////////////////////////////////////////////////////////////////
// Wzór na mnożenie kwaternionów (q0 + iq1 + jq2 + kq3) * (p0 + ip1 + jp2 + kp3)
// (q0p0 - q1p1 - q2p2 + q3p3)
// (q1p0 + q0p1 - q3p2 + q2p3)
// (q2p0 + q3p1 + q0p2 - q1p3)
// (q3p0 - q2p1 + q1p2 + q0p3)
// Źródło: https://ahrs.readthedocs.io/en/latest/filters/aqua.html
// Parametry: *q i *p - wskaźniki na mnożone kwaterniony
// *wynik - wskaźnik na kwaternion będący wynikiem mnożenia
// Zwraca: nic
// Czas trwania: 1,05us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void MnozenieKwaternionow(float *q, float *p, float *wynik)
{
	*(wynik + 0) = *(q+0) * *(p+0) - *(q+1) * *(p+1) - *(q+2) * *(p+2) - *(q+3) * *(p+3);
	*(wynik + 1) = *(q+0) * *(p+1) + *(q+1) * *(p+0) + *(q+2) * *(p+3) - *(q+3) * *(p+2);
	*(wynik + 2) = *(q+0) * *(p+2) - *(q+1) * *(p+3) + *(q+2) * *(p+0) + *(q+3) * *(p+1);
	*(wynik + 3) = *(q+0) * *(p+3) + *(q+1) * *(p+2) - *(q+2) * *(p+1) + *(q+3) * *(p+0);
}

////////////////////////////////////////////////////////////////////////////////
// Przepisanie kwaternionu z postaci algebraicznaj do macierzowej na podstawie: https://www.youtube.com/watch?v=ZgOmCYfw6os&t=1621s&ab_channel=MateuszKowalski
// Parametry:
// [we] *q - wskaźnik na kwaternion q = (q0 + iq1 + jq2 + kq3)
// [wy] *m - wskaźnik na macierz m[4 wiersze]x[4 kolumny]
// Zwraca: nic
// Czas trwania: 1,065us na 200MHz
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
// Czas trwania: 0,379us na 200MHz
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
// Czas trwania: 5,8us na 200MHz
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



////////////////////////////////////////////////////////////////////////////////
// Obrót wektora wokół osi i o kąt opisane przez kwaternion obortu v
// Na podstawie https://youtu.be/ZgOmCYfw6os?si=W9FL7V7r92M1e8Zf
// Parametry:
// [we] *wektor_we - wskaźnik na wektor wejsciowy [x, y, z],
// [we] *v - wskaźnik na kwaternion obrotu v = (v0 + ivx + jvy + kvz)
// [wy] *wektor_we - wskaźnik na wektor po obrocie [x', y', z'],
// Zwraca: nic
// Czas trwania: 2,135us na 200MHz
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
// Czas trwania: 0,364us na 200MHz
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
// Czas trwania: 0,329us na 200MHz
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
// [wy] *sprzezony - wskaźnik na kwaternion sprzężony z wejściowym
// Zwraca: nic
// Czas trwania: 0,790us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KwaternionSprzezony(float *q, float *sprzezony)
{
	*(sprzezony+0) = *(q+0);
	for (uint8_t n=1; n<4; n++)
		*(sprzezony+n) = *(q+n) * -1.0f;
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza kąty z wektora podanego jako kwaternion: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions - Quaternion → Euler angles (z-x-z extrinsic - czyli zewnętrzny ukłąd odniesienia)
// phi   = atan2((qi*qk + qj*qr), -(qj*qk - qi*qr))
// theta = arccos(-qi^2 -qj^2 + qk^2 + qr^2)
// psi   = atan2((qi*qk - qj*qr), (qj*qk + qi*qr))
// Parametry:
// [we] *qA - wskaźnik na kwaternion wektora przyspieszenia A = (q0 + iqx + jqy + kqz)
// [we] *qM - wskaźnik na kwaternion wektora magnetycznego M = (q0 + iqx + jqy + kqz)
// [wy] *katy - wskaźnik na zmienną z 3 kątami [Phi, Theta, Psi]
// Zwraca: nic
// Czas trwania: 2,33us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KatyKwaterniona1(float *qA, float *qM, float *fKaty)
{
	*(fKaty+0) = atan2f((*(qA+1) * *(qA+3) + *(qA+2) * *(qA+0)), -1*(*(qA+2) * *(qA+3) - *(qA+1) * *(qA+0)));
	*(fKaty+1) = acosf((*(qA+0) * *(qA+0)) - (*(qA+1) * *(qA+1)) - (*(qA+2) * *(qA+2)) + (*(qA+3) * *(qA+3)));
	*(fKaty+2) = atan2f((*(qM+1) * *(qM+3) - *(qM+2) * *(qM+0)), (*(qM+2) * *(qM+3) + *(qM+1) * *(qM+0)));
}




////////////////////////////////////////////////////////////////////////////////
// Oblicza kąty z wektora podanego jako kwaternion [qr, qi, qj, qk]: https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions - Quaternion → Euler angles (z-y′-x″ intrinsic - czyli lokalny układ odniesienia)
// Wymaga aby wektory były znormalizowane, bo acosf() przyjmuje tylko liczby z zakresu -1..1
// phi   = atan2(2*(qr*qi + qj*qk), 1-2*(qi^2 + qj^2))
// theta = arccos(2*(qr * qj - qk * qi)
// psi   = atan2(2*(qr*qk + qi*qj), 1-2*(qj^2 + qk^2))
// Parametry:
// [we] *qA - wskaźnik na kwaternion wektora przyspieszenia A = (q0 + iqx + jqy + kqz)
// [we] *qM - wskaźnik na kwaternion wektora magnetycznego M = (q0 + iqx + jqy + kqz)
// [wy] *katy - wskaźnik na zmienną z 3 kątami [Phi, Theta, Psi]
// Zwraca: nic
// Czas trwania: us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KatyKwaterniona2(float *qA, float *qM, float *fKaty)
{
	*(fKaty+0) = atan2f(2 * (*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - 2 * (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2)));
	*(fKaty+1) = acosf (2 * (*(qA+0) * *(qA+2) - *(qA+3) * *(qA+1)));
	*(fKaty+2) = atan2f(2 * (*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - 2 * (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3)));
}
