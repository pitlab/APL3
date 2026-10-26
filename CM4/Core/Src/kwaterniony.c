//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Biblioteka obliczeń na kwaternionach
//
// (c) PitLab 2025
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Kwaterniony.h>
#include <Wspolne.h>
//Lektura obowiązkowa:
//https://youtu.be/HMDb9zJ2BdA?si=NFm44BkfRmKqpYiF - dr. Tomasz Miller - Historia i wyjaśnienie właściwości kwaternionów
//https://youtu.be/ZgOmCYfw6os?si=fDKRwn95x-n1le50 - Mateusz Kowalski - Algortym obrotu w przestrzeni 3D przy użyciu kwaternionów
//https://youtu.be/j5m71_TaQwE?si=XlQSB3M4NHd1mqvY - Mateusz Kowalski - Macierz obrotu wokół dowolnej osi o dowolny kąt


//stKatyRozszerz stKatyRozszerz = {0};


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
// Przepisanie kwaternionu z postaci macierzowej do algebraicznej na podstawie: https://www.youtube.com/watch?v=ZgOmCYfw6os&t=1621s&ab_channel=MateuszKowalski
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
	*(m+3*4+1) = *(a+3*4+0) * *(b+0*4+0) + *(a+3*4+1) * *(b+1*4+0) + *(a+3*4+2) * *(b+2*4+0) + *(a+3*4+3) * *(b+3*4+0);	//wiersz 4, kol 2
	*(m+3*4+2) = *(a+3*4+0) * *(b+0*4+0) + *(a+3*4+1) * *(b+1*4+0) + *(a+3*4+2) * *(b+2*4+0) + *(a+3*4+3) * *(b+3*4+0);	//wiersz 4, kol 3
	*(m+3*4+3) = *(a+3*4+0) * *(b+0*4+0) + *(a+3*4+1) * *(b+1*4+0) + *(a+3*4+2) * *(b+2*4+0) + *(a+3*4+3) * *(b+3*4+0);	//wiersz 4, kol 4
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
/*void KatyKwaterniona1(float *qA, float *qM, float *fKaty)
{
	*(fKaty+0) = atan2f((*(qA+1) * *(qA+3) + *(qA+2) * *(qA+0)), -1*(*(qA+2) * *(qA+3) - *(qA+1) * *(qA+0)));
	*(fKaty+1) = acosf((*(qA+0) * *(qA+0)) - (*(qA+1) * *(qA+1)) - (*(qA+2) * *(qA+2)) + (*(qA+3) * *(qA+3)));
	*(fKaty+2) = atan2f((*(qM+1) * *(qM+3) - *(qM+2) * *(qM+0)), (*(qM+2) * *(qM+3) + *(qM+1) * *(qM+0)));
}*/




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
/*void KatyKwaterniona2(float *qA, float *qM, float *fKaty)
{
	*(fKaty+0) = atan2f(2 * (*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - 2 * (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2)));
	*(fKaty+1) = acosf (2 * (*(qA+0) * *(qA+2) - *(qA+3) * *(qA+1)));
	*(fKaty+2) = atan2f(2 * (*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - 2 * (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3)));
}*/



////////////////////////////////////////////////////////////////////////////////
// Oblicza kąty Eulera: Phi, Theta, Psi z wektora podanego jako kwaternion [qr, qi, qj, qk]: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// Wymaga aby wektory były znormalizowane, bo asinf() przyjmuje tylko liczby z zakresu -1..1
// Parametry:
// [we] *qA - wskaźnik na kwaternion wektora przyspieszenia A = (q0 + iqx + jqy + kqz)
// [we] *qM - wskaźnik na kwaternion wektora magnetycznego M = (q0 + iqx + jqy + kqz)
// [wy] *katy - wskaźnik na zmienną z 3 kątami [Phi, Theta, Psi]
// Zwraca: nic
// Czas trwania: 4,52us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KatyKwaterniona(float *qA, float *qM, float *fKaty)
{
	//float a, b;

	//*(fKaty+0) = atan2f(2 * (*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - 2 * (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2))); 	//zwraca podwójny kąt z odwrotnym znakiem
	//a = 2 * (*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3));
	//b = 1 - 2 * (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2));
	//*(fKaty+0) = atan2f(a, b) / 2;
	//*(fKaty+0) = atan2f((*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2)));	//zwraca pojedynczy kąt z odwrotnym znakiem
	*(fKaty+0) = -atan2f((*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2)));	//zwraca pojedynczy kąt z właściwym znakiem

	//*(fKaty+1) = asinf (2 * (*(qA+0) * *(qA+2) - *(qA+1) * *(qA+3)));		//zwraca podwójny kąt z odwrotnym znakiem
	//a = 2 * (*(qA+0) * *(qA+2) - *(qA+1) * *(qA+3));
	//*(fKaty+1) = asinf (a) / 2;
	//*(fKaty+1) = asinf ((*(qA+0) * *(qA+2) - *(qA+1) * *(qA+3)));		//zwraca pojedynczy kąt z odwrotnym znakiem
	*(fKaty+1) = -asinf ((*(qA+0) * *(qA+2) - *(qA+1) * *(qA+3)));		//zwraca pojedynczy kąt z właściwym znakiem

	//*(fKaty+2) = atan2f(2 * (*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - 2 * (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3)));		//zwraca podwójny kąt z odwrotnym znakiem
	//a = 2 * (*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2));
	//b = 1 - 2 * (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3));
	//*(fKaty+2) = atan2f(a, b) / 2;
	//*(fKaty+2) = atan2f((*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3)));		//zwraca pojedynczy kąt z odwrotnym znakiem
	*(fKaty+2) = -atan2f((*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3)));		//zwraca pojedynczy kąt z właściwym znakiem
}


////////////////////////////////////////////////////////////////////////////////
// Oblicza kąty Eulera: Phi, Theta, Psi z wektora podanego jako kwaternion [qr, qi, qj, qk]: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// Wymaga aby wektory były znormalizowane, bo używa sqrtf(1-...)
// Parametry:
// [we] *qA - wskaźnik na kwaternion wektora przyspieszenia A = (q0 + iqx + jqy + kqz)
// [we] *qM - wskaźnik na kwaternion wektora magnetycznego M = (q0 + iqx + jqy + kqz)
// [wy] *katy - wskaźnik na zmienną z 3 kątami [Phi, Theta, Psi]
// Zwraca: nic
// Czas trwania: 7,8us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KatyKwaterniona2(float *qA, float *qM, float *fKaty)
{
	float a;//, b;

	//*(fKaty+0) = atan2f(2 * (*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - 2 * (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2)));	//zwraca podwójny kąt z odwrotnym znakiem
	//a = 2 * (*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3));
	//b = 1 - 2 * (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2));
	//*(fKaty+0) = atan2f(a, b) / 2;
	*(fKaty+0) = atan2f((*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2)));		//zwraca pojedynczy kąt z odwrotnym znakiem

	//wersja na atan2 niewymagająca normalizacji
	a = 2 * (*(qA+0) * *(qA+2) - *(qA+1) * *(qA+3));
	//*(fKaty+1) = 2 * atan2f(sqrtf(1+a), sqrtf(1-a)) - M_PI/2;
	//*(fKaty+1) = (2 * atan2f(sqrtf(1+a), sqrtf(1-a)) - M_PI/2) / 2;
	*(fKaty+1) = atan2f(sqrtf(1+a), sqrtf(1-a)) - M_PI/4;		//zwraca pojedynczy kąt z odwrotnym znakiem

	//*(fKaty+2) = atan2f(2 * (*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - 2 * (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3)));		//zwraca podwójny kąt z odwrotnym znakiem
	//a = 2 * (*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2));
	//b = 1 - 2 * (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3));
	//*(fKaty+2) = atan2f(a, b) / 2;
	*(fKaty+2) = atan2f((*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3)));	//zwraca pojedynczy kąt z odwrotnym znakiem
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza kąty Eulera: Phi, Theta, Psi z wektora podanego jako kwaternion [qr, qi, qj, qk] na podstawie wzorów:
//yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
//pitch = asin(2 * (w * y - z * x))
//roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
// Parametry:
// [we] *qA - wskaźnik na kwaternion wektora przyspieszenia A = (q0 + iqx + jqy + kqz)
// [we] *qM - wskaźnik na kwaternion wektora magnetycznego M = (q0 + iqx + jqy + kqz)
// [wy] *katy - wskaźnik na zmienną z 3 kątami [Phi, Theta, Psi]
// Zwraca: nic
// Czas trwania: ? us na 200MHz
////////////////////////////////////////////////////////////////////////////////
void KatyKwaterniona3(float *qA, float *qM, float *fKaty)
{
	*(fKaty+0) = atan2f(2 * (*(qA+0) * *(qA+1) + *(qA+2) * *(qA+3)), 1 - 2 * (*(qA+1) * *(qA+1) + *(qA+2) * *(qA+2))) / -2;
	*(fKaty+1) = asinf (2 * (*(qA+0) * *(qA+2) - *(qA+3) * *(qA+1))) / -2;
	*(fKaty+2) = atan2f(2 * (*(qM+0) * *(qM+3) + *(qM+1) * *(qM+2)), 1 - 2 * (*(qM+2) * *(qM+2) + *(qM+3) * *(qM+3))) / -2;
}



///////////////////////////////////////////////////////////////////////////////
// Konwertuje kwaternion na kąty Eulera. Ma ten feler że pochylenie zwraca tylko w zakresie +-Pi/2
// Parametry:
//  *fQ - wskaźnik na kwaternion wejsciowy
//  *fKaty - wskaźnik na tablicę 3 kątów Eulera
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KwaternionNaKąty(float *fQ, float *fKaty)
{
	*(fKaty + 0) = atan2f(2.0f * (*(fQ+0) * *(fQ+1) + *(fQ+2) * *(fQ+3)), 1.0f - 2.0f * (*(fQ+1) * *(fQ+1) + *(fQ+2) * *(fQ+2)));
	*(fKaty + 2) = atan2f(2.0f * (*(fQ+0) * *(fQ+3) + *(fQ+1) * *(fQ+2)), 1.0f - 2.0f * (*(fQ+2) * *(fQ+2) + *(fQ+3) * *(fQ+3)));

	//pochylenie normalnie zwracane jest w zakresie +-Pi/2, więc aby rozszerzć kąt do +-Pi, trzeba zbadać, w którym z dwu alternatywnych położeniach znajduje się BSP
    float fSinPochylenia = 2.0f * (*(fQ+0) * *(fQ+2) - *(fQ+1) * *(fQ+3));
    if (fabsf(fSinPochylenia) >= 1.0f)
    	*(fKaty + 1) = -copysignf(M_PI / 2.0f, fSinPochylenia); // Zabezpieczenie przed Gimbal Lock / NaN
    else
    	*(fKaty + 1) = asinf(fSinPochylenia);
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje kwaternion na kąty Eulera. Porównuje alternatywne reprezentację katów i sprawdza ich odległość od poprzedniej pozycji.
// Ponieważ zmieniają się jednocześnie dwa kąty, więc to nie działa, chociaz pomysł jest fajny
// Parametry:
//  *fQ - wskaźnik na kwaternion wejsciowy
//  *fKaty - wskaźnik na tablicę 3 kątów Eulera
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KwaternionNaKąty2(float *fQ, float *fKaty)
{
	float fKątPoprzedni[3];
	float fKątAlternatywny[3];

	//zapamiętaj poprzednią wartość kata
	for (uint8_t n=0; n<3; n++)
		fKątPoprzedni[n] = *(fKaty + n);

	*(fKaty + 0) = atan2f(2.0f * (*(fQ+0) * *(fQ+1) + *(fQ+2) * *(fQ+3)), 1.0f - 2.0f * (*(fQ+1) * *(fQ+1) + *(fQ+2) * *(fQ+2)));
	*(fKaty + 2) = atan2f(2.0f * (*(fQ+0) * *(fQ+3) + *(fQ+1) * *(fQ+2)), 1.0f - 2.0f * (*(fQ+2) * *(fQ+2) + *(fQ+3) * *(fQ+3)));

	//pochylenie normalnie zwracane jest w zakresie +-Pi/2, więc aby rozszerzć kąt do +-Pi, trzeba zbadać, w którym z dwu alternatywnych położeniach znajduje się BSP
    float fSinPochylenia = 2.0f * (*(fQ+0) * *(fQ+2) - *(fQ+1) * *(fQ+3));
    //if (fabsf(fSinPochylenia) >= 1.0f)
    	//*(fKaty + 1) = -copysignf(M_PI / 2.0f, fSinPochylenia); // Zabezpieczenie przed Gimbal Lock / NaN
    //else
    	//*(fKaty + 1) = asinf(fSinPochylenia);

    if (fSinPochylenia > 1.0f)
    	fSinPochylenia = 1.0f;
    else if (fSinPochylenia < -1.0f)
      	fSinPochylenia = -1.0f;
    //sinPochylenia = fmaxf(-1.0f, fminf(1.0f, fSinPochylenia));
    *(fKaty + 1) = asinf(fSinPochylenia);		//pierwsza alternatywna kąta pochylenia

	fKątAlternatywny[0] = ZawińPi(M_PI + *(fKaty + 0));	//wartość alternatywnego kąta przechylenia
	fKątAlternatywny[1] = ZawińPi(M_PI - *(fKaty + 1));	//wartość alternatywnego kąta pochylenia
	fKątAlternatywny[2] = ZawińPi(M_PI + *(fKaty + 2));	//wartość alternatywnego kąta odchylenia

	//policz kwadrat odległości do poprzedniej wartości kąta aby pozbyć się ewentualnych znaków
	float fOdległość1 = 0.0f;
	float fOdległość2 = 0.0f;

	for (uint8_t n=0; n<3; n++)
	{
		fOdległość1 += powf(ZawińPi(*(fKaty + n) - fKątPoprzedni[n]), 2.0f);
		fOdległość2 += powf(ZawińPi(fKątAlternatywny[n] - fKątPoprzedni[n]), 2.0f);
	}
	if (fOdległość2 < fOdległość1)	//jeżeli kąt alternatywny jest bliżej, to użyj jego
	{
		for (uint8_t n=0; n<3; n++)
			*(fKaty + n) = fKątAlternatywny[n];
	}
}



////////////////////////////////////////////////////////////////////////////////
// Konwertuje kwaternion na kąty Eulera
// Phi = atan2(2.0f * (qw*qx + qy*qz), 1.0f - 2.0f * (qx*qx + qy*qy))
// Parametry:
//  *fQ - wskaźnik na kwaternion wejsciowy
//  *fKaty - wskaźnik na tablicę 3 kątów Eulera
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KwaternionNaKąty3(float *fQ, float *fKaty, stKatyRozszerz_t *stKatyRozszerz)
{
	float fKątAlternatywny[3];

	*(fKaty + 0) = atan2f(2.0f * (*(fQ+0) * *(fQ+1) + *(fQ+2) * *(fQ+3)), 1.0f - 2.0f * (*(fQ+1) * *(fQ+1) + *(fQ+2) * *(fQ+2)));
	*(fKaty + 2) = atan2f(2.0f * (*(fQ+0) * *(fQ+3) + *(fQ+1) * *(fQ+2)), 1.0f - 2.0f * (*(fQ+2) * *(fQ+2) + *(fQ+3) * *(fQ+3)));

	//pochylenie normalnie zwracane jest w zakresie +-Pi/2, więc aby rozszerzć kąt do +-Pi, trzeba zbadać, w którym z dwu alternatywnych położeniach znajduje się BSP
    float fSinPochylenia = 2.0f * (*(fQ+0) * *(fQ+2) - *(fQ+1) * *(fQ+3));
    fSinPochylenia = fmaxf(-1.0f, fminf(1.0f, fSinPochylenia));
    *(fKaty + 1) = asinf(fSinPochylenia);		//pierwsza alternatywna kąta pochylenia

	fKątAlternatywny[0] = ZawińPi(M_PI + *(fKaty + 0));	//wartość alternatywnego kąta przechylenia
	fKątAlternatywny[1] = ZawińPi(M_PI - *(fKaty + 1));	//wartość alternatywnego kąta pochylenia
	fKątAlternatywny[2] = ZawińPi(M_PI + *(fKaty + 2));	//wartość alternatywnego kąta odchylenia

	if (!stKatyRozszerz->cZainicjowany)
	{
		for (uint8_t n=0; n<3; n++)
			stKatyRozszerz->fKaty[n] = *(fKaty + n);

		for (uint8_t n=0; n<4; n++)
			stKatyRozszerz->fQwat[n] = *(fQ + n);
		stKatyRozszerz->cZainicjowany = 1;
		return;
	}

	//Względny obrót:	fdQ = KwaternionSprzezony(Qpoprzedni) * Qbieżący
	float fQSprzężony[4];
	float fdQ[4];
	KwaternionSprzezony(fQ, fQSprzężony);
	MnozenieKwaternionow(fQSprzężony, fQ, fdQ);

	if (fdQ[0] < 0.0f)
	{
		for (uint8_t n=0; n<4; n++)
			fdQ[n] = -fdQ[n];
	}

	//liczę przybliżone przyrosty katów obrotu. Poprawne dla małych przyrostów
	float fRóżnicaKąta[3];
	for (uint8_t n=0; n<3; n++)
		fRóżnicaKąta[n] = 2.0f * fdQ[n+1];

	//oblicz wartość przewidywanego kąta
	float fPrzewidywanyKąt[3];

	for (uint8_t n=0; n<3; n++)
		fPrzewidywanyKąt[n] = ZawińPi(stKatyRozszerz->fKaty[n] + fRóżnicaKąta[n]);


	//policz kwadrat odległości do poprzedniej wartości kąta aby pozbyć się ewentualnych znaków
	float fOdległość1 = 0.0f;
	float fOdległość2 = 0.0f;

	for (uint8_t n=0; n<3; n++)
	{
		fOdległość1 += powf(ZawińPi(*(fKaty + n) - fPrzewidywanyKąt[n]), 2.0f);
		fOdległość2 += powf(ZawińPi(fKątAlternatywny[n] - fPrzewidywanyKąt[n]), 2.0f);
	}
	if (fOdległość2 < fOdległość1)	//jeżeli kąt alternatywny jest bliżej, to użyj jego
	{
		for (uint8_t n=0; n<3; n++)
			*(fKaty + n) = fKątAlternatywny[n];
	}

	//zapamiętanie wartości biezących jako poprzednich
	for (uint8_t n=0; n<3; n++)
		stKatyRozszerz->fKaty[n] = *(fKaty + n);

	for (uint8_t n=0; n<4; n++)
		stKatyRozszerz->fQwat[n] = fQ[n];
}



////////////////////////////////////////////////////////////////////////////////
// Oblicza wartość znormalizowanego kwaternionu na podstawie katów Eulera na podstawie wzoru:
//q0 = CosPhi2 * CosThe2 * CosPsi2 + SinPhi2 * SinThe2 * SinPsi2
//q1 = SinPhi2 * CosThe2 * CosPsi2 - CosPhi2 * SinThe2 * SinPsi2
//q2 = CosPhi2 * SinThe2 * CosPsi2 + SinPhi2 * CosThe2 * SinPsi2
//q3 = CosPhi2 * CosThe2 * SinPsi2 - SinPhi2 * SinThe2 * CosPsi2
// Parametry:
//  *fKaty - wskaźnik na tablicę 3 kątów eulera
//  *fQ - wskaźnik na kwaternion
// Zwraca: nic
////////////////////////////////////////////////////////////////////////////////
void KatyNaKwaternion(float *fKaty, float *fQ)
{
	float q[4];

	float fSinPhi2 = sinf(*(fKaty + 0) / 2);
	float fCosPhi2 = cosf(*(fKaty + 0) / 2);
	float fSinThe2 = sinf(*(fKaty + 1) / 2);
	float fCosThe2 = cosf(*(fKaty + 1) / 2);
	float fSinPsi2 = sinf(*(fKaty + 2) / 2);
	float fCosPsi2 = cosf(*(fKaty + 2) / 2);

	q[0] = fCosPhi2 * fCosThe2 * fCosPsi2 + fSinPhi2 * fSinThe2 * fSinPsi2;
	q[1] = fSinPhi2 * fCosThe2 * fCosPsi2 - fCosPhi2 * fSinThe2 * fSinPsi2;
	q[2] = fCosPhi2 * fSinThe2 * fCosPsi2 + fSinPhi2 * fCosThe2 * fSinPsi2;
	q[3] = fCosPhi2 * fCosThe2 * fSinPsi2 - fSinPhi2 * fSinThe2 * fCosPsi2;

	NormalizujWektor(q, fQ, 4);
}



////////////////////////////////////////////////////////////////////////////////
// Normalizacja wektora o podanym rozmiarze, w tym również kwaternionu
// Parametry:
// [we] *fWe - wskaźnik na wektor wejściowy
// [wy] *fWy - wskaźnik na wektor znormalizowany
// [we] chRozmiar - rozmiar wektora
// Zwraca: nic
// Czas trwania: 2,06us @200MHz dla 3 elementów
////////////////////////////////////////////////////////////////////////////////
void NormalizujWektor(float * fWe, float *fWy, uint8_t chRozmiar)
{
	float fNorm = 0.0f;

	for (uint8_t n=0; n<chRozmiar; n++)
		fNorm += *(fWe + n) * *(fWe + n);
	fNorm = sqrtf(fNorm);

	for (uint8_t n=0; n<chRozmiar; n++)
	{
		if (fNorm > 1e-12f)
			*(fWy + n) = *(fWe + n) / fNorm;
		else
			*(fWy + n) = 0.0f;
	}
}


