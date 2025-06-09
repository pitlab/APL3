//#define NUM_AXIS  6 //liczba regulowanych osi: pochylenie, przechylenie, odchylenie, wysokość, prędkość + rezerwa
#define FRAM_FLOAT_SIZE     4   //rozmiar liczby float
#define PID_FRAM_CH_SIZE    4   //ilość zmiennych na kanał regulatora zapisywanych do FRAM

//Rodzaj regulatora
#define REG_KAT     1   //regulator kątowy
#define REG_LIN     0   //regulator liniowy

//definicje nazw regulowanych parametrów
#define ROLL   0    //regulator sterowania przechyleniem (lotkami w samolocie)
#define PITCH  1    //regulator sterowania pochyleniem (sterem wysokości)
#define YAW    2    //regulator sterowania obrotem (sterem kierunku)
#define ALTI   3    //regulator sterowania wysokością
#define GPSN   4    //regulator sterowania prędkością i położeniem północnym
#define GPSE   5    //regulator sterowania prędkością i położeniem wschodnim

#define NUM_PARAMS  6 //liczba regulowanych parametrów


//definicje nazw regulatorów
#define PID_PHI 	0   //regulator sterowania przechyleniem (lotkami w samolocie)
#define PID_GYP 	1   //regulator sterowania prędkością kątową przechylenia (żyroskop P)
#define PID_THE 	2   //regulator sterowania pochyleniem (sterem wysokości)
#define PID_GYQ 	3   //regulator sterowania prędkością kątową pochylenia (żyroskop Q)
#define PID_PSI 	4  	//regulator sterowania odchyleniem (sterem kierunku)
#define PID_GYR 	5   //regulator sterowania prędkością kątową odchylenia (żyroskop R)
#define PID_WYS 	6   //regulator sterowania wysokością
#define PID_WAR 	7   //regulator sterowani prędkością wznoszenia (wario)
#define PID_NAW_N 	8   //regulator sterowania nawigacją w kierunku północnym
#define PID_PRE_N	9  	//regulator sterowania prędkością w kierunku północnym
#define PID_NAW_E 	10  //regulator sterowania nawigacją w kierunku wschodnim
#define PID_PRE_E	11 	//regulator sterowania prędkością w kierunku wschodnim

#define LICZBA_PID  12 //liczba regulatorów


//definicje trybów pracy regulatora
#define REG_OFF           0   //wyłączony regulator
#define REG_MAN           1   //regulacja ręczna prosto z drążka
#define REG_ACRO          2   //regulacja pierwszej pochodnej parametru (prędkość katowa, prędkość wznoszenia)
#define REG_STAB          3   //regulacja parametru właściwego (kąt , wysokość)
#define REG_GPS_SPEED     4   //regulacja prędkości liniowej w osiach XYZ
#define REG_GPS_POS       5   //regulacja prędkości liniowej w osiach XYZ ze stabilizacją położenia

#define NUM_REG_MOD       6   //liczba trybów regulatora
