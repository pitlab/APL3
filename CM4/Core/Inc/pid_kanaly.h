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
#define GPSX   4    //regulator sterowania prędkością i położeniem w osi X
#define GPSY   5    //regulator sterowania prędkością i położeniem w osi Y

#define NUM_PARAMS  6 //liczba regulowanych parametrów


//definicje nazw regulatorów
#define PID_GYP 0   //regulator sterowania prędkością kątową przechylenia (żyroskop P)
#define PID_PHI 1   //regulator sterowania przechyleniem (lotkami w samolocie)
#define PID_GYQ 2   //regulator sterowania prędkością kątową pochylenia (żyroskop Q)
#define PID_THE 3   //regulator sterowania pochyleniem (sterem wysokości)
#define PID_GYR 4   //regulator sterowania prędkością kątową odchylenia (żyroskop R)
#define PID_PSI 5  	//regulator sterowania odchyleniem (sterem kierunku)
#define PID_VAR 6   //regulator sterowani prędkością wznoszenia (wario)
#define PID_ALT 7   //regulator sterowania wysokością
#define PID_SPX 8   //regulator sterowania prędkością postępową w X
#define PID_POX 9  	//regulator stabilizacji wzgl. punktu w osi X
#define PID_SPY 10  //regulator sterowania prędkością postępową w Y
#define PID_POY 11  //regulator stabilizacji wzgl. punktu w osi Y

#define NUM_PIDS  12 //liczba regulatorów


//definicje trybów pracy regulatora
#define REG_OFF           0   //wyłączony regulator
#define REG_MAN           1   //regulacja ręczna prosto z drążka
#define REG_ACRO          2   //regulacja pierwszej pochodnej parametru (prędkość katowa, prędkość wznoszenia)
#define REG_STAB          3   //regulacja parametru właściwego (kąt , wysokość)
#define REG_GPS_SPEED     4   //regulacja prędkości liniowej w osiach XYZ
#define REG_GPS_POS       5   //regulacja prędkości liniowej w osiach XYZ ze stabilizacją położenia

#define NUM_REG_MOD       6   //liczba trybów regulatora
