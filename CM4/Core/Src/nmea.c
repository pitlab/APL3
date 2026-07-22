//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł dekodowania protokołu NMEA
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include <Nmea.h>
#include <stdio.h>
#include "wymiana.h"

//definicje zmiennych
uint8_t cBufStanu[BUF_STAN_SIZE];	//tablica do zbierania odbieranych danych
uint8_t cBajtStanu;
uint8_t cStan;		//stan protokołu dekodującego
int8_t cStrefaCzas = -1;       //różnica między czasem GPS a strefą czasową użytkownika
uint8_t cNoweDane;       //odebrano komplet nowych danych Lon-Lat
uint8_t cNewGAlti;     //odebarno dane o wysokości

//CGA indeks GS oznacza zmianna dla danych pobieranych z systemu GLONASS
uint16_t sGMSek;
uint16_t sGMSekGS;
uint8_t cGGA_LatNS, cGGA_LonWE;

//GSA
uint8_t cGSAMode1;     //tryb pracy A-automatyczny, M-ręczny
uint8_t cSatNumber;    //liczba satelitów
uint8_t cSatNumberGS;
float fGPDOP, fGHDOP, fGVDOP; //rozmycie precyzji

//RMC
uint8_t cRMC_Status;
uint16_t sGAltitude, sGSpeed100k, sGCourse100k,sGSpeed100kGS, sGCourse100kGS, sGAltitudeGS;
float fGCourseGS = 0, fGSpeedGS = 0;
double dLongitudeRAW, dLatitudeRAW;	//, dLatitudeGS, dLongitudeGS;

extern volatile unia_wymianyCM4_t uDaneCM4;



////////////////////////////////////////////////////////////////////////////////
// Dekoduj ciągi znaków NMEA
// Parametry: odebrany bajt
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t DekodujNMEA(uint8_t cDaneIn)
{
    uint8_t cBłąd = BLAD_OK;

    if (cBajtStanu >= BUF_STAN_SIZE)
        cBajtStanu = 0;

    cBufStanu[cBajtStanu] = cDaneIn;
    cBajtStanu++;
    

    switch(cStan)
    {
    case ST_NAGLOWEK1:	//czekaj na nagłówek
	if (cDaneIn == '$')
	{
	    cStan = ST_NAGLOWEK2;
        cBajtStanu = 0;
	}
	break;

    case ST_NAGLOWEK2:	//czekaj na znaki GP badz na znaki GN
	if (cBajtStanu == 2)
	{
	    if ((cBufStanu[0] == 'G') && (cBufStanu[1] == 'P'))
	    	cStan = ST_MSGID_GP;
	    else
	    if ((cBufStanu[0] == 'G') && (cBufStanu[1] == 'N'))
	    	cStan = ST_MSGID_GN;
	    else
	    if ((cBufStanu[0] == 'P') && (cBufStanu[1] == 'M'))
	    	cStan = ST_MSGID_MTK;
	    else
	    	cStan = ST_ERR;    //błąd

	    cBajtStanu = 0;
	}
	break;
	
    case ST_MSGID_GP:  //czekaj na typ komunikatu GPS
	if (cBajtStanu == 4)	//3 znaki typu komunikatu + przecinek
	{
	    if (cBufStanu[0]== 'G')    //GGA, GLL, GSA, GSV
	    {
			if (cBufStanu[1]== 'G')	//GGx
			{
				if (cBufStanu[2]== 'A')    //GGA +
					cStan = ST_GGA_TIME;
				else
					cStan = ST_ERR;
			}
			else
					if (cBufStanu[1]== 'S')	//GSx
			{
				if (cBufStanu[2]== 'A')    //GSA +
					cStan = ST_GSA_MODE1;
				else
					cStan = ST_ERR;
			}
	    }
	    else
        if (cBufStanu[0]== 'R')    //RMC +
        {
			if (cBufStanu[1]== 'M')	//RMx
			{
				if (cBufStanu[2]== 'C')    //RMC
					cStan = ST_RMC_UTC;
				else
					cStan = ST_ERR;
			}
            else
            	cStan = ST_ERR;
	    }
        else
        	cStan = ST_ERR;
        cBajtStanu = 0;
	}
	break;

    case ST_MSGID_GN:  //czekaj na typ komunikatu GLONASS
	if (cBajtStanu == 4)	//3 znaki typu komunikatu + przecinek
	{
	    if (cBufStanu[0]== 'G')    //GGA, GLL, GSA, GSV
	    {
	    	if (cBufStanu[1]== 'S')	//GSx
			{
				if (cBufStanu[2]== 'A')    //GSA +
					cStan = ST_GSA_MODE1_GS;
				else
					cStan = ST_ERR;
			}
	    	else
	    	if (cBufStanu[1]== 'G')	//GGx
			{
				if (cBufStanu[2]== 'A')    //GGA +
					cStan = ST_GGA_TIME;
				else
					cStan = ST_ERR;
			}
			else
	    		cStan = ST_ERR;
	    }
	    else
	    if (cBufStanu[0]== 'R')    //RMC +
        {
	    	if (cBufStanu[1]== 'M')	//RMx
			{
				if (cBufStanu[2]== 'C')    //RMC
					cStan = ST_RMC_UTC_GS;
				else
					cStan = ST_ERR;
			}
	    	else
	    		cStan = ST_ERR;
	    }
	    else
	    if (cBufStanu[0]== 'T')    //TXT
	    {
	    	if (cBufStanu[1]== 'X')	//TXx
			{
				if (cBufStanu[2]== 'T')    //TXT
					cStan = ST_TXT;
				else
					cStan = ST_ERR;
			}
			else
				cStan = ST_ERR;
	    }
	    else
	    	cStan = ST_ERR;
	    cBajtStanu = 0;
	}
	break;
    //dekoduj komunikat GGA
    case ST_GGA_TIME:		//czas
        if (cDaneIn == '.')
		{
			uDaneCM4.dane.stGnss1.cGodz = Asci2UChar(cBufStanu+0, 2) - cStrefaCzas;
			uDaneCM4.dane.stGnss1.cMin  = Asci2UChar(cBufStanu+2, 2);
			uDaneCM4.dane.stGnss1.cSek  = Asci2UChar(cBufStanu+4, 2);
		}
		else
		if (cDaneIn == ',')
		{
			sGMSek  = Asci2UShort(cBufStanu+7, cBajtStanu-7-1);
			cStan = ST_GGA_LATITUDE;
			cBajtStanu = 0;
		}
		else
		if (cBajtStanu > 12)
		{
			cStan = ST_ERR;
			cBajtStanu = 0;
		}
		break;

    case ST_GGA_LATITUDE:   //długość geograficzna
        if (cDaneIn == ',')
        {
            cStan = ST_GGA_LAT_NS;
			cBajtStanu = 0;
		}
		break;


    case ST_GGA_LAT_NS:
        if (cDaneIn == ',')
		{
			cGGA_LatNS = cBufStanu[0];
            cStan = ST_GGA_LONGITUD;
            cBajtStanu = 0;
		}
		break;

    case ST_GGA_LONGITUD:   //szerokość geograficzna
        if (cDaneIn == ',')
        {
             cStan = ST_GGA_LON_WE;
			 cBajtStanu = 0;
       	}
		break;

    case ST_GGA_LON_WE:
       if (cDaneIn == ',')
       {
    	   cGGA_LonWE = cBufStanu[0];
           cStan = ST_GGA_FIX_IND;
           cBajtStanu = 0;
       }
       break;

    case ST_GGA_FIX_IND:
       if (cDaneIn == ',')
       {
    	   uDaneCM4.dane.stGnss1.cFix = cBufStanu[0] - '0';	//fix przechowuj jako liczbę a nie znak
           cStan = ST_GGA_SAT_USE;
           cBajtStanu = 0;
       }
       break;

    case ST_GGA_SAT_USE:
       if (cDaneIn == ',')
       {
    	   uDaneCM4.dane.stGnss1.cLiczbaSatelit = Asci2UChar(cBufStanu, cBajtStanu-1);
           cStan = ST_GGA_HDOP;
           cBajtStanu = 0;
       }
       break;

    case ST_GGA_HDOP:	//GPS zwraca puste pole
    	if (cDaneIn == ',')
    	{
    		 if (cBajtStanu > 3)
    			 cBłąd = DecodeFloat(cBufStanu, cBajtStanu-1, (float*)&uDaneCM4.dane.stGnss1.fHdop);
            cStan = ST_GGA_ALTITUDE;
            cBajtStanu = 0;
    	}
    	break;
        
    case ST_GGA_ALTITUDE: 
        if (cDaneIn == '.')
        	cBajtStanu--;  //nie analizuj kropki, liczba będzie 10x większa
        else
        if (cDaneIn == ',')
        {
            if ((cBajtStanu > 1) && (cBajtStanu < 7))
                sGAltitude = Asci2UShort(cBufStanu+0, cBajtStanu-1);
            else
                sGAltitude = 0;
            uDaneCM4.dane.stGnss1.fWysokoscMSL = (float)sGAltitude/10;
            cNewGAlti = 1; //flaga nowych danych o wysokości
            cStan = ST_NAGLOWEK1;
            cBajtStanu = 0;
        }
        break;


    //dekodowanie komunikatu GSA
    case ST_GSA_MODE1:
        if (cDaneIn == ',')
        {
        	cGSAMode1 = cBufStanu[0];
            cStan = ST_GSA_MODE2;
            cBajtStanu = 0;
        }
        break;
        
    case ST_GSA_MODE2:
        if (cDaneIn == ',')
        {
            cStan = ST_GSA_SAT_USED;
            cSatNumber = 0;
            cBajtStanu = 0;
        }
        break;

    case ST_GSA_SAT_USED:
        if (cDaneIn == ',')
        {
            cSatNumber++;
            if (cSatNumber >= 12)
            {
                cStan = ST_GSA_PDOP;
                cBajtStanu = 0;
            }

        }
        break;

    case ST_GSA_PDOP:
        if (cDaneIn == ',')
        {
            cBłąd = DecodeFloat(cBufStanu, cBajtStanu-1, &fGPDOP);
            if ((cBłąd == BLAD_OK) | (cBłąd == BLAD_BRAK_DANYCH))
                cStan = ST_GSA_HDOP;
            else
                cStan = ST_ERR;
            cBajtStanu = 0;
		}
        break;

    case ST_GSA_HDOP:
        if (cDaneIn == ',')
        {
            cBłąd = DecodeFloat(cBufStanu, cBajtStanu-1, &fGHDOP);
            uDaneCM4.dane.stGnss1.fHdop = fGHDOP;
            if ((cBłąd == BLAD_OK) | (cBłąd == BLAD_BRAK_DANYCH))
                cStan = ST_GSA_VDOP;
            else
                cStan = ST_ERR;
            cBajtStanu = 0;
		}
        break;

    case ST_GSA_VDOP:
        if (cDaneIn == '*')
        {
            cBłąd = DecodeFloat(cBufStanu, cBajtStanu-1, &fGVDOP);
            if ((cBłąd == BLAD_OK) | (cBłąd == BLAD_BRAK_DANYCH))
                cStan = ST_NAGLOWEK1;
            else
                cStan = ST_ERR;
            cBajtStanu = 0;
        }
        break;

         //dekodowanie komunikatu GSA z systemu GLONASS
    case ST_GSA_MODE1_GS:
        if (cDaneIn == ',')
        {
        	cGSAMode1 = cBufStanu[0];
            cStan = ST_GSA_MODE2_GS;
            cBajtStanu = 0;
        }
        break;
        
    case ST_GSA_MODE2_GS:
        if (cDaneIn == ',')
        {
        	uDaneCM4.dane.stGnss1.cFix = cBufStanu[0]-'0';
            if ((uDaneCM4.dane.stGnss1.cFix < 1) || (uDaneCM4.dane.stGnss1.cFix > 3))   //sprawdź czy to liczba
            {
            	 uDaneCM4.dane.stGnss1.cFix = 1; //jeżeli błąd błąd to przyjmij najgorszy możliwy
            	 cStan = ST_ERR;
            }
            else
                cStan = ST_GSA_SAT_USED_GS;
            uDaneCM4.dane.stGnss1.cLiczbaSatelit = 0;
            cBajtStanu = 0;
        }
        break;

    case ST_GSA_SAT_USED_GS:
        if (cDaneIn == ',')
        {
        	uDaneCM4.dane.stGnss1.cLiczbaSatelit++;
            if (uDaneCM4.dane.stGnss1.cLiczbaSatelit >= 12)
            {
                cStan = ST_GSA_PDOP_GS;
                cBajtStanu = 0;
            }
        }
        break;

    case ST_GSA_PDOP_GS:
        if (cDaneIn == ',')
        {
            cBłąd = DecodeFloat(cBufStanu, cBajtStanu-1, &fGPDOP);
            if ((cBłąd == BLAD_OK) | (cBłąd == BLAD_BRAK_DANYCH))
                cStan = ST_GSA_HDOP_GS;
            else
                cStan = ST_ERR;
            cBajtStanu = 0;
        }
        break;

    case ST_GSA_HDOP_GS:
        if (cDaneIn == ',')
        {
            cBłąd = DecodeFloat(cBufStanu, cBajtStanu-1, &fGHDOP);
            if ((cBłąd == BLAD_OK) | (cBłąd == BLAD_BRAK_DANYCH))
                cStan = ST_GSA_VDOP_GS;
            else
                cStan = ST_ERR;
            cBajtStanu = 0;
        }
        break;

    case ST_GSA_VDOP_GS:
        if (cDaneIn == '*')
        {
            cBłąd = DecodeFloat(cBufStanu, cBajtStanu-1, &fGVDOP);
            if ((cBłąd == BLAD_OK) | (cBłąd == BLAD_BRAK_DANYCH))
                cStan = ST_NAGLOWEK1;
            else
                cStan = ST_ERR;
            cBajtStanu = 0;
        }
        break;

    //dekodowanie komunikatów MTK
    case ST_MSGID_MTK:
    	if (cDaneIn == ',')
    	{
    		 if ((cBufStanu[0] == 'T') && (cBufStanu[1] == 'K') && (cBufStanu[2] == '0') && (cBufStanu[3] == '1'))
    		 {
    			 if (cBufStanu[4] == '1')
    				 cStan = ST_PMTK011;
    			 else
    			 if (cBufStanu[4] == '0')
    			     cStan = ST_PMTK010;
    			 else
    				 cStan = ST_ERR;
    		 }
    		 else
    			 cStan = ST_ERR;
    		 cBajtStanu = 0;
    	}
    	break;

    //dekodowanie komunikatu startowego chipsetu MTK: $PMTK011,MTKGPS*08<CR><LF>
    case ST_PMTK011:
    	if (cDaneIn == 0x0A)	//<LF>
    	{
    		 if ((cBufStanu[0] == 'M') && (cBufStanu[1] == 'T') && (cBufStanu[2] == 'K') && (cBufStanu[3] == 'G') && (cBufStanu[4] == 'P') && (cBufStanu[5] == 'S'))
    			 cStan = ST_PMTK010;
    		 else
    			 cStan = ST_ERR;
    		 cBajtStanu = 0;
    	}
    	break;

    //Dekodowanie komunikatu kończącego inicjalizację modułu: $PMTK010,001*2E<CR><LF>
    case ST_PMTK010:
    	const uint8_t chWzorzec[] =  "001*2E";
    	if (cDaneIn == 0x0A)	//<LF>
    	{
			for (uint8_t n=0; n<6; n++)
			{
				if (cBufStanu[n] != chWzorzec[n])
				{
					cStan = ST_ERR;
					cBajtStanu = 0;
					break;
				}
			}
    		uDaneCM4.dane.nZainicjowano |= INIT_WYKR_MTK;
    		cStan = ST_NAGLOWEK1;
    		cBajtStanu = 0;
    	}
    	break;

    //dekodowanie komunikatu RMC Recommended Minimum GLONASS
    case ST_RMC_UTC_GS:
    	if (cDaneIn == '.')
    	{
        	  uDaneCM4.dane.stGnss1.cGodz = Asci2UChar(cBufStanu+0, 2) - cStrefaCzas;
        	  uDaneCM4.dane.stGnss1.cMin  = Asci2UChar(cBufStanu+2, 2);
        	  uDaneCM4.dane.stGnss1.cSek  = Asci2UChar(cBufStanu+4, 2);
    	}
		else
		if (cDaneIn == ',')
		{
			sGMSek  = Asci2UShort(cBufStanu+7, cBajtStanu-7-1);
			cStan = ST_RMC_STATUS_GS;
			cBajtStanu = 0;
		}
		else
		if (cBajtStanu > 12)
		{
			cStan = ST_ERR;
			cBajtStanu = 0;
		}
		break;

    case ST_RMC_STATUS_GS:	    //status A-dane ważne, V-dane nieważne
        if (cDaneIn == ',')
        {
        	if (cBajtStanu == 2)
        	{
        		cRMC_Status = cBufStanu[0];
        		cStan = ST_RMC_LATITUDE_GS;
        	}
        	else
        		cStan = ST_ERR;
        	cBajtStanu = 0;
        }
        break;

    case ST_RMC_LATITUDE_GS:   //długość geograficzna
        if (cDaneIn == ',')
        {
            if (cRMC_Status == 'A')
            {
                cBłąd = DecodeLonLat(cBufStanu, cBajtStanu, &dLatitudeRAW);
                if (cBłąd)
                {
                    cStan = ST_ERR;
                    cBajtStanu = 0;
                    return cBłąd;
                }
            }
            cStan = ST_RMC_LAT_NS_GS;
            cBajtStanu = 0;
        }
        break;

    case ST_RMC_LAT_NS_GS:
        if (cDaneIn == ',')
		{
			if (cBajtStanu == 2)
			{
				if (cBufStanu[0] == 'S')
					uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW * -1;
				else
					uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW;
			}
			cStan = ST_RMC_LONGITUD_GS;
            cBajtStanu = 0;
		}
        break;

    case ST_RMC_LONGITUD_GS:   //szerokość geograficzna
        if (cDaneIn == ',')
        {
            if (cRMC_Status == 'A')
            {
                cBłąd = DecodeLonLat(cBufStanu, cBajtStanu, &dLongitudeRAW);
                if (cBłąd)
                {
                    cStan = ST_ERR;
                    cBajtStanu = 0;
                    return cBłąd;
                }
            }
            cStan = ST_RMC_LON_WE_GS;
            cBajtStanu = 0;
        }
        if (cBajtStanu > 12)
        {
        	cStan = ST_ERR;
        	cBajtStanu = 0;
        }
        break;

    case ST_RMC_LON_WE_GS:
        if (cDaneIn == ',')
		{
			if (cBajtStanu == 2)
			{
				if (cBufStanu[0] == 'W')
					uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW * -1;
				else
					uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW;
			}
			cStan = ST_RMC_SPEED_GS;
			cBajtStanu = 0;
		}
		break;

    case ST_RMC_SPEED_GS:
        if (cDaneIn == '.')
		{
			sGSpeed100k = 100 * Asci2UShort(cBufStanu+0, cBajtStanu-1);
		}
		else
		if (cDaneIn == ',')
		{
			if (cBajtStanu > 1)
				sGSpeed100k += Asci2UShort(cBufStanu+cBajtStanu-3, 2);
			else
				sGSpeed100k = 0;
			//zamień jednostke z węzłów = [mila/h] na [m/s]
			uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi = (float)sGSpeed100k *(float)0.00514444;
			//cStan = ST_RMC_COURSE_GS;
			cStan = ST_RMC_COURSE;
			cBajtStanu = 0;
		}
		break;

    /*case ST_RMC_COURSE_GS:
        if (cDaneIn == '.')
        {
        	sGCourse100k = 100 * Asci2UShort(cBufStanu+0, cBajtStanu-1);
        }
        else
        if (cDaneIn == ',')
        {
            if (cBajtStanu > 1)
            	sGCourse100k += Asci2UShort(cBufStanu+cBajtStanu-3, 2);
            else
            	sGCourse100kGS = 0;
            uDaneCM4.dane.stGnss1.fKurs = (float)sGCourse100k/100;
            cStan = ST_RMC_DATE;
            cBajtStanu = 0;
            //wykonaj pomiar czasu od pojawienia się ostatniego kompletu danych
            cNoweDane = 2;     //mamy komplet nowych danych
            //nGpsTime = CountTime(&nLastGpsTime);
        }
        break; */

    case ST_RMC_UTC:
   		if ((cDaneIn == '.') && (cRMC_Status == 'A'))
		{
    		uDaneCM4.dane.stGnss1.cGodz = Asci2UChar(cBufStanu+0, 2) - cStrefaCzas;
    		uDaneCM4.dane.stGnss1.cMin  = Asci2UChar(cBufStanu+2, 2);
    		uDaneCM4.dane.stGnss1.cSek  = Asci2UChar(cBufStanu+4, 2);
		}
		else
        if (cDaneIn == ',')
		{
			sGMSek  = Asci2UShort(cBufStanu+7, cBajtStanu-7-1);
			cStan = ST_RMC_STATUS;
			cBajtStanu = 0;
		}
		else
		if (cBajtStanu > 12)
		{
			cStan = ST_ERR;
			cBajtStanu = 0;
		}
		break;

    case ST_RMC_STATUS:	    //status A-dane ważne, V-dane nieważne
        if (cDaneIn == ',')
		{
			if (cBajtStanu == 2)
			{
				cRMC_Status = cBufStanu[0];
				cStan = ST_RMC_LATITUDE;
			}
			else
				cStan = ST_ERR;
            cBajtStanu = 0;
		}
		break;

    case ST_RMC_LATITUDE:   //długość geograficzna
        if (cDaneIn == ',')
        {
            //if (cRMC_Status == 'A')
            {
                cBłąd = DecodeLonLat(cBufStanu, cBajtStanu, &dLatitudeRAW);
                if (cBłąd)
                {
                    cStan = ST_ERR;
                    cBajtStanu = 0;
                    return cBłąd;
                }
            }
            cStan = ST_RMC_LAT_NS;
            cBajtStanu = 0;
        }
        break;

    case ST_RMC_LAT_NS:
        if (cDaneIn == ',')
        {
        	if (cBajtStanu == 2)
        	{
        		if (cBufStanu[0] == 'S')
        			uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW * -1;
        		else
        			uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW;
        		cStan = ST_RMC_LONGITUD;
        	}
            else
            	cStan = ST_ERR;
            cBajtStanu = 0;
        }
        break;

    case ST_RMC_LONGITUD:   //szerokość geograficzna
        if (cDaneIn == ',')
        {
            //if (cRMC_Status == 'A')
            {
                cBłąd = DecodeLonLat(cBufStanu, cBajtStanu, &dLongitudeRAW);
                if (cBłąd)
                {
                    cStan = ST_ERR;
                    cBajtStanu = 0;
                    return cBłąd;
                }
            }
            cStan = ST_RMC_LON_WE;
            cBajtStanu = 0;
        }
        if (cBajtStanu > 12)
        {
        	cStan = ST_ERR;
        	cBajtStanu = 0;
        }
        break;

    case ST_RMC_LON_WE:
        if (cDaneIn == ',')
        {
        	if (cBajtStanu == 2)
        	{
        		if (cBufStanu[0] == 'W')
        			uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW * -1;
        		else
        			uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW;
                cStan = ST_RMC_SPEED;
        	}
        	else
        		cStan = ST_ERR;
            cBajtStanu = 0;
        }
        break;

    case ST_RMC_SPEED:
        if (cDaneIn == '.')
        {
            //if (cRMC_Status == 'A')
        	sGSpeed100k = 100 * Asci2UShort(cBufStanu+0, cBajtStanu-1);
        }
        else
       	if (cDaneIn == ',')
       	{
            if (cBajtStanu > 1)
            	sGSpeed100k += Asci2UShort(cBufStanu+cBajtStanu-3, 2);
            else
            	sGSpeed100k = 0;
            cStan = ST_RMC_COURSE;
            //fGSpeed = (float)sGSpeed100k /100;
            //zamień jednostke z węzłów = [mila/h] na [m/s]
            //fGSpeed = (float)sGSpeed100k /(float)194.38445;
            uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi = (float)sGSpeed100k *(float)0.00514444;
            cBajtStanu = 0;
       	}
        break;

    case ST_RMC_COURSE:
        if (cDaneIn == '.')
		{
			if (cRMC_Status == 'A')
				sGCourse100k = 100 * Asci2UShort(cBufStanu+0, cBajtStanu-1);
			else
				sGCourse100k = 0;
		}
		else
		if (cDaneIn == ',')
		{

            if (cBajtStanu > 1)
            	sGCourse100k += Asci2UShort(cBufStanu+cBajtStanu-3, 2);
            else
            	sGCourse100k = 0;
            uDaneCM4.dane.stGnss1.fKurs = (float)sGCourse100k/100;
            cStan = ST_RMC_DATE;
            cBajtStanu = 0;
            //wykonaj pomiar czasu od pojawienia się ostatniego kompletu danych
            cNoweDane = 2;     //mamy komplet nowych danych
            //nGpsTime = CountTime(&nLastGpsTime);
		}
        break;

    case ST_RMC_DATE:
    	if (cDaneIn == ',')
		{
    		if ((cBajtStanu >= 4) && (cRMC_Status == 'A'))
    		{
				uDaneCM4.dane.stGnss1.cDzien = Asci2UChar(cBufStanu+0, 2);
				uDaneCM4.dane.stGnss1.cMies  = Asci2UChar(cBufStanu+2, 2);
				uDaneCM4.dane.stGnss1.cRok   = Asci2UChar(cBufStanu+4, 2);
    		}
            cStan = ST_RMC_MAG_VAR;
            cBajtStanu = 0;
		}
        break;

    case ST_RMC_MAG_VAR:
        if (cDaneIn == ',')
        {
            cStan = ST_NAGLOWEK1;
            cBajtStanu = 0;
        }
        break;

    case ST_TXT:
    	if (cBajtStanu == 10)	//odbiór nieistotnych danych przed komunikatem
    	{
    		cStan = ST_TXT_OPIS;
    		cBajtStanu = 0;
    	}
    	break;

    case ST_TXT_OPIS:
    	if (cDaneIn == '*')	//czekaj na koniec napisu i kopiuj go do napisu w buforze wymiany z terminującym zerem
    	{
    		/*if (cBajtStanu < (ROZMIAR_BUF_NAPISU_WYMIANY-1))
    		{
    			uDaneCM4.dane.chNapis[cBajtStanu] = cDaneIn;
    			uDaneCM4.dane.chNapis[cBajtStanu+1] = 0;	//koniec stringu
    		}*/
    		if ((cBufStanu[0] == 'u') && (cBufStanu[1] == '-') && (cBufStanu[2] == 'b') && (cBufStanu[3] == 'l') && (cBufStanu[4] == 'o') && (cBufStanu[5] == 'x'))
    			uDaneCM4.dane.nZainicjowano |= INIT_WYKR_UBLOX;
    		cStan = ST_NAGLOWEK1;
    		cBajtStanu = 0;
    	}
    	break;


    default:
    	cStan = ST_ERR;    //błąd
    	cBłąd = BLAD_ZLY_STAN_NMEA;
    	break;
    }	//switch

    if (cBłąd == BLAD_BRAK_DANYCH)   //nie raportuj tego błędu
        cBłąd = BLAD_OK;

    return cBłąd;
}



////////////////////////////////////////////////////////////////////////////////
// zamień znaki na liczbę 8 bitową bez znaku
// Parametry: 
// *cDaneIn - wskaźnik na tablicę z danymi
// chLiczbaZnakow - ilość danych w tablicy
// Zwraca: przekonwertowaną liczbę
////////////////////////////////////////////////////////////////////////////////
uint8_t Asci2UChar(uint8_t *cDaneIn, uint8_t chLiczbaZnakow)
{
    uint8_t chLiczba = 0;

    if (chLiczbaZnakow >= 3)
	chLiczba = 100*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 2)
	chLiczba += 10*(*cDaneIn++ - '0');
    chLiczba += *cDaneIn++ - '0';

    return chLiczba;
}



////////////////////////////////////////////////////////////////////////////////
// zamień znaki na liczbę 16 bitową bez znaku
// Parametry: 
// *cDaneIn - wskaźnik na tablicę z danymi
// chLiczbaZnakow - ilość danych w tablicy
// Zwraca: przekonwertowaną liczbę
////////////////////////////////////////////////////////////////////////////////
uint16_t Asci2UShort(uint8_t *cDaneIn, uint8_t chLiczbaZnakow)
{
    uint16_t sLiczba = 0;

    if (chLiczbaZnakow >= 5)
	sLiczba = 10000*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 4)
	sLiczba += 1000*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 3)
	sLiczba += 100*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 2)
	sLiczba += 10*(*cDaneIn++ - '0');
    sLiczba += *cDaneIn++ - '0';

    return sLiczba;
}



////////////////////////////////////////////////////////////////////////////////
// zamień znaki na liczbę 32 bitową bez znaku
// Parametry: 
// *cDaneIn - wskaźnik na tablicę z danymi
// chLiczbaZnakow - ilość danych w tablicy
// Zwraca: przekonwertowaną liczbę
////////////////////////////////////////////////////////////////////////////////
unsigned long Asci2ULong(uint8_t *cDaneIn, uint8_t chLiczbaZnakow)
{
    unsigned long lLiczba = 0;

    if (chLiczbaZnakow >= 8)
	lLiczba += 1000000*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 7)
	lLiczba += 1000000*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 6)
	lLiczba += 100000*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 5)
	lLiczba += 10000*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 4)
	lLiczba += 1000*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 3)
	lLiczba += 100*(*cDaneIn++ - '0');
    if (chLiczbaZnakow >= 2)
	lLiczba += 10*(*cDaneIn++ - '0');
    lLiczba += *cDaneIn++ - '0';

    return lLiczba;
}



////////////////////////////////////////////////////////////////////////////////
// Dekoduj szerokość, lub wysokość geograficzną
// Parametry: 
// [i] *cDaneIn - wskaźnik na tablicę z danymi zakończoną przecinkiem
// [i] cLiczbaZnakow - ilość danych w tablicy
// [o] *fStopnie - liczba stopni współrzędnej
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t DecodeLonLat(uint8_t *cDaneIn, uint8_t cLiczbaZnakow, double *dStopnie)
{
    uint8_t *cBkpAdr;
    uint8_t chPoz = 0;
    
    if ((cLiczbaZnakow > 12) || (cLiczbaZnakow < 9))
	return BLAD_DLUGOSC_LONLAT;

    cBkpAdr = cDaneIn;    //kopia wskaźnika

    //znajdź pozycję przecinka w komunikacie
    while (*cBkpAdr != '.')
    {
	chPoz++;
	cBkpAdr++;
        if (chPoz > 5)
            return BLAD_DLUGOSC_LONLAT;
    }

    if ((chPoz < 3) || (chPoz > 5))   //szerokość 1-3 cyfrowa
	return BLAD_DLUGOSC_LONLAT;

    *dStopnie  = (double)Asci2UChar(cDaneIn, chPoz-2);				//2 znaki stopni
    *dStopnie += (double)Asci2UChar(cDaneIn+(chPoz-2), 2)/60;		//2 znaki minut
    *dStopnie += (double)Asci2ULong(cDaneIn+(chPoz+1), 4)/600000;	//4 znaki 10-tysiecznej części minuty

    return BLAD_OK;
}



////////////////////////////////////////////////////////////////////////////////
// Dekoduj wartość zmiennoprzecinkową taką jak DOP
// Parametry: 
// [i] *cDaneIn - wskaźnik na tablicę z danymi zakończoną przecinkiem
// [i] cLiczbaZnakow - ilość danych w tablicy
// [o] *fResult - wskaźnik na liczbę wynikową
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t DecodeFloat(uint8_t *cDaneIn, uint8_t cLiczbaZnakow, float *fResult)
{
    uint8_t x, cIn;
    int nLiczba = 0, nDzielnik = 10;

    *fResult = 0.0;
    if (cLiczbaZnakow < 2)
        return BLAD_BRAK_DANYCH;

    //przetwarzaj liczby do kropki dziesietnej
    for (x=0; x<cLiczbaZnakow; x++)
    {
        if (*(cDaneIn) == '.')
        {
            cDaneIn++;
            x++;
            break;
        }
        else
        {
            nLiczba *= 10;  
            cIn = (*cDaneIn - '0');
            if ((cIn < 0) || (cIn > 9))   //sprawdź czy to liczba
                return BLAD_ZLE_DANE;
            else
                nLiczba += cIn;
        }
        cDaneIn++;
    }
     *fResult = nLiczba;

    //przetwarzaj liczby po kropce
    for (; x<cLiczbaZnakow; x++)
    {
        if (*(cDaneIn) == ',')
            break;
        else
        {
            cIn = (*cDaneIn - '0');
            if ((cIn < 0) || (cIn > 9))   //sprawdź czy to liczba
                return BLAD_ZLE_DANE;
            else
                *fResult += (float)cIn / nDzielnik;
        }

        cDaneIn++;
        nDzielnik *= 10;  
    }
    return BLAD_OK;
}



