//////////////////////////////////////////////////////////////////////////////
//
// AutoPitLot v3.0
// Moduł dekodowania protokołu NMEA
//
// (c) PitLab 2024
// http://www.pitlab.pl
//////////////////////////////////////////////////////////////////////////////
#include "nmea.h"
#include <stdio.h>
#include "wymiana.h"

//definicje zmiennych
uint8_t chBufStanu[BUF_STAN_SIZE];	//tablica do zbierania odbieranych danych
uint8_t chBajtStanu;
uint8_t chStan;		//stan protokołu dekodującego
uint8_t chKomunikat;	//dekodowany komunikat
int8_t chStrefaCzas = -1;       //różnica między czasem GPS a strefą czasową użytkownika
uint8_t chNoweDane;       //odebrano komplet nowych danych Lon-Lat
uint8_t chNewGAlti;     //odebarno dane o wysokości
//uint32_t nLastGpsTime, nLastGpsTimeGS;      //zmienna do pomiaru czasu między kolejnymi komunikatami
//uint32_t nGpsTime, nGpsTimeGS;          //czas między kolejnymi komunikatami RMC [us]

//CGA indeks GS oznacza zmianna dla danych pobieranych z systemu GLONASS
//uint8_t chGHour, chGMin, chGSek;
//uint8_t chGHourGS, chGMinGS, chGSekGS;
uint16_t sGMSek;
uint16_t sGMSekGS;
//uint8_t chGSats;
//uint8_t chGGAFix;
uint8_t chGGA_LatNS, chGGA_LonWE;

//GSA
uint8_t chGSAMode1;     //tryb pracy A-automatyczny, M-ręczny
//uint8_t chGFix, chGFixGS;         //rodzaj fixa: 1-brak, 2-2D, 3-3D
uint8_t chSatNumber;    //liczba satelitów
uint8_t chSatNumberGS;
float fGPDOP, fGHDOP, fGVDOP; //rozmycie precyzji

//RMC
uint8_t chRMC_Status;
uint16_t sGAltitude, sGSpeed100k, sGCourse100k,sGSpeed100kGS, sGCourse100kGS, sGAltitudeGS;
//float fGSpeed = 0, fGCourse = 0;
float fGCourseGS = 0, fGSpeedGS = 0;
//uint8_t chGDay, chGMon, chGRok;
//uint8_t chGDayGS, chGMonGS, chGRokGS;
//float fGAlti, fGAltiGS;
double dLongitudeRAW, dLatitudeRAW;	//, dLatitudeGS, dLongitudeGS;

extern volatile unia_wymianyCM4_t uDaneCM4;



////////////////////////////////////////////////////////////////////////////////
// Dekoduj ciągi znaków NMEA
// Parametry: odebrany bajt
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t DekodujNMEA(uint8_t chDaneIn)
{
    uint8_t chErr;
    chErr = BLAD_OK;

    if (chBajtStanu >= BUF_STAN_SIZE)
        chBajtStanu = 0;   

    chBufStanu[chBajtStanu] = chDaneIn;
    chBajtStanu++;
    

    switch(chStan)
    {
    case ST_NAGLOWEK1:	//czekaj na nagłówek
	if (chDaneIn == '$')
	{
	    chStan = ST_NAGLOWEK2;    
        chBajtStanu = 0;
	}
	break;

    case ST_NAGLOWEK2:	//czekaj na znaki GP badz na znaki GN
	if (chBajtStanu == 2)
	{
	    if ((chBufStanu[0] == 'G') && (chBufStanu[1] == 'P'))
	    	chStan = ST_MSGID_GP;
	    else
	    if ((chBufStanu[0] == 'G') && (chBufStanu[1] == 'N'))
	    	chStan = ST_MSGID_GN;
	    else
	    if ((chBufStanu[0] == 'P') && (chBufStanu[1] == 'M'))
	    	chStan = ST_MSGID_MTK;
	    else
	    	chStan = ST_ERR;    //błąd

	    chBajtStanu = 0;
	}
	break;
	
    case ST_MSGID_GP:  //czekaj na typ komunikatu GPS
	if (chBajtStanu == 4)	//3 znaki typu komunikatu + przecinek
	{
	    if (chBufStanu[0]== 'G')    //GGA, GLL, GSA, GSV
	    {
			if (chBufStanu[1]== 'G')	//GGx
			{
				if (chBufStanu[2]== 'A')    //GGA +
					chStan = ST_GGA_TIME;
				else
					chStan = ST_ERR;
			}
			else
					if (chBufStanu[1]== 'S')	//GSx
			{
				if (chBufStanu[2]== 'A')    //GSA +
					chStan = ST_GSA_MODE1;
				else
					chStan = ST_ERR;
			}
	    }
	    else
        if (chBufStanu[0]== 'R')    //RMC +
        {
			if (chBufStanu[1]== 'M')	//RMx
			{
				if (chBufStanu[2]== 'C')    //RMC
					chStan = ST_RMC_UTC;
				else
					chStan = ST_ERR;
			}
            else
            	chStan = ST_ERR;
	    }
        else
        	chStan = ST_ERR;
        chBajtStanu = 0;
	}
	break;

    case ST_MSGID_GN:  //czekaj na typ komunikatu GLONASS
	if (chBajtStanu == 4)	//3 znaki typu komunikatu + przecinek
	{
	    if (chBufStanu[0]== 'G')    //GGA, GLL, GSA, GSV
	    {
	    	if (chBufStanu[1]== 'S')	//GSx
			{
				if (chBufStanu[2]== 'A')    //GSA +
					chStan = ST_GSA_MODE1_GS;
				else
					chStan = ST_ERR;
			}
	    	else
	    	if (chBufStanu[1]== 'G')	//GGx
			{
				if (chBufStanu[2]== 'A')    //GGA +
					chStan = ST_GGA_TIME;
				else
					chStan = ST_ERR;
			}
			else
	    		chStan = ST_ERR;
	    }
	    else
	    if (chBufStanu[0]== 'R')    //RMC +
        {
	    	if (chBufStanu[1]== 'M')	//RMx
			{
				if (chBufStanu[2]== 'C')    //RMC
					chStan = ST_RMC_UTC_GS;
				else
					chStan = ST_ERR;
			}
	    	else
	    		chStan = ST_ERR;
	    }
	    else
	    if (chBufStanu[0]== 'T')    //TXT
	    {
	    	if (chBufStanu[1]== 'X')	//TXx
			{
				if (chBufStanu[2]== 'T')    //TXT
					chStan = ST_TXT;
				else
					chStan = ST_ERR;
			}
			else
				chStan = ST_ERR;
	    }
	    else
	    	chStan = ST_ERR;
	    chBajtStanu = 0;
	}
	break;
    //dekoduj komunikat GGA
    case ST_GGA_TIME:		//czas
        if (chDaneIn == '.')
		{
			uDaneCM4.dane.stGnss1.chGodz = Asci2UChar(chBufStanu+0, 2) - chStrefaCzas;
			uDaneCM4.dane.stGnss1.chMin  = Asci2UChar(chBufStanu+2, 2);
			uDaneCM4.dane.stGnss1.chSek  = Asci2UChar(chBufStanu+4, 2);
		}
		else
		if (chDaneIn == ',')
		{
			sGMSek  = Asci2UShort(chBufStanu+7, chBajtStanu-7-1);
			chStan = ST_GGA_LATITUDE;
			chBajtStanu = 0;
		}
		else
		if (chBajtStanu > 12)
		{
			chStan = ST_ERR;
			chBajtStanu = 0;
		}
		break;

    case ST_GGA_LATITUDE:   //długość geograficzna
        if (chDaneIn == ',')
        {
            chStan = ST_GGA_LAT_NS;
			chBajtStanu = 0;
		}
		break;


    case ST_GGA_LAT_NS:
        if (chDaneIn == ',')
		{
			chGGA_LatNS = chBufStanu[0];
            chStan = ST_GGA_LONGITUD;
            chBajtStanu = 0;
		}
		break;

    case ST_GGA_LONGITUD:   //szerokość geograficzna
        if (chDaneIn == ',')
        {
             chStan = ST_GGA_LON_WE;
			 chBajtStanu = 0;
       	}
		break;

    case ST_GGA_LON_WE:
       if (chDaneIn == ',')
       {
    	   chGGA_LonWE = chBufStanu[0];
           chStan = ST_GGA_FIX_IND;
           chBajtStanu = 0;
       }
       break;

    case ST_GGA_FIX_IND:
       if (chDaneIn == ',')
       {
    	   uDaneCM4.dane.stGnss1.chFix = chBufStanu[0] - '0';	//fix przechowuj jako liczbę a nie znak
           chStan = ST_GGA_SAT_USE;
           chBajtStanu = 0;
       }
       break;

    case ST_GGA_SAT_USE:
       if (chDaneIn == ',')
       {
    	   uDaneCM4.dane.stGnss1.chLiczbaSatelit = Asci2UChar(chBufStanu, chBajtStanu-1);
           chStan = ST_GGA_HDOP;
           chBajtStanu = 0;
       }
       break;

    case ST_GGA_HDOP:	//GPS zwraca puste pole
    	if (chDaneIn == ',')
    	{
    		 if (chBajtStanu > 3)
    			 chErr = DecodeFloat(chBufStanu, chBajtStanu-1, (float*)&uDaneCM4.dane.stGnss1.fHdop);
            chStan = ST_GGA_ALTITUDE;
            chBajtStanu = 0;
    	}
    	break;
        
    case ST_GGA_ALTITUDE: 
        if (chDaneIn == '.')
        	chBajtStanu--;  //nie analizuj kropki, liczba będzie 10x większa
        else
        if (chDaneIn == ',')
        {
            if ((chBajtStanu > 1) && (chBajtStanu < 7))
                sGAltitude = Asci2UShort(chBufStanu+0, chBajtStanu-1);
            else
                sGAltitude = 0;
            uDaneCM4.dane.stGnss1.fWysokoscMSL = (float)sGAltitude/10;
            chNewGAlti = 1; //flaga nowych danych o wysokości
            chStan = ST_NAGLOWEK1;
            chBajtStanu = 0;
        }
        break;


    //dekodowanie komunikatu GSA
    case ST_GSA_MODE1:
        if (chDaneIn == ',')
        {
        	chGSAMode1 = chBufStanu[0];
            chStan = ST_GSA_MODE2;
            chBajtStanu = 0;
        }
        break;
        
    case ST_GSA_MODE2:
        if (chDaneIn == ',')
        {
            chStan = ST_GSA_SAT_USED;
            chSatNumber = 0;
            chBajtStanu = 0;
        }
        break;

    case ST_GSA_SAT_USED:
        if (chDaneIn == ',')
        {
            chSatNumber++;
            if (chSatNumber >= 12)
            {
                chStan = ST_GSA_PDOP;
                chBajtStanu = 0;
            }

        }
        break;

    case ST_GSA_PDOP:
        if (chDaneIn == ',')
        {
            chErr = DecodeFloat(chBufStanu, chBajtStanu-1, &fGPDOP);
            if ((chErr == BLAD_OK) | (chErr == ERR_BRAK_DANYCH))
                chStan = ST_GSA_HDOP;
            else
                chStan = ST_ERR; 
            chBajtStanu = 0;
		}
        break;

    case ST_GSA_HDOP:
        if (chDaneIn == ',')
        {
            chErr = DecodeFloat(chBufStanu, chBajtStanu-1, &fGHDOP);
            uDaneCM4.dane.stGnss1.fHdop = fGHDOP;
            if ((chErr == BLAD_OK) | (chErr == ERR_BRAK_DANYCH))
                chStan = ST_GSA_VDOP;
            else
                chStan = ST_ERR;
            chBajtStanu = 0;
		}
        break;

    case ST_GSA_VDOP:
        if (chDaneIn == '*')
        {
            chErr = DecodeFloat(chBufStanu, chBajtStanu-1, &fGVDOP);             
            if ((chErr == BLAD_OK) | (chErr == ERR_BRAK_DANYCH))
                chStan = ST_NAGLOWEK1;
            else
                chStan = ST_ERR;
            chBajtStanu = 0;
        }
        break;

         //dekodowanie komunikatu GSA z systemu GLONASS
    case ST_GSA_MODE1_GS:
        if (chDaneIn == ',')
        {
        	chGSAMode1 = chBufStanu[0];
            chStan = ST_GSA_MODE2_GS;
            chBajtStanu = 0;
        }
        break;
        
    case ST_GSA_MODE2_GS:
        if (chDaneIn == ',')
        {
        	uDaneCM4.dane.stGnss1.chFix = chBufStanu[0]-'0';
            if ((uDaneCM4.dane.stGnss1.chFix < 1) || (uDaneCM4.dane.stGnss1.chFix > 3))   //sprawdź czy to liczba
            {
            	 uDaneCM4.dane.stGnss1.chFix = 1; //jeżeli błąd błąd to przyjmij najgorszy możliwy
            	 chStan = ST_ERR;
            }
            else
                chStan = ST_GSA_SAT_USED_GS;
            uDaneCM4.dane.stGnss1.chLiczbaSatelit = 0;
            chBajtStanu = 0;
        }
        break;

    case ST_GSA_SAT_USED_GS:
        if (chDaneIn == ',')
        {
        	uDaneCM4.dane.stGnss1.chLiczbaSatelit++;
            if (uDaneCM4.dane.stGnss1.chLiczbaSatelit >= 12)
            {
                chStan = ST_GSA_PDOP_GS;
                chBajtStanu = 0;
            }
        }
        break;

    case ST_GSA_PDOP_GS:
        if (chDaneIn == ',')
        {
            chErr = DecodeFloat(chBufStanu, chBajtStanu-1, &fGPDOP);
            if ((chErr == BLAD_OK) | (chErr == ERR_BRAK_DANYCH))
                chStan = ST_GSA_HDOP_GS;
            else
                chStan = ST_ERR; 
            chBajtStanu = 0;
        }
        break;

    case ST_GSA_HDOP_GS:
        if (chDaneIn == ',')
        {
            chErr = DecodeFloat(chBufStanu, chBajtStanu-1, &fGHDOP);
            if ((chErr == BLAD_OK) | (chErr == ERR_BRAK_DANYCH))
                chStan = ST_GSA_VDOP_GS;
            else
                chStan = ST_ERR;
            chBajtStanu = 0;
        }
        break;

    case ST_GSA_VDOP_GS:
        if (chDaneIn == '*')
        {
            chErr = DecodeFloat(chBufStanu, chBajtStanu-1, &fGVDOP);             
            if ((chErr == BLAD_OK) | (chErr == ERR_BRAK_DANYCH))
                chStan = ST_NAGLOWEK1;
            else
                chStan = ST_ERR;
            chBajtStanu = 0;
        }
        break;

    //dekodowanie komunikatów MTK
    case ST_MSGID_MTK:
    	if (chDaneIn == ',')
    	{
    		 if ((chBufStanu[0] == 'T') && (chBufStanu[1] == 'K') && (chBufStanu[2] == '0') && (chBufStanu[3] == '1'))
    		 {
    			 if (chBufStanu[4] == '1')
    				 chStan = ST_PMTK011;
    			 else
    			 if (chBufStanu[4] == '0')
    			     chStan = ST_PMTK010;
    			 else
    				 chStan = ST_ERR;
    		 }
    		 else
    			 chStan = ST_ERR;
    		 chBajtStanu = 0;
    	}
    	break;

    //dekodowanie komunikatu startowego chipsetu MTK: $PMTK011,MTKGPS*08<CR><LF>
    case ST_PMTK011:
    	if (chDaneIn == 0x0A)	//<LF>
    	{
    		 if ((chBufStanu[0] == 'M') && (chBufStanu[1] == 'T') && (chBufStanu[2] == 'K') && (chBufStanu[3] == 'G') && (chBufStanu[4] == 'P') && (chBufStanu[5] == 'S'))
    			 chStan = ST_PMTK010;
    		 else
    			 chStan = ST_ERR;
    		 chBajtStanu = 0;
    	}
    	break;

    //Dekodowanie komunikatu kończącego inicjalizację modułu: $PMTK010,001*2E<CR><LF>
    case ST_PMTK010:
    	const uint8_t chWzorzec[] =  "001*2E";
    	if (chDaneIn == 0x0A)	//<LF>
    	{
			for (uint8_t n=0; n<6; n++)
			{
				if (chBufStanu[n] != chWzorzec[n])
				{
					chStan = ST_ERR;
					chBajtStanu = 0;
					break;
				}
			}
    		uDaneCM4.dane.nZainicjowano |= INIT_WYKR_MTK;
    		chStan = ST_NAGLOWEK1;
    		chBajtStanu = 0;
    	}
    	break;

    //dekodowanie komunikatu RMC Recommended Minimum GLONASS
    case ST_RMC_UTC_GS:
    	if (chDaneIn == '.')
    	{
        	  uDaneCM4.dane.stGnss1.chGodz = Asci2UChar(chBufStanu+0, 2) - chStrefaCzas;
        	  uDaneCM4.dane.stGnss1.chMin  = Asci2UChar(chBufStanu+2, 2);
        	  uDaneCM4.dane.stGnss1.chSek  = Asci2UChar(chBufStanu+4, 2);
    	}
		else
		if (chDaneIn == ',')
		{
			sGMSek  = Asci2UShort(chBufStanu+7, chBajtStanu-7-1);
			chStan = ST_RMC_STATUS_GS;
			chBajtStanu = 0;
		}
		else
		if (chBajtStanu > 12)
		{
			chStan = ST_ERR;
			chBajtStanu = 0;
		}
		break;

    case ST_RMC_STATUS_GS:	    //status A-dane ważne, V-dane nieważne
        if (chDaneIn == ',')
        {
        	if (chBajtStanu == 2)
        	{
        		chRMC_Status = chBufStanu[0];
        		chStan = ST_RMC_LATITUDE_GS;
        	}
        	else
        		chStan = ST_ERR;
        	chBajtStanu = 0;
        }
        break;

    case ST_RMC_LATITUDE_GS:   //długość geograficzna
        if (chDaneIn == ',')
        {
            if (chRMC_Status == 'A')
            {
                chErr = DecodeLonLat(chBufStanu, chBajtStanu, &dLatitudeRAW);
                if (chErr)
                {
                    chStan = ST_ERR;
                    chBajtStanu = 0;
                    return chErr;
                }
            }
            chStan = ST_RMC_LAT_NS_GS;
            chBajtStanu = 0;
        }
        break;

    case ST_RMC_LAT_NS_GS:
        if (chDaneIn == ',')
		{
			if (chBajtStanu == 2)
			{
				if (chBufStanu[0] == 'S')
					uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW * -1;
				else
					uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW;
			}
			chStan = ST_RMC_LONGITUD_GS;
            chBajtStanu = 0;
		}
        break;

    case ST_RMC_LONGITUD_GS:   //szerokość geograficzna
        if (chDaneIn == ',')
        {
            if (chRMC_Status == 'A')
            {
                chErr = DecodeLonLat(chBufStanu, chBajtStanu, &dLongitudeRAW);
                if (chErr)
                {
                    chStan = ST_ERR;
                    chBajtStanu = 0;
                    return chErr;
                }
            }
            chStan = ST_RMC_LON_WE_GS;
            chBajtStanu = 0;
        }
        if (chBajtStanu > 12)
        {
        	chStan = ST_ERR;
        	chBajtStanu = 0;
        }
        break;

    case ST_RMC_LON_WE_GS:
        if (chDaneIn == ',')
		{
			if (chBajtStanu == 2)
			{
				if (chBufStanu[0] == 'W')
					uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW * -1;
				else
					uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW;
			}
			chStan = ST_RMC_SPEED_GS;
			chBajtStanu = 0;
		}
		break;

    case ST_RMC_SPEED_GS:
        if (chDaneIn == '.')
		{
			sGSpeed100k = 100 * Asci2UShort(chBufStanu+0, chBajtStanu-1);
		}
		else
		if (chDaneIn == ',')
		{
			if (chBajtStanu > 1)
				sGSpeed100k += Asci2UShort(chBufStanu+chBajtStanu-3, 2);
			else
				sGSpeed100k = 0;
			//zamień jednostke z węzłów = [mila/h] na [m/s]
			uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi = (float)sGSpeed100k *(float)0.00514444;
			//chStan = ST_RMC_COURSE_GS;
			chStan = ST_RMC_COURSE;
			chBajtStanu = 0;
		}
		break;

    /*case ST_RMC_COURSE_GS:
        if (chDaneIn == '.')
        {
        	sGCourse100k = 100 * Asci2UShort(chBufStanu+0, chBajtStanu-1);
        }
        else
        if (chDaneIn == ',')
        {
            if (chBajtStanu > 1)
            	sGCourse100k += Asci2UShort(chBufStanu+chBajtStanu-3, 2);
            else
            	sGCourse100kGS = 0;
            uDaneCM4.dane.stGnss1.fKurs = (float)sGCourse100k/100;
            chStan = ST_RMC_DATE;
            chBajtStanu = 0;
            //wykonaj pomiar czasu od pojawienia się ostatniego kompletu danych
            chNoweDane = 2;     //mamy komplet nowych danych
            //nGpsTime = CountTime(&nLastGpsTime);
        }
        break; */

    case ST_RMC_UTC:
    	if (chDaneIn == '.')
		{
    		uDaneCM4.dane.stGnss1.chGodz = Asci2UChar(chBufStanu+0, 2) - chStrefaCzas;
    		uDaneCM4.dane.stGnss1.chMin  = Asci2UChar(chBufStanu+2, 2);
    		uDaneCM4.dane.stGnss1.chSek  = Asci2UChar(chBufStanu+4, 2);
		}
		else
        if (chDaneIn == ',')
		{
			sGMSek  = Asci2UShort(chBufStanu+7, chBajtStanu-7-1);
			chStan = ST_RMC_STATUS;
			chBajtStanu = 0;
		}
		else
		if (chBajtStanu > 12)
		{
			chStan = ST_ERR;
			chBajtStanu = 0;
		}
		break;

    case ST_RMC_STATUS:	    //status A-dane ważne, V-dane nieważne
        if (chDaneIn == ',')
		{
			if (chBajtStanu == 2)
			{
				chRMC_Status = chBufStanu[0];
				chStan = ST_RMC_LATITUDE;
			}
			else
				chStan = ST_ERR;
            chBajtStanu = 0;
		}
		break;

    case ST_RMC_LATITUDE:   //długość geograficzna
        if (chDaneIn == ',')
        {
            //if (chRMC_Status == 'A')
            {
                chErr = DecodeLonLat(chBufStanu, chBajtStanu, &dLatitudeRAW);
                if (chErr)
                {
                    chStan = ST_ERR;
                    chBajtStanu = 0;
                    return chErr;
                }
            }
            chStan = ST_RMC_LAT_NS;
            chBajtStanu = 0;
        }
        break;

    case ST_RMC_LAT_NS:
        if (chDaneIn == ',')
        {
        	if (chBajtStanu == 2)
        	{
        		if (chBufStanu[0] == 'S')
        			uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW * -1;
        		else
        			uDaneCM4.dane.stGnss1.dSzerokoscGeo = dLatitudeRAW;
        		chStan = ST_RMC_LONGITUD;
        	}
            else
            	chStan = ST_ERR;
            chBajtStanu = 0;
        }
        break;

    case ST_RMC_LONGITUD:   //szerokość geograficzna
        if (chDaneIn == ',')
        {
            //if (chRMC_Status == 'A')
            {
                chErr = DecodeLonLat(chBufStanu, chBajtStanu, &dLongitudeRAW);
                if (chErr)
                {
                    chStan = ST_ERR;
                    chBajtStanu = 0;
                    return chErr;
                }
            }
            chStan = ST_RMC_LON_WE;
            chBajtStanu = 0;
        }
        if (chBajtStanu > 12)
        {
        	chStan = ST_ERR;
        	chBajtStanu = 0;
        }
        break;

    case ST_RMC_LON_WE:
        if (chDaneIn == ',')
        {
        	if (chBajtStanu == 2)
        	{
        		if (chBufStanu[0] == 'W')
        			uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW * -1;
        		else
        			uDaneCM4.dane.stGnss1.dDlugoscGeo = dLongitudeRAW;
                chStan = ST_RMC_SPEED;                
        	}
        	else
        		chStan = ST_ERR;
            chBajtStanu = 0;
        }
        break;

    case ST_RMC_SPEED:
        if (chDaneIn == '.')
        {
            //if (chRMC_Status == 'A')
        	sGSpeed100k = 100 * Asci2UShort(chBufStanu+0, chBajtStanu-1);
        }
        else
       	if (chDaneIn == ',')
       	{
            if (chBajtStanu > 1)
            	sGSpeed100k += Asci2UShort(chBufStanu+chBajtStanu-3, 2);
            else
            	sGSpeed100k = 0;
            chStan = ST_RMC_COURSE;
            //fGSpeed = (float)sGSpeed100k /100;
            //zamień jednostke z węzłów = [mila/h] na [m/s]
            //fGSpeed = (float)sGSpeed100k /(float)194.38445;
            uDaneCM4.dane.stGnss1.fPredkoscWzglZiemi = (float)sGSpeed100k *(float)0.00514444;
            chBajtStanu = 0;
       	}
        break;

    case ST_RMC_COURSE:
        if (chDaneIn == '.')
		{
			if (chRMC_Status == 'A')
				sGCourse100k = 100 * Asci2UShort(chBufStanu+0, chBajtStanu-1);
			else
				sGCourse100k = 0;
		}
		else
		if (chDaneIn == ',')
		{

            if (chBajtStanu > 1)
            	sGCourse100k += Asci2UShort(chBufStanu+chBajtStanu-3, 2);
            else
            	sGCourse100k = 0;
            uDaneCM4.dane.stGnss1.fKurs = (float)sGCourse100k/100;
            chStan = ST_RMC_DATE;
            chBajtStanu = 0;
            //wykonaj pomiar czasu od pojawienia się ostatniego kompletu danych
            chNoweDane = 2;     //mamy komplet nowych danych
            //nGpsTime = CountTime(&nLastGpsTime);
		}
        break;

    case ST_RMC_DATE:
    	if (chDaneIn == ',')
		{
    		if (chBajtStanu >= 4)
    		{
				uDaneCM4.dane.stGnss1.chDzien = Asci2UChar(chBufStanu+0, 2);
				uDaneCM4.dane.stGnss1.chMies  = Asci2UChar(chBufStanu+2, 2);
				uDaneCM4.dane.stGnss1.chRok    = Asci2UChar(chBufStanu+4, 2);
    		}
            chStan = ST_RMC_MAG_VAR;
            chBajtStanu = 0;
		}
        break;

    case ST_RMC_MAG_VAR:
        if (chDaneIn == ',')
        {
            chStan = ST_NAGLOWEK1;
            chBajtStanu = 0;
        }
        break;

    case ST_TXT:
    	if (chBajtStanu == 10)	//odbiór nieistotnych danych przed komunikatem
    	{
    		chStan = ST_TXT_OPIS;
    		chBajtStanu = 0;
    	}
    	break;

    case ST_TXT_OPIS:
    	if (chDaneIn == '*')	//czekaj na koniec napisu i kopiuj go do napisu w buforze wymiany z terminującym zerem
    	{
    		/*if (chBajtStanu < (ROZMIAR_BUF_NAPISU_WYMIANY-1))
    		{
    			uDaneCM4.dane.chNapis[chBajtStanu] = chDaneIn;
    			uDaneCM4.dane.chNapis[chBajtStanu+1] = 0;	//koniec stringu
    		}*/
    		if ((chBufStanu[0] == 'u') && (chBufStanu[1] == '-') && (chBufStanu[2] == 'b') && (chBufStanu[3] == 'l') && (chBufStanu[4] == 'o') && (chBufStanu[5] == 'x'))
    			uDaneCM4.dane.nZainicjowano |= INIT_WYKR_UBLOX;
    		chStan = ST_NAGLOWEK1;
    		chBajtStanu = 0;
    	}
    	break;


    default:
    	chStan = ST_ERR;    //błąd
    	chErr = ERR_ZLY_STAN_NMEA;
    	break;
    }	//switch

    if (chErr == ERR_BRAK_DANYCH)   //nie raportuj tego błędu
        chErr = BLAD_OK;

    return chErr;
}



////////////////////////////////////////////////////////////////////////////////
// zamień znaki na liczbę 8 bitową bez znaku
// Parametry: 
// *chDaneIn - wskaźnik na tablicę z danymi
// chLiczbaZnakow - ilość danych w tablicy
// Zwraca: przekonwertowaną liczbę
////////////////////////////////////////////////////////////////////////////////
uint8_t Asci2UChar(uint8_t *chDaneIn, uint8_t chLiczbaZnakow)
{
    uint8_t chLiczba = 0;

    if (chLiczbaZnakow >= 3)
	chLiczba = 100*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 2)
	chLiczba += 10*(*chDaneIn++ - '0');
    chLiczba += *chDaneIn++ - '0';

    return chLiczba;
}


////////////////////////////////////////////////////////////////////////////////
// zamień znaki na liczbę 16 bitową bez znaku
// Parametry: 
// *chDaneIn - wskaźnik na tablicę z danymi
// chLiczbaZnakow - ilość danych w tablicy
// Zwraca: przekonwertowaną liczbę
////////////////////////////////////////////////////////////////////////////////
uint16_t Asci2UShort(uint8_t *chDaneIn, uint8_t chLiczbaZnakow)
{
    uint16_t sLiczba = 0;

    if (chLiczbaZnakow >= 5)
	sLiczba = 10000*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 4)
	sLiczba += 1000*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 3)
	sLiczba += 100*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 2)
	sLiczba += 10*(*chDaneIn++ - '0');
    sLiczba += *chDaneIn++ - '0';

    return sLiczba;
}



////////////////////////////////////////////////////////////////////////////////
// zamień znaki na liczbę 32 bitową bez znaku
// Parametry: 
// *chDaneIn - wskaźnik na tablicę z danymi
// chLiczbaZnakow - ilość danych w tablicy
// Zwraca: przekonwertowaną liczbę
////////////////////////////////////////////////////////////////////////////////
unsigned long Asci2ULong(uint8_t *chDaneIn, uint8_t chLiczbaZnakow)
{
    unsigned long lLiczba = 0;

    if (chLiczbaZnakow >= 8)
	lLiczba += 1000000*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 7)
	lLiczba += 1000000*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 6)
	lLiczba += 100000*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 5)
	lLiczba += 10000*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 4)
	lLiczba += 1000*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 3)
	lLiczba += 100*(*chDaneIn++ - '0');
    if (chLiczbaZnakow >= 2)
	lLiczba += 10*(*chDaneIn++ - '0');
    lLiczba += *chDaneIn++ - '0';

    return lLiczba;
}


////////////////////////////////////////////////////////////////////////////////
// Dekoduj szerokość, lub wysokość geograficzną
// Parametry: 
// [i] *chDaneIn - wskaźnik na tablicę z danymi zakończoną przecinkiem
// [i] chLiczbaZnakow - ilość danych w tablicy
// [o] *fStopnie - liczba stopni współrzędnej
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t DecodeLonLat(uint8_t *chDaneIn, uint8_t chLiczbaZnakow, double *dStopnie)
{
    uint8_t *chBkpAdr;
    uint8_t chPoz = 0;
    
    if ((chLiczbaZnakow > 12) || (chLiczbaZnakow < 9))
	return ERR_DLUGOSC_LONLAT;

    chBkpAdr = chDaneIn;    //kopia wskaźnika

    //znajdź pozycję przecinka w komunikacie
    while (*chBkpAdr != '.')
    {
	chPoz++;
	chBkpAdr++;
        if (chPoz > 5)
            return ERR_DLUGOSC_LONLAT;
    }

    if ((chPoz < 3) || (chPoz > 5))   //szerokość 1-3 cyfrowa
	return ERR_DLUGOSC_LONLAT;

    *dStopnie  = (double)Asci2UChar(chDaneIn, chPoz-2);
    *dStopnie += (double)Asci2UChar(chDaneIn+(chPoz-2), 2)/60;
    *dStopnie += (double)Asci2ULong(chDaneIn+(chPoz+1), 4)/600000;

    return BLAD_OK;
}


////////////////////////////////////////////////////////////////////////////////
// Dekoduj wartość zmiennoprzecinkową taką jak DOP
// Parametry: 
// [i] *chDaneIn - wskaźnik na tablicę z danymi zakończoną przecinkiem
// [i] chLiczbaZnakow - ilość danych w tablicy
// [o] *fResult - wskaźnik na liczbę wynikową
// Zwraca: kod błedu
////////////////////////////////////////////////////////////////////////////////
uint8_t DecodeFloat(uint8_t *chDaneIn, uint8_t chLiczbaZnakow, float *fResult)
{
    uint8_t x, chIn;
    int nLiczba = 0, nDzielnik = 10;

    *fResult = 0.0;
    if (chLiczbaZnakow < 2)
        return ERR_BRAK_DANYCH;

    //przetwarzaj liczby do kropki dziesietnej
    for (x=0; x<chLiczbaZnakow; x++)
    {
        if (*(chDaneIn) == '.')
        {
            chDaneIn++;
            x++;
            break;
        }
        else
        {
            nLiczba *= 10;  
            chIn = (*chDaneIn - '0');
            if ((chIn < 0) || (chIn > 9))   //sprawdź czy to liczba
                return ERR_ZLE_DANE;
            else
                nLiczba += chIn;
        }
        chDaneIn++;
    }
     *fResult = nLiczba;

    //przetwarzaj liczby po kropce
    for (; x<chLiczbaZnakow; x++)
    {
        if (*(chDaneIn) == ',')
            break;
        else
        {
            chIn = (*chDaneIn - '0');
            if ((chIn < 0) || (chIn > 9))   //sprawdź czy to liczba
                return ERR_ZLE_DANE;
            else
                *fResult += (float)chIn/nDzielnik;
              //fLiczba += (float)chIn/nDzielnik;

        }

        chDaneIn++;
        nDzielnik *= 10;  
    }

     //*fResult = fLiczba;
    return BLAD_OK;
}



