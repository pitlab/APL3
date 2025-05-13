//definicje zmiennych strojenia PID�w wolnym kana�em aparatury
#define TUNE_NOTHING    0   //brak wyboru zmiennej do strojenia
#define TUNE_ROLL_RATE_KP   1   //strojone jest wzmocnienie regulatora pochodnej k�ta przechylenia
#define TUNE_ROLL_RATE_TI   2   //strojony jest czas zdwojenia regulatora pochodnej k�ta przechylenia
#define TUNE_ROLL_RATE_TD   3   //strojony jest czas r�niczkowania regulatora pochodnej k�ta przechylenia
#define TUNE_ROLL_RATE_IL   4   //strojony jest limit ca�ki regulatora pochodnej k�ta przechylenia
#define TUNE_ROLL_STAB_KP   5   //strojone jest wzmocnienie regulatora k�ta przechylenia
#define TUNE_ROLL_STAB_TI   6   //strojony jest czas zdwojenia regulatora k�ta przechylenia
#define TUNE_ROLL_STAB_TD   7   //strojony jest czas r�niczkowania regulatora k�ta przechylenia
#define TUNE_ROLL_STAB_IL   8   //strojony jest limit ca�ki regulatora k�ta przechylenia

#define TUNE_PITCH_RATE_KP  9   //strojone jest wzmocnienie regulatora pochodnej k�ta pochylenia
#define TUNE_PITCH_RATE_TI  10  //strojony jest czas zdwojenia regulatora pochodnej k�ta pochylenia
#define TUNE_PITCH_RATE_TD  11  //strojony jest czas r�niczkowania regulatora pochodnej k�ta pochylenia
#define TUNE_PITCH_RATE_IL  12  //strojony jest limit ca�ki regulatora pochodnej k�ta pochylenia
#define TUNE_PITCH_STAB_KP  13  //strojone jest wzmocnienie regulatora k�ta pochylenia
#define TUNE_PITCH_STAB_TI  14  //strojony jest czas zdwojenia regulatora k�ta pochylenia
#define TUNE_PITCH_STAB_TD  15  //strojony jest czas r�niczkowania regulatora k�ta pochylenia
#define TUNE_PITCH_STAB_IL  16  //strojony jest limit ca�ki regulatora k�ta pochylenia

#define TUNE_YAW_RATE_KP    17  //strojone jest wzmocnienie regulatora pochodnej k�ta odchylenia
#define TUNE_YAW_RATE_TI    18  //strojony jest czas zdwojenia regulatora pochodnej k�ta odchylenia
#define TUNE_YAW_RATE_TD    19  //strojony jest czas r�niczkowania regulatora pochodnej k�ta odchylenia
#define TUNE_YAW_RATE_IL    20  //strojony jest limit ca�ki regulatora pochodnej k�ta odchylenia
#define TUNE_YAW_STAB_KP    21  //strojone jest wzmocnienie regulatora k�ta odchylenia
#define TUNE_YAW_STAB_TI    22  //strojony jest czas zdwojenia regulatora k�ta odchylenia
#define TUNE_YAW_STAB_TD    23  //strojony jest czas r�niczkowania regulatora k�ta odchylenia
#define TUNE_YAW_STAB_IL    24  //strojony jest limit ca�ki regulatora k�ta odchylenia

#define TUNE_ALTI_RATE_KP   25   //strojone jest wzmocnienie regulatora pr�dko�ci pionowej
#define TUNE_ALTI_RATE_TI   26   //strojony jest czas zdwojenia regulatora pr�dko�ci pionowej
#define TUNE_ALTI_RATE_TD   27   //strojony jest czas r�niczkowania regulatora pr�dko�ci pionowej
#define TUNE_ALTI_RATE_IL   28   //strojony jest limit ca�ki regulatora pr�dko�ci pionowej
#define TUNE_ALTI_STAB_KP   29   //strojone jest wzmocnienie regulatora wysoko�ci
#define TUNE_ALTI_STAB_TI   30   //strojony jest czas zdwojenia regulatora wysoko�ci
#define TUNE_ALTI_STAB_TD   31   //strojony jest czas r�niczkowania regulatora wysoko�ci
#define TUNE_ALTI_STAB_IL   32   //strojony jest limit ca�ki regulatora wysoko�ci

#define TUNE_GPS_SPD_KP     33   //strojona jest wzmocnienie regulatora  pr�dko�ci GPS
#define TUNE_GPS_SPD_TI     34   //strojona jest czas zdwojenia regulatora  pr�dko�ci GPS
#define TUNE_GPS_SPD_TD     35   //strojona jest czas r�niczkowania pr�dko�ci GPS
#define TUNE_GPS_SPD_IL     36   //strojona jest limit ca�ki regulatora pr�dko�ci GPS
#define TUNE_GPS_POS_KP     37   //strojona jest wzmocnienie regulatora pozycji GPS
#define TUNE_GPS_POS_TI     38   //strojona jest czas zdwojenia regulatora pozycji GPS
#define TUNE_GPS_POS_TD     39   //strojona jest czas r�niczkowania regulatora pozycji GPS
#define TUNE_GPS_POS_IL     40   //strojona jest limit ca�ki regulatora pozycji GPS

#define TUNE_ROLL_RATE_DF   41   //strojona jest g��boko�� r�niczkowania
#define TUNE_ROLL_STAB_DF   42   //strojona jest g��boko�� r�niczkowania
#define TUNE_PITCH_RATE_DF  43   //strojona jest g��boko�� r�niczkowania
#define TUNE_PITCH_STAB_DF  44   //strojona jest g��boko�� r�niczkowania
#define TUNE_YAW_RATE_DF    45   //strojona jest g��boko�� r�niczkowania
#define TUNE_YAW_STAB_DF    46   //strojona jest g��boko�� r�niczkowania
#define TUNE_ALTI_RATE_DF   47   //strojona jest g��boko�� r�niczkowania
#define TUNE_ALTI_STAB_DF   48   //strojona jest g��boko�� r�niczkowania

#define  TUNE_HOVER_PPM     49  //strojony jest ci�g pozwalaj�cy na zerow� p�ywalno��.

#define TUNE_END            49
  