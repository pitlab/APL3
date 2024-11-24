# APL3
Projekt eksperymentalnego autopilota z obsługą kamery do samolotów RC.

To są same początki projektu. Obecnie jest zaimplementowana nastepująca funkcjonalność:
- Sterowanie wyświetlaczem od Raspbery Pi 480x320 sterowanym po SPI.
- Odczyt i 3-punktowa kalibracja ekranu dotykowego. Do uruchomienia jeszcza kalibracja wielopunktowa.
- Zapis, odczyt i kasowanie pamięci Fash NOR. Niby działa ale jeszcze nie za każdym razem. Obecna prędkość zapisu danych to 258 kB/s, odczytu 44 MB/s.
- System zapisu konfiguracji do pamięci Flash w paczkach po 30 bajtów.
- Menu graficzne do obsługi wielu funkcjonalności, ekran powitalny z prezentacją wykrytego sprzetu.