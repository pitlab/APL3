# APL3
Projekt eksperymentalnego autopilota z obsługą kamery do samolotów RC.

To są początki projektu. Obecnie jest zaimplementowana następująca funkcjonalność:
- Sterowanie wyświetlaczem od Raspbery Pi 480x320 po SPI.
- Odczyt i 3-punktowa kalibracja ekranu dotykowego. Do uruchomienia jest jeszcza kalibracja wielopunktowa.
- Zapis, odczyt i kasowanie pamięci Fash NOR. Niby działa, ale jeszcze nie za każdym razem. Obecna prędkość zapisu danych to 258 kB/s, odczytu 45 MB/s.
- System zapisu konfiguracji do pamięci Flash w paczkach po 30 bajtów.
- Menu graficzne do obsługi wielu funkcjonalności, ekran powitalny z prezentacją wykrytego sprzętu.


Do zrobienia:
 - Dodać obsługę pamięci flash W25Q128JV po QSPI
 - Oprogramować wgrywanie próbek głosu i obrazów do zewnętrznej pamięci flash
 - Oprogramować generowanie komunikatów głosowych
 - Wlutować pamięć RAM i uruchomić komunikację z nią.
 - Podłączyć i uruchomić kamerę
 - uruchomoć komunikację po porcie LPUART
 - wlutować i uruchomić komunikację po ethernet
 - Przenieść z APL2 obsługę pętli głównej, serw, mikserów, zapisu konfiguracji do FRAM, telemetrii do rdzenia CM4
 - Oprogramować bufor wymiany komunikatów między rdzeniami
 - Uruchomić i oprogramować obsługę czujników modułu inercyjnego
 - Oprogramować obsługę karty SD po pełnym interfejsie pod kątem zapisu obrazu