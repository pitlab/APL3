# APL3
Projekt eksperymentalnego autopilota dla samolotów RC z obsługą kamery cyfrowej.

To są początki projektu. Obecnie jest zaimplementowana następująca funkcjonalność:
Rdzeń CM7:
- Sterowanie wyświetlaczem od Raspbery Pi 480x320 po SPI.
- Odczyt i 3-punktowa kalibracja ekranu dotykowego. Do uruchomienia jest jeszcza kalibracja wielopunktowa.
- Menu graficzne do obsługi wielu funkcjonalności, ekran powitalny z prezentacją wykrytego sprzętu.
- Obsługa pamieci Flash NOR S29GL256S90 na magistrali równoległej 16-bit
- Obsługa pamięci Flash W25Q128JV na magistrali QSPI
- Obsługa zewnętrznej pamięci statycznej IS61WV204816BLL na magistrali równoległej 16-bit
- Testy pomiaru transferu dla wszystkich pamięci
- System zapisu konfiguracji do pamięci Flash w paczkach po 30 bajtów + 2 bajty ID i sumy kontrolnej
- Wymiana komunikatów z rdzeniem CM4 i prezentacja ich na LCD

Rdzeń CM4:
- Obsługa pamięci FRAM
- Rozpoczęta obsługa expandera IO
- Wymiana komunikatów z rdzeniem CM7

Do zrobienia:
 - Dodać obsługę pamięci SDRAM, kupić właściwy układ
 - Uruchomić zewnętrzne rezonatory kwarcowe, wymienić kwarc na inny, dobrać inne pojemności bo nie działa z 12pF i 0pF
 - Uruchomić i oprogramować obsługę czujników modułu inercyjnego
 - Oprogramować wgrywanie próbek głosu i obrazów do zewnętrznej pamięci flash
 - Oprogramować generowanie komunikatów głosowych
 - Podłączyć i uruchomić kamerę
 - uruchomoć komunikację po porcie LPUART
 - wlutować i uruchomić komunikację po ethernet
 - Przenieść z APL2 obsługę pętli głównej, serw, mikserów, zapisu konfiguracji do FRAM, telemetrii do rdzenia CM4
 - Oprogramować obsługę karty SD po pełnym interfejsie pod kątem zapisu obrazu
 - Uruchomić układ ethernet
 