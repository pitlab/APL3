# APL3
### Projekt eksperymentalnego autopilota dla samolotów RC z obsługą kamery cyfrowej.

To są początki projektu. Obecnie jest zaimplementowana następująca funkcjonalność:
### Rdzeń CM7:
- Sterowanie wyświetlaczem od Raspbery Pi 480x320 po SPI.
- Odczyt i 3-punktowa kalibracja ekranu dotykowego. Do uruchomienia jest jeszcza kalibracja wielopunktowa.
- Menu graficzne do obsługi wielu funkcjonalności, ekran powitalny z prezentacją wykrytego sprzętu.
- Obsługa pamieci Flash NOR S29GL256S90 na magistrali równoległej 16-bit
- Obsługa pamięci Flash W25Q128JV na magistrali QSPI
- Obsługa zewnętrznej pamięci statycznej IS61WV204816BLL na magistrali równoległej 16-bit
- Testy pomiaru transferu dla wszystkich pamięci
- System zapisu konfiguracji do pamięci Flash w paczkach po 30 bajtów + 2 bajty ID i sumy kontrolnej
- Wymiana komunikatów z rdzeniem CM4 i prezentacja ich na LCD

### Rdzeń CM4:
- Obsługa pamięci FRAM
- Rozpoczęta obsługa expandera IO
- Wymiana komunikatów z rdzeniem CM7
- Sterowanie pętla główną programu, podział czasu procesora na odcinki czasowe

#### Do zrobienia:
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
 
 
 
 ## Zarządzanie pamiecią
 								Pozwolenia dla MPU
Adres		Rozm	CPU		Instr	Share	Cache	Buffer		Nazwa			Zastosowanie
0x00000000	64K		CM7				 				 			ITCMRAM			
0x08000000	1024K	CM7		+		-		+		-			FLASH			kod programu dla CM7
0x08100000	1024K	CM4		+		-		+		-			FLASH			kod programu dla CM4
0x20000000	128K	CM7											DTCMRAM			
0x24000000	512K	CM7		-		-		-		+			SRAM_AXI_D1		stos i dane dla CM7
0x30000000  128K	CM4		-		+		-		+			SRAM1_AHB_D2	stos i dane dla CM4
0x30020000	128K	CM7		-		-		-		-   		SRAM2_AHB_D2	bufory ethernet
0x30040000	32K		CM7		-		+		+		+  			SRAM3_AHB_D2
0x38000000	64K		CM4+7	-		+		-		-			SRAM4_AHB_D3	współdzielenie danych między rdzeniami, sterowane HSEM1 i HSEM2
0x38800000	4K		CM7											BACKUP
0x60000000	4M		CM7		-		+		-		+			EXT_SRAM		bufor obrazu z kamery
0x68000000	32M		CM7		+		+		+		-			FLASH_NOR		
  
    		
  		