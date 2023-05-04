
# BMI270 sensor API for zephyr

Jest to projekt oprogramowania umożliwiającego konfigurację oraz obsługę czujnika BMI270 od  Bosch Sensortec w środowisku zephyr z wykorzystaniem interfejsu SPI. Oprogramowanie dedykowane jest dla układu nRF5340 DK od Nordic Semiconductor. 

## Kompilacja projektu
Pliki bmi270_api.c oraz bmi270_api.h umieszczamy w folderze /src w naszej aplikacji.

W pliku CMakeLists.txt podajemy plik bmi270_api.c, aby został skompilowany.

np. target_sources(app PRIVATE src/main.c src/bmi270_api.c)

## Inicjalizacja czujnika 

 - konfiguracja pinów (plik .overlay)
 
 W głównym folderze projektu tworzymy plik z rozszerzeniem .overlay i dodajemy urządzenie bmi270 do devicetree podłączając je do urządzenia SPI.
 
W przykładzie podłączono do arduino_spi.
 - konfiguracja modułów kompilacyjnych w projekcie (plik .conf)
 
 W głównym folderze projektu, w pliku prj.conf dodajemy następujące moduły:

 `CONFIG_GPIO=y`

`CONFIG_SPI=y`

`CONFIG_SENSOR=y`




 - Inicjalizacja czujnika w pliku .c
 
 Po wykonaniu uprzedniej konfiguracji możemy zainicjalizować czujnik w kodzie źródłowym korzystając z funkcji:
 
bmi270_init() 

## Konfiguracja czujnika - żyroskop

 - konfiguracja Output Data Rate (ODR), full scale, oversampling

 - konfiguracja filtrów

 - konfiguracja FIFO

 - konfiguracja przerwań

 - konfiguracja power mode


## Konfiguracja czujnika - akcelerometr

 - konfiguracja Output Data Rate (ODR), full scale, oversampling

 - konfiguracja filtrów

 - konfiguracja FIFO

 - konfiguracja przerwań

 - konfiguracja power mode



## Przykład użycia

Przykład użycia dostępny w folderze examples
