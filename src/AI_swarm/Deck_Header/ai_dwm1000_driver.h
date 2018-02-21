//beinhaltet die Funktionen, welche ben�tigt werden um den DWM1000 zu steuern (Daten Senden, Ranging, ...)

#include "ai_datatypes.h"
#include <stdbool.h>


bool setup_dwm1000_spi_interface();
/*
initialisiert das SPI interface (nach Anleitung durch "stm32f4xx_spi.c" - Z.17 - Z.134)
*/

bool dwm1000_SendData(void * data, int lengthOfData /*adressen?, ...*/);
/*
sendet Daten �ber den UWB-Bus

data - bytehaufen, der gesendet wird
lengthOfData - L�nge der Daten in byte
(return true, wenn gegl�ckt)
*/


e_message_type_t dwm1000_ReceiveData(void * data, int lengthOfData);
/*
liest Daten im dwm1000 Receivebuffer

data - stelle, an die die Daten geschrieben werden k�nnen
lengthOfData - L�nge der Daten die vom Receivebuffer gelesen werden sollen
(return true, wenn gegl�ckt)
*/

float dwm1000_getDistance(double nameOfOtherDWM);
/*
startet Rangingvorgang

nameOfOtherDWM - PAN (Personal Area Network) Identifier (8Byte) des anderen DWMs (Register 0x03)
*/

st_DWM_Config_t dwm1000_init(st_DWM_Config_t newConfig);
/*
beschreibt alle Register mit den in "newConfig" f�r diese enthaltenen Werten

newConfig - Konfigurationsstrukt, das die Werte der Register enth�lt
return die Werte der aktuellen Konfiguration des DWM1000s um Pr�fen zu k�nnen ob init gegl�ckt
*/
