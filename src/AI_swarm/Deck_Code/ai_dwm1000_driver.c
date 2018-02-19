#include "ai_dwm1000_driver.h"
#include "Driver_SPI.h"


bool dwm1000_SendData(void * data, int lengthOfData /*adressen?, ...*/) {
	//1. Aufbauen der Transmit Frame f�r den SPI Bus an den DWM1000
	//2. Data auf Transmit Data Buffer Register
	//3. Transmit Frame Control aktualisieren
	//4. Sendung �berpr�fen (Timestamp abholen?, ...)
}


e_message_type_t dwm1000_ReceiveDate(void * data, int lengthOfData) {
	//1. Receive Buffer Auslesen
	//2. Art der Nachricht entschl�sseln
	//3. Auf Data schreiben
	//4. Art der Nachricht zur�ckgeben
}


float dwm1000_getDistance(double nameOfOtherDWM) {
	//1. 
	//2.
	//3.
	//4.
}


st_DWM_Config_t dwm1000_init(st_DWM_Config_t newConfig) {
	//1. f�r jeden Eintrag Register Schreiben
	//2. Alle Register lesen und in Ausgabe zusammenfassen
}


