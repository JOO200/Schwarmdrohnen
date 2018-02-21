#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../../../vendor/CMSIS/CMSIS/Driver/Include/Driver_SPI.h"
#include "stm32f4xx_spi.h"
#include "deck_spi.h"
#include "../ai_task.h"

#define BaudRate SPI_BAUDRATE_21MHZ	//hier auch 11.5, 5.25, 2.625, 1.3125 ausw�hlbar

//hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)

//helper Functions:
void spiStart(){
	spiBeginTransaction(BaudRate);
}

void spiStop(){
	spiEndTransaction();
}


bool setup_dwm1000_spi_interface(){
	spiBegin();		//call der Funktion aus "deck_spi.c""


	st_DWM_Config_t config;

	//hier config ausf�llen



}

bool dwm1000_SendData(void * data, int lengthOfData /*adressen?, ...*/) {
	//1. Aufbauen der Transmit Frame f�r den SPI Bus an den DWM1000
	//2. Data auf Transmit Data Buffer Register
	//3. Transmit Frame Control aktualisieren
	//4. Sendung �berpr�fen (Timestamp abholen?, ...)
}


e_message_type_t dwm1000_ReceiveData(void * data, int lengthOfData) {
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

	spiStart();	//f�r Mutexinteraktion genutzt	
	
	// -------- Init SPI --------

	//Baud Rate
		//Bausrate Nachricht aus newConfig erstellen

	spiExchange(0,0,0);



	// -------- Init UWB --------

	//PAN Identifier
		//Identifeier Nachricht aus newConfig erstellen

	spiExchange(0,0,0);



	spiStop();	//f�r Mutexinteraktion genutzt	
}


void __attribute__((used)) EXTI4_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line4);
	DWM1000_IRQ_ISR();	//ISR in ai_task.c
}



