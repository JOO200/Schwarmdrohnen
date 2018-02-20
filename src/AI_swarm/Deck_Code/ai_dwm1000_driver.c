#include "ai_dwm1000_driver.h"
#include "Driver_SPI.h"
#include "stm32f4xx_spi.h"
#include "deck_spi.h"


#define READ_TFC //Instruction Manual S.12

#define DWMINIT_TFC_TFLEN 0b0001100		//Orientierung: Bit  6, Bit 5, ... , Bit 0 etc.
#define DWMINIT_TFC_TFLE 0b000		//Orientierung: Bit  9, Bit 8, Bit 7 
#define DWMINIT_TFC_R 0b000		
#define DWMINIT_TFC_TXBR 0b01		// hier �ndern f�r 850 kbps
#define DWMINIT_TFC_TR 0b0	
#define DWMINIT_TFC_TXPRF 0b01
#define DWMINIT_TFC_TXPSR 0b01
#define DWMINIT_TFC_PE 0b01
#define DWMINIT_TFC_TXBOFFS 0b0000000000
#define DWMINIT_TFC_IFSDELAY 0b00000000			//Zusammensetzung: IFSDELAY+TXBOFFS+PE+....

#define BaudRate SPI_BAUDRATE_21MHZ;	//hier auch 11.5, 5.25, 2.625, 1.3125 ausw�hlbar

//hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)



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

	spiExchange();



	// -------- Init UWB --------

	//PAN Identifier
		//Identifeier Nachricht aus newConfig erstellen

	spiExchange();



	spiStop();	//f�r Mutexinteraktion genutzt	
}


void __attribute__((used)) EXTI4_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line4);
	DWM1000_IRQ_ISR();	//ISR in ai_task.c
}

//helper Functions:
void spiStart(){
	spiBeginTransaction(BaudRate);
}

void spiStop(){
	spiEndTransaction();
}

