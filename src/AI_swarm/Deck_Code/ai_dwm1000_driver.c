#include "ai_dwm1000_driver.h"
#include "Driver_SPI.h"
#include "stm32f4xx_spi.h"
#include "deck_spi.h"
#include "stm32f4xx_exti.h"	//wird benötigt um externe interrupts zu initialisieren

/*Register "Transmit Frame Control", besteht aus 5 Byte
Beschreibung:
Orientierung: 
Bit 39, Bit 38, ..., Bit 0
Letztendlich muss man nur noch die einzelnen Zahlen addieren und erhält das 5-Byte große Register
Binär -> Hex
00000000 00000000 00000000 00000000 00001100 -> 0x000000000C - TFLEN, Transmission Frame Length (0-6)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TFLE, Transmit Frame Length Extension (7-9)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - R, Reserved (10-12)
00000000 00000000 00000000 00100000 00000000 -> 0x0000002000 - TXBR, Transmit Bit Rate (13+14)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TR, Transmit Ranging enable  (15)
00000000 00000000 00000001 00000000 00000000 -> 0x0000010000 - TXPRF, Transmit Pulse Repetition Frequency (16+17)
00000000 00000000 00000100 00000000 00000000 -> 0x0000040000 - TXPSR, Transmit Pramble Symbol Repititions  (18+19)
00000000 00000000 00010000 00000000 00000000 -> 0x0000100000 - PE, Preamble Extension (20+21)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TXBOFFS, Transmit Buffer index offset (22-31)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - IFSDELAY, Extension für TXBOFFS  (32-39)
 
00000000 00000000 00010101 00100000 00001100 -> 0x000015200C - TFC 
*/
#define READ_TFC //Instruction Manual S.12

<<<<<<< HEAD
#define DWMINIT_TFC_TFLEN 0b0001100		//Orientierung: Bit  6 - 0 
#define DWMINIT_TFC_TFLE 0b000		//Orientierung: Bit  9, Bit 8, Bit 7 
#define DWMINIT_TFC_R 0b000		
#define DWMINIT_TFC_TXBR 0b01		// hier ändern für 850 kbps
#define DWMINIT_TFC_TR 0b0	
#define DWMINIT_TFC_TXPRF 0b01
#define DWMINIT_TFC_TXPSR 0b01
#define DWMINIT_TFC_PE 0b01
#define DWMINIT_TFC_TXBOFFS 0b0000000000
#define DWMINIT_TFC_IFSDELAY 0b00000000			//Zusammensetzung: IFSDELAY+TXBOFFS+PE+....
=======
#define DWMINIT_TFC 0x000015200C;	

>>>>>>> 1ce734a4258000675f5d4fca33894dcb50152146

#define BaudRate SPI_BAUDRATE_21MHZ;	//hier auch 11.5, 5.25, 2.625, 1.3125 auswählbar

//defines für Interrupt Config
#define EXTI_Line11 ((uint32_t)0x00800);
#define EXTI_LineN 	EXTI_Line11;	//bestimmt exti input port (von Bitcraze RX genannt)
#define EXTI_Mode_Interrupt 0x00;	//interrupt mode - aus stm32f4xx_exti.h
#define EXTI_Trigger_Rising 0x08;	//aus stm32f4xx_exti.h
#define ENABLE 

//defines für SPI
#define CS_PIN DECK_GPIO_IO1		//CS/SS Pin ist "IO_1"
#define GPIO_Mode_OUT 0x01;			
#define GPIO_OType_OD 0x01;			


//hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)


//inhalt von locodec.c init (ab Z.312) inspiriert (und angepasst)
bool setup_dwm1000_communication(){
	
	// ---- Aus ST-Doku: ----
	/*
	In order to use an I / O pin as an external interrupt source, follow steps
		below :
	(#) Configure the I / O in input mode using GPIO_Init()
		(#) Select the input source pin for the EXTI line using SYSCFG_EXTILineConfig()
		(#) Select the mode(interrupt, event) and configure the trigger
		selection(Rising, falling or both) using EXTI_Init()
		(#) Configure NVIC IRQ channel mapped to the EXTI line using NVIC_Init()

		[..]
	(@) SYSCFG APB clock must be enabled to get write access to SYSCFG_EXTICRx
		registers using RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	*/


	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	spiBegin();

	// Init IRQ input
	bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_IRQ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_Init(GPIO_PORT, &GPIO_InitStructure);


	// Init reset output
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RESET;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIO_PORT, &GPIO_InitStructure);


	//Interrupt Initstruct vorbereiten
	EXTI_InitStructure.EXTI_Line = EXTI_LineN;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	EXTI_Init(EXTI_InitStructure);


	// Enable interrupt
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_LOW_PRI;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	


}

bool dwm1000_SendData(void * data, int lengthOfData /*Adressen?, ...*/) {
	//1. Aufbauen der Transmit Frame für den SPI Bus an den DWM1000
	//2. Data auf Transmit Data Buffer Register
	//3. Transmit Frame Control aktualisieren
	//4. Sendung überprüfen (Timestamp abholen?, ...)
}


e_message_type_t dwm1000_ReceiveData(void * data, int lengthOfData) {
	//1. Receive Buffer Auslesen
	//2. Art der Nachricht entschlüsseln
	//3. Auf Data schreiben
	//4. Art der Nachricht zurückgeben
}


float dwm1000_getDistance(double nameOfOtherDWM) {
	//1. 
	//2.
	//3.
	//4.
}


st_DWM_Config_t dwm1000_init(st_DWM_Config_t newConfig) {

	spiStart();	//für Mutexinteraktion genutzt	
	
	// -------- Init SPI --------

	//Baud Rate
		//Bausrate Nachricht aus newConfig erstellen

	spiExchange();



	// -------- Init UWB --------

	//PAN Identifier
		//Identifeier Nachricht aus newConfig erstellen

	spiExchange();



	spiStop();	//für Mutexinteraktion genutzt	
}


void __attribute__((used)) EXTI11_Callback(void)
{
	EXTI_ClearITPendingBit(EXTI_Line4);
	DMW1000_IRQ_Flag = true;	//aktiviert synchrone "ISR" in ai_task.c
}

//-------------------------------------- helper Functions: -------------------------------------------------------------
void spiStart(){
	spiBeginTransaction(BaudRate);
}

void spiStop(){
	spiEndTransaction();
}




