#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../../../vendor/CMSIS/CMSIS/Driver/Include/Driver_SPI.h"
#include "stm32f4xx_spi.h"
#include "deck_spi.h"
#include "stm32f4xx_exti.h"	//wird benoetigt um externe interrupts zu initialisieren
#include "stm32f4xx_exti.h"	//wird ben�tigt um externe interrupts zu initialisieren
#include "../ai_datatypes.h"

#include "../ai_task.h"


//hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)

//helper Functions:
void spiStart(){
	spiBeginTransaction(BaudRate);
}

void spiStop(){
	spiEndTransaction();
}

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
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_IRQ; //GPIO_PIN_11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_Init(GPIO_PORT, &GPIO_InitStructure);


	// Init reset output
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_RESET; //GPIO_PIN_10
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
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;	//EXTI15_10_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_LOW_PRI;	//13
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	


}


bool dwm1000_SendData(void * data, int lengthOfData, enum e_message_type_t message_type, char targetID /*Adressen?, ...*/) {
	spiStart();

	//1. Aufbauen der Transmit Frame f�r den SPI Bus an den DWM1000
	void *sendData;
	int messageSize = lengthOfData + sizeof(char) + sizeof(enum e_message_type_t);
	sendData = malloc(messageSize);	// zwei bytes Extra um Art der Nachricht und Name des Senders beizufügen
	if (sendData == NULL) {
		return false;		//kein Mem mehr verfügbar --> Funktion wird nicht Ausgeführt
	}
	char senderID = my_ai_name; 

	*(char*)sendData = senderID;																	//Sender ID ist erstes Byte der Nachricht
	*(char*)((int*)sendData + sizeof(char)) = targetID;												//targedID ist ab zweites Byte der Nachricht
	*(enum e_message_type_t*)((int*)sendData + 2*sizeof(char)) = message_type;			//message ytpe ist ab drittes Byte der Nachricht
	for (int i = 0; i < lengthOfData; i++)															//jedes byte einzeln auf alloc Speicher schreiben
	{
		*((char*)sendData + 2*sizeof(char) + sizeof(enum e_message_type_t) + i) = *((char*)data + i);
	}

	//2. Data auf Transmit Data Buffer Register packen
	char instruction = WRITE_TXBUFFER;
	char receiveByte = 0x00;

	void *placeHolder = malloc(lengthOfData);
	
	for (int i = 0; i < lengthOfData; i++)
	{
		*((char*)placeHolder + i) = 0;
	}
	
	//spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx)		
	spiExchange(1, &instruction, placeHolder);	//instruction Schicken - write txbuffer

	spiExchange(lengthOfData, sendData, placeHolder);

	//3. System Control aktualisieren
	instruction = READ_SYS_CTRL;
	spiExchange(1, &instruction, &receiveByte);	//instruction: ich will sysctrl lesen

	void * sysctrl = malloc(5);
	spiExchange(5, placeHolder, sysctrl);		//syscontrol lesen


	*(int*)sysctrl |= SET_TRANSMIT_START;		//neuen Inhalt für syscontrol erstellen

	instruction = WRITE_SYS_CTRL;			
	spiExchange(1, instruction, placeHolder);	//instruction: ich will syscontrol schreiben

	spiExchange(5, sysctrl, placeHolder);		//syscontrol schreiben

	//4. Sendung ueberpruefen (Timestamp abholen?, ...)
		//.. noch keine Relevanten Dinge eingefallen

	//5. Resourcen freigeben
	free(sendData);
	free(placeHolder);
	free(sysctrl);

	spiStop();
}


e_message_type_t dwm1000_ReceiveData(void * data, int lengthOfData) {
	//1. Receive Buffer Auslesen
	//2. Receive Enable Setzten
	//3. Art der Nachricht entschl�sseln
	//4. Auf Data schreiben
	//5. Art der Nachricht zurueckgeben
}


float dwm1000_getDistance(double nameOfOtherDWM) {
	// Beschreibung für eine Drohne
	//1. Nachrichten an Partner schicken (Master)
	//2. Partner antwortet  mit seinem Timestamp (Slave)
	//3. Master empfägt Slave-Timestamp
	//4. Errechnung der Response Time
	//5. (Gestoppte Zeit - (TransmitTimestamp_Ziel - ReceiveTimestamp_Ziel))/2 * Lichtgeschw = Abstand
	// Danach weiß der Master, Initiator den Abstand 
}

void dwm1000_sendDistance(char id_requester) {
	//1. Funktion aktiviert, nach Distance request Eingang
	//2. Antworten, damit requester Zeit stopppen kann

	float Slave_timestamp_on_first_request;		
	float Slave_timestamp_on_final_request;
	getDistance();
	getimmediateAnswer();

	//3. Zeit zwischen Receive Timestamp und Transmit Timestamp an requester schicken, damit von gestoppter zeit abgezogen werden kann
	//Receive Timestamp - Transmit Timestamp
}

enum e_interrupt_type_t dwm1000_EvalInterrupt()
{
	//5 Bytes fuer System Event Status Register reservieren
	void *sesrContents;
	int registerSize = 5;			
	sesrContents = malloc(registerSize);	


	void *placeholder;
	placeholder = malloc(registerSize);

	//Placeholder mit 0 fuellen
	for (int i = 0; i < registerSize; i++)					
	{
		*((char*)placeholder + i) = 0;
	}
	char instruction = READ_SESR;

	spiStart();
	
	//Instruction Transmitten (an Slave)	
	spiExchange(1, &instruction, placeholder);		

	//Register auslesen
	spiExchange(registerSize, placeholder, sesrContents);

	//Gelesenes Register auswerten
	enum e_interrupt_type_t retVal;
	char TFSMask = MASK_TRANSMIT_FRAME_SENT;
	short RFSMask = MASK_RECEIVE_FRAME_SENT;

	//ist Transmit Frame Sent gesetzt?
	if ((*(char*)sesrContents & TFSMask) > 1)		//char, weil Mask 1 Byte lang
	{
		retVal = TX_DONE;
	}

	//ist Receive Frame Sent gesetzt?
	else if ((*(short*)sesrContents & RFSMask) > 1)	//short, weil Mask 2 Byte lang
	{
		retVal = RX_DONE;
	}

	//Resourcen freigeben
	free(placeholder);
	free(sesrContents);
	spiStop();

	return retVal;
}


st_DWM_Config_t dwm1000_init(st_DWM_Config_t newConfig) {

	spiStart();	//fuer Mutexinteraktion genutzt	
	
	// -------- Init SPI --------

	//Baud Rate
		//Bausrate Nachricht aus newConfig erstellen

	spiExchange(0,0,0);



	// -------- Init UWB --------

	//PAN Identifier
		//Identifeier Nachricht aus newConfig erstellen

	spiExchange(0,0,0);



	spiStop();	//fuer Mutexinteraktion genutzt	
}


void __attribute__((used)) EXTI11_Callback(void)
{
	EXTI_ClearITPendingBit(EXTI_Line4);
	DMW1000_IRQ_Flag = true;	//aktiviert synchrone "ISR" in ai_task.c
}






