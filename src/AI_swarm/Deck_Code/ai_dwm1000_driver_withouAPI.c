/*
ai_dwm1000_driver.c

Diese Datei definiert die Funktionen, welche benoetigt werden, um den DWM1000 zu steuern (Daten Senden, Ranging, ...).
Zusaetzlich werden die wichtigen Registerbkonfiguriert.

Sollten keine Funktionsaenderungen des DWM1000 gewuenscht sein, sollte dieser Driver am Besten NICHT geaendert werden!
*/

#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../../../vendor/CMSIS/CMSIS/Driver/Include/Driver_SPI.h"
#include "stm32f4xx_spi.h"
#include "deck_spi.h"
#include "stm32f4xx_exti.h"	//wird benoetigt um externe interrupts zu initialisieren
#include "../ai_datatypes.h"
#include "stm32f4xx.h"
#include "../ai_task.h"
#include <stdlib.h>

//Hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)

//Helper Functions:
void spiStart(){
	spiBeginTransaction(BaudRate);
}

void spiStop(){
	spiEndTransaction();
}

//Beschreibt die angegebene Anzahl an Bytes mit "0"
void fillMemZero(void * mem, char size){
	//Placeholder mit 0 fuellen
	char zero = 0;
	for (int i = 0; i < size; i++)
	{
		*((char*)mem + i) = zero;
	}
}

//Schreibt die angegeben Anzahl an Bytes von origin in Target
 void writeToMem(void *target, double *origin, char size){
	 for (int i = 0; i < size; i++)
	 {
	 	*((char*)target + i) = *(char*)(origin + 1);
	 }
 }

//Setzt Bits, die im Beispiel gesetzt sind (für init) - behält bereits gesetzte Bits gesetzt
void setInitBits(void *target, double *origin, char size){
	for (int i = 0; i < size; i++)
		 {
		 	*((char*)target + i) = *(char*)(origin + 1) | *((char*)target + i);
		 }
}


//Inhalt von locodec.c init (ab Z.312) inspiriert (und angepasst)
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

	EXTI_Init(&EXTI_InitStructure);

	// Enable interrupt
	NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQChannel;	//EXTI15_10_IRQn
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_LOW_PRI;	//13
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Reset the DW1000 chip
	GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 0);
	vTaskDelay(M2T(10));
	GPIO_WriteBit(GPIO_PORT, GPIO_PIN_RESET, 1);
	vTaskDelay(M2T(10));

	return 1;
}

bool dwm1000_SendData(st_message_t *message) {
	spiStart();

	/*
	//1. Aufbauen der Transmit Frame fuer den SPI Bus an den DWM1000
	void *sendData;
	int messageSize = lengthOfData + sizeof(char) + sizeof(e_message_type_t);
	sendData = malloc(messageSize);	// zwei bytes Extra um Art der Nachricht und Name des Senders beizufügen
	if (sendData == NULL) {
		return false;		//kein Mem mehr verfügbar --> Funktion wird nicht Ausgeführt
	}
	char senderID = my_ai_name; 

	*(char*)sendData = senderID;																//Sender ID ist erstes Byte der Nachricht
	*(char*)((int*)sendData + sizeof(char)) = targetID;											//targedID ist ab zweites Byte der Nachricht
	*(e_message_type_t*)((int*)sendData + 2*sizeof(char)) = message_type;						//message ytpe ist ab drittes Byte der Nachricht
	for (int i = 0; i < lengthOfData; i++)														//jedes byte einzeln auf alloc Speicher schreiben
	{
		*((char*)sendData + 2*sizeof(char) + sizeof(e_message_type_t) + i) = *((char*)data + i);
	}

	//2. Data auf Transmit Data Buffer Register packen
	unsigned char instruction = WRITE_TXBUFFER;
	unsigned char receiveByte = 0x00;

	void *placeHolder = malloc(lengthOfData);
	
	fillMemZero(placeHolder, lengthOfData);
	*/
	
	//spiExchange(size_t length, const uint8_t * data_tx, uint8_t * data_rx)		
	
	//2. Data auf Transmit Data Buffer Register packen
	unsigned char instruction = WRITE_TXBUFFER;
	unsigned char receiveByte = 0x00;

	unsigned int lengthOfMessage = sizeof(st_message_t);
	void *placeHolder = pvPortMalloc(lengthOfMessage);
	fillMemZero(placeHolder, lengthOfMessage);

	spiExchange(1, &instruction, placeHolder);	//instruction Schicken - write txbuffer

	spiExchange(lengthOfMessage, (void*)message, placeHolder);

	//3. System Control aktualisieren
	instruction = READ_SYS_CTRL;
	spiExchange(1, &instruction, &receiveByte);	//instruction: ich will sysctrl lesen

	void * sysctrl = pvPortMalloc(5);
			double debugHilfe = *(double*)sysctrl;
	fillMemZero(sysctrl, 5);
			debugHilfe = *(double*)sysctrl;

	spiExchange(5, placeHolder, sysctrl);		//syscontrol lesen
			debugHilfe = *(double*)sysctrl;


	double transmitBits = WRITE_TRANSMIT_BITs;
	setInitBits(sysctrl, &transmitBits, 5);		//neuen Inhalt für syscontrol erstellen

	instruction = WRITE_SYS_CTRL;			
	spiExchange(1, &instruction, placeHolder);	//instruction: ich will syscontrol schreiben

		if (debugHilfe > -1)
				debugHilfe = *(double*)sysctrl;

	spiExchange(5, sysctrl, placeHolder);		//syscontrol schreiben

	//4. Sendung ueberpruefen (Timestamp abholen?, ...)
		//.. noch keine Relevanten Dinge eingefallen

	//5. Resourcen freigeben
	vPortFree(placeHolder);
	vPortFree(sysctrl);

	spiStop();

	return 1;
}

e_message_type_t dwm1000_ReceiveData(st_message_t *data) {
	//1. Receive Buffer Auslesen
	unsigned int lengthOfData = sizeof(st_message_t);
	void *placeHolder = pvPortMalloc(lengthOfData);
	fillMemZero(placeHolder, lengthOfData);

	unsigned char instruction = READ_RXBUFFER;

	spiExchange(1, &instruction, placeHolder);		//Instruction: Eceive Buffer soll gelesen werden
	spiExchange(lengthOfData, (void*)data, placeHolder);	//READ_RXBUFFER schreiben

	//2. Receive Enable Setzten
	
	instruction = WRITE_SYS_CTRL;

	spiExchange(1, &instruction, placeHolder);		//Instruction: Eceive Buffer soll gelesen werden
	fillMemZero(placeHolder, lengthOfData);
	spiExchange(lengthOfData, (void*)data, placeHolder);	//READ_RXBUFFER schreiben

	//3. Art der Nachricht entschluesseln
	//4. Auf Data schreiben
	//5. Art der Nachricht zurueckgeben 
	e_message_type_t retType;
	retType = DISTANCE_REQUEST;


	vPortFree(placeHolder);

	return retType;

}


//---------------------- RANGING ----------------------

/*
Sendet Distanzrequest an spezifiziertes Target

// Beschreibung des gesammten Ranginvorgangs:
	//1. Nachrichten an Partner schicken (requester)
		Flag setzen: requestTransmitTimestampPending

	//2. Partner antwortet  direkt (target)
		Flag setzen: immediateAnswerTransmitTimestampPending

	//3. Master empfägt target-Nachricht - berechnet T_round (Zeit die die Nachricht hin und zurück gebraucht hat)
		Flag setzen: processingTimePending

	//4. target berechnet seine Bearbeitungszeit (immediateAnswerTranceiveTimestamp - requestReceiveTimestamp)

	//5. (Gestoppte Zeit - (Bearbeitungstimestamp))/2 * Lichtgeschw = Abstand

	// Danach weiß der requester den Abstand 
*/
void dwm1000_requestDistance(char targetID) {
	st_message_t requestMessage;
	requestMessage.senderID = AI_NAME;
	requestMessage.targetID = targetID;
	requestMessage.messageType = DISTANCE_REQUEST;

	dwm1000_SendData(&requestMessage);
}
/*
	//1. Funktion aktiviert, nach Distance request Eingang
	//2. Antworten, damit requester Zeit stopppen kann

	//Zusatz: Zeit zwischen Receive Timestamp und Transmit Timestamp an requester schicken, damit von gestoppter Zeit abgezogen werden kann
	//Receive Timestamp - Transmit Timestamp
*/
void dwm1000_immediateDistanceAnswer(char id_requester) {
	st_message_t immediateAnswer;
	immediateAnswer.senderID = AI_NAME;
	immediateAnswer.targetID = id_requester;
	immediateAnswer.messageType = IMMEDIATE_ANSWER;

	dwm1000_SendData(&immediateAnswer);
}

void dwm1000_sendProcessingTime(char id_requester) {
	
	spiStart();
	
	//1. Timestamp TX lesen, Register 0x17 Timestamp von bit 0-39
	void *txTimestamp;
	int txStampSize = 5;			//5 Bytes für Timestaqmp reservieren
	txTimestamp = pvPortMalloc(txStampSize);

	void *placeholder;
	placeholder = pvPortMalloc(txStampSize);

	unsigned char instruction = READ_TX_TIMESTAMP;

	//Instruction Transmitten	
	spiExchange(1, &instruction, placeholder);

	//Placeholder mit 0 fuellen
	fillMemZero(placeholder, txStampSize);

	//txTmstpRegister auslesen
	spiExchange(txStampSize, placeholder, txTimestamp);

	//2. Timestamp RX lesen, Register 0x15 Timestamp von bit 0-39
	void *rxTimestamp;
	int rxStampSize = 5;			//5 Bytes für Timestaqmp reservieren
	rxTimestamp = pvPortMalloc(rxStampSize);

	instruction = READ_RX_TIMESTAMP;

	//Instruction Transmitten	
	spiExchange(1, &instruction, placeholder);
	
	//Placeholder mit 0 fuellen
	fillMemZero(placeholder, rxStampSize);

	//rxTmstpRegister auslesen
	spiExchange(rxStampSize, placeholder, rxTimestamp);


//------------------------------------------------------ hier nur einen Teil des Timestamps lesen (besteht aus 2 Zahlen wir sollten die in 15,65 pikosek nehmen)

	//3. Differenz bestimmen, Rx-Tx, Ergebnis in 15,65 Pico sek (1ps = 10^⁻12s)
		//hier Josy und Janik
		//erst timestamp aufdröseln

	ai_time rxTime = /*nur die letzten paar bytes oder so vom Register*/ *15.65;
	ai_time txTime = /*nur die letzten paar bytes oder so vom Register*/ *15.65;
	ai_time processingTime = txTime - txTime;

	//4. an requester schicken


	st_message_t processingTimeMessage;
	processingTimeMessage.targetID = id_requester;
	processingTimeMessage.time = processingTime;
	processingTimeMessage.messageType = PROCESSING_TIME;

	dwm1000_SendData(&processingTimeMessage);


	vPortFree(txTimestamp);
	vPortFree(placeholder);
	vPortFree(rxTimestamp);
	spiStop();
}

//auslesen des rxTimestamps
ai_time getRxTimestamp(){

}

//auslesen des txTimestamps
ai_time getTxTimestamp(){

}

e_interrupt_type_t dwm1000_EvalInterrupt()
{
	//5 Bytes fuer System Event Status Register reservieren
	void *sesrContents;
	int registerSize = 5;			
	sesrContents = pvPortMalloc(registerSize);

	void *placeholder;
	placeholder = pvPortMalloc(registerSize);

	unsigned char instruction = READ_SESR;

	spiStart();
	
	//Instruction Transmitten (an Slave)	--> lesen von System Event Status Register
	spiExchange(1, &instruction, placeholder);		

	//Placeholder mit 0 fuellen
	fillMemZero(placeholder, registerSize);

	//Register auslesen
	spiExchange(registerSize, placeholder, sesrContents);

	//Gelesenes Register auswerten
	e_interrupt_type_t retVal = FAILED_EVAL;
	unsigned char TFSMask = MASK_TRANSMIT_FRAME_SENT;
	short RFSMask = MASK_RECEIVE_DATA_FRAME_READY;

	//ist Transmit Frame Sent gesetzt?
	if ((*(char*)sesrContents & TFSMask) > 1)		//char, weil Mask 1 Byte lang
	{
		retVal = TX_DONE;
	}

	//ist Receive Data Frame Ready
	else if ((*(short*)sesrContents & RFSMask) > 1)	//short, weil Mask 2 Byte lang
	{
		retVal = RX_DONE;
	}

	//Resourcen freigeben
	vPortFree(placeholder);
	vPortFree(sesrContents);
	spiStop();

	return retVal;
}

void dwm1000_init() {

	spiStart();	//fuer Mutexinteraktion genutzt	
	

	//------------------------ TFC LESEN
	//Init Transmission
	unsigned char emptyByte = 0;
	unsigned char instruction = READ_TFC;

	//Read Transmission Frame Control Register
	spiExchange(1, &instruction, &emptyByte);
	
	//WRITE_INIT_TX_FCTRL
	void *tfcContents = pvPortMalloc(5);
	double initValTFC = WRITE_INIT_TX_FCTRL;
	void *placeHolder = pvPortMalloc(5);


	spiExchange(5, placeHolder, tfcContents);		//Inhalt des TFC Registers auslesen
	setInitBits(tfcContents, &initValTFC, 5);		//Inhalt mit gewolltem ODERn

	//init werte für tfc in dmw1000 packen
	instruction = WRITE_TFC;
	emptyByte = 0;
	spiExchange(1,  &instruction, &emptyByte);	//write tfc Instruction

	fillMemZero(placeHolder, 5);
	spiExchange(5, tfcContents, placeHolder);

	vPortFree(tfcContents);
	vPortFree(placeHolder);

	//------------------------ PAN Adresse setzten (reg id = 0x03)

	//nötig? -ne, da immer bei Nachricht sender und target dabei sein sollten

	//------------------------ System Event Mask Register (reg id = 0x0E)
	//Read syseventmask
	instruction = READ_SYS_EVENT_MASK;
	spiExchange(1, &instruction, &emptyByte);

	//syseventmask
	void *sysEvMaskContents = pvPortMalloc(4);
	double initValSysEvMask= WRITE_INIT_SYS_EVENT_MASK;
	placeHolder = pvPortMalloc(4);

	//Placeholder mit 0 fuellen
	fillMemZero(sysEvMaskContents, 4);
	fillMemZero(placeHolder, 4);

	spiExchange(4, placeHolder, sysEvMaskContents);		//Inhalt des syseventmask Registers auslesen
	setInitBits(sysEvMaskContents, &initValSysEvMask, 4);		//Inhalt mit gewolltem ODERn

	//init werte für syseventmask in dmw1000 packen
	instruction = WRITE_SYS_EVENT_MASK;
	emptyByte = 0;
	spiExchange(1,  &instruction, &emptyByte);	//write syseventmask Instruction

	fillMemZero(placeHolder, 5);
	spiExchange(5, sysEvMaskContents, placeHolder);

	vPortFree(sysEvMaskContents);

	spiStop();	//fuer Mutexinteraktion genutzt
	vPortFree(placeHolder);
}

void __attribute__((used)) EXTI11_Callback(void)
{
	EXTI_ClearITPendingBit(EXTI_Line4);
	DMW1000_IRQ_Flag = 1;	//aktiviert synchrone "ISR" in ai_task.c
}