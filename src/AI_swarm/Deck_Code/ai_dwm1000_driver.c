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


static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;

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

//Functions und dwOps init from locodeck.c:
static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_2MHZ;
/************ Low level ops for libdw **********/
static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memset(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  memcpy(data, spiRxBuffer+headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  memcpy(spiTxBuffer, header, headerLength);
  memcpy(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}
static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiSpeed = SPI_BAUDRATE_2MHZ;
  }
  else if (speed == dwSpiSpeedHigh)
  {
    spiSpeed = SPI_BAUDRATE_21MHZ;
  }
}
static void delayms(dwDevice_t* dev, unsigned int delay)
{
  vTaskDelay(M2T(delay));
}

static dwOps_t dwOps = {
  .spiRead = spiRead,
  .spiWrite = spiWrite,
  .spiSetSpeed = spiSetSpeed,
  .delayms = delayms,
};

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

	// Initialize the driver
	dwInit(dwm, &dwOps);       // Init libdw

	int result = dwConfigure(dwm);
	if (result != 0) {
		DEBUG_PRINT("Failed to configure DW1000!\r\n");
		return;
	}

	dwEnableAllLeds(dwm);

	dwTime_t delay = {.full = 0};
	dwSetAntenaDelay(dwm, delay);

	dwAttachSentHandler(dwm, txCallback);
	dwAttachReceivedHandler(dwm, rxCallback);
	dwAttachReceiveTimeoutHandler(dwm, rxTimeoutCallback);

	dwNewConfiguration(dwm);
	dwSetDefaults(dwm);
	dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
	dwSetChannel(dwm, CHANNEL_2);
	dwUseSmartPower(dwm, true);
	dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

	dwSetReceiveWaitTimeout(dwm, RX_TIMEOUT);

	dwCommitConfiguration(dwm);


	return 1;
}


bool dwm1000_SendData(st_message_t *message) {
	dwNewTransmit(dwm);
	dwSetDefaults(dwm);
	dwSetData(dwm, (uint8_t*)message, MAC802154_HEADER_LENGTH+sizeof(st_message_t));

	dwStartTransmit(dwm);

	return 1;
}

e_message_type_t dwm1000_ReceiveData(st_message_t *data) {
	int dataLength = dwGetDataLength(dwm);

	if (dataLength == 0){
		DEBUG_PRINT("[ai_swarm]Wrong Receive Message size! (size = 0)\r\n");
		return 0;		//if Fall fuer leere Empfangsdaten
	}
	if (dataLength != sizeof(st_message_t)){
		DEBUG_PRINT("[ai_swarm]Wrong Receive Message size! (size = 0)\r\n");
	}

	st_message_t rxPacket;
	memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);  //packet mit Nullen ueberschreiben
	
	dwGetData(dwm, (uint8_t*)&rxPacket, dataLength);	//get Packet und befuellen

	return rxPacket;

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


	//----------hier nur einen Teil des Timestamps lesen (besteht aus 2 Zahlen wir sollten die in 15,65 pikosek nehmen)

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

/*//auslesen des rxTimestamps
ai_time getRxTimestamp(){

}

//auslesen des txTimestamps
ai_time getTxTimestamp(){

}*/

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
