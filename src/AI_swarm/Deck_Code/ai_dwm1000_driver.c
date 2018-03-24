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
#include "mac.h"

//Hier wird deck_spi.h/deck_spi.c angewendet (in src/deck/api/...)

static dwDevice_t dwm_device;
static dwDevice_t *dwm = &dwm_device;


//meset/memcpy führt zu Problemen also hier meine eig funktionen:
//setzt eine Speicherregion byte für byte auf den gewünschten Wert (z.B. 0)
void nicoSetsMem(void * target, const uint8_t value, unsigned int size){
	for(unsigned int i = 0;i < size;i++){
		*(uint8_t*)(target + i) = value;
	}
}

//kopiert eine speicherregion in eine andere
void nicoCpysMem(void * target, const void * source, unsigned int size){
	for(unsigned int i = 0;i < size;i++){
		*(uint8_t*)(target + i) = *(uint8_t*)(source + i);
	}
}


//Functions und dwOps init from locodeck.c:
static uint8_t spiTxBuffer[196];
static uint8_t spiRxBuffer[196];
static uint16_t spiSpeed = SPI_BAUDRATE_3MHZ;
/************ Low level ops for libdw **********/
static void spiRead(dwDevice_t* dev, const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  nicoCpysMem(spiTxBuffer, header, headerLength);
  nicoSetsMem(spiTxBuffer+headerLength, 0, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  nicoCpysMem(data, spiRxBuffer+headerLength, dataLength);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}
static void spiWrite(dwDevice_t* dev, const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  spiBeginTransaction(spiSpeed);
  digitalWrite(CS_PIN, LOW);
  nicoCpysMem(spiTxBuffer, header, headerLength);
  nicoCpysMem(spiTxBuffer+headerLength, data, dataLength);
  spiExchange(headerLength+dataLength, spiTxBuffer, spiRxBuffer);
  digitalWrite(CS_PIN, HIGH);
  spiEndTransaction();
}
static void spiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
  if (speed == dwSpiSpeedLow)
  {
    spiSpeed = SPI_BAUDRATE_3MHZ;
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


//Functions for interrupt handling
e_interrupt_type_t lastInterrupt;
static void dwm1000_transmitDoneHandler(){
	lastInterrupt = TX_DONE;
}

static void dwm1000_receiveHandler(){
	lastInterrupt = RX_DONE;
}

static void dwm1000_receiveFailedHandler(){
	lastInterrupt = RX_FAILED;
}

static void dwm1000_receiveTimeoutHandler(){
	lastInterrupt = RX_INT_TIMEOUT;
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

	// Init CS pin
	pinMode(CS_PIN, OUTPUT);

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
		return 0;
	}

	dwEnableAllLeds(dwm);

	dwTime_t delay = {.full = 0};
	dwSetAntenaDelay(dwm, delay);

	dwAttachSentHandler(dwm, dwm1000_transmitDoneHandler);
	dwAttachReceivedHandler(dwm, dwm1000_receiveHandler);
	dwAttachReceiveFailedHandler(dwm, dwm1000_receiveFailedHandler);
	dwAttachReceiveTimeoutHandler(dwm, dwm1000_receiveTimeoutHandler);

	dwNewConfiguration(dwm);
	dwSetDefaults(dwm);
	dwEnableMode(dwm, MODE_SHORTDATA_FAST_ACCURACY);
	dwSetChannel(dwm, CHANNEL_2);
	dwUseSmartPower(dwm, true);
	dwSetPreambleCode(dwm, PREAMBLE_CODE_64MHZ_9);

	dwReceivePermanently(dwm, true);
	dwStartReceive(dwm);

	dwCommitConfiguration(dwm);


	return 1;
}


bool dwm1000_SendData(st_message_t *message) {
	spiSetSpeed(dwm, dwSpiSpeedLow);

	dwNewTransmit(dwm);
	dwSetData(dwm, (uint8_t*)message, sizeof(*message));

	dwStartTransmit(dwm);

	vTaskDelay(M2T(3));

	dwStartReceive(dwm);

	return 1;
}

e_message_type_t dwm1000_ReceiveData(st_message_t *data) {
	unsigned int dataLength = dwGetDataLength(dwm);

	/*if (dataLength == 0){
		DEBUG_PRINT("[ai_swarm]Wrong Receive Message size! (size = 0)\r\n");
		return 0;		//if Fall fuer leere Empfangsdaten
	}
	if (dataLength != sizeof(*data)){
		DEBUG_PRINT("[ai_swarm]Wrong Receive Message size! (size = 0)\r\n");
	}*/
	dwGetData(dwm, (uint8_t*)&data, dataLength);	//get Packet und befuellen

	dwStartReceive(dwm);


	return data->messageType;

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
	
	
	//1. Timestamp TX lesen, Register 0x17 Timestamp von bit 0-39

	dwTime_t txTimeStamp;
	dwGetTransmitTimestamp(dwm, &txTimeStamp);

	//2. Timestamp RX lesen, Register 0x15 Timestamp von bit 0-39
	dwTime_t rxTimeStamp;
	dwGetReceiveTimestamp(dwm, &rxTimeStamp);


	dwTime_t processingTime;
	processingTime.full = rxTimeStamp.full - txTimeStamp.full;

	//4. an requester schicken


	st_message_t processingTimeMessage;
	processingTimeMessage.targetID = id_requester;
	processingTimeMessage.time = processingTime;
	processingTimeMessage.messageType = PROCESSING_TIME;

	dwm1000_SendData(&processingTimeMessage);
}

dwTime_t dwm1000_getTxTimestamp(){
	dwTime_t ret;
	dwGetTransmitTimestamp(dwm, &ret);

	return ret;
}

dwTime_t dwm1000_getRxTimestamp(){
	dwTime_t ret;
	dwGetTransmitTimestamp(dwm, &ret);

	return ret;
}

/*//auslesen des rxTimestamps
ai_time getRxTimestamp(){

}

//auslesen des txTimestamps
ai_time getTxTimestamp(){

}*/

e_interrupt_type_t dwm1000_EvalInterrupt()
{
	//zurücksetzen
	lastInterrupt = FAILED_EVAL;

	//funktionen callen, welche je nach statusregisterzustand entsprechende handler callt
	dwHandleInterrupt(dwm);

	//von handlern gesetzte werte zurückgeben
	return lastInterrupt;
}

/*void dwm1000_init() {

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
}*/

void __attribute__((used)) EXTI11_Callback(void)
{
	DWM1000_IRQ_FLAG = TRUE;	//wird in while(1) in task main abgearbeitet
	NVIC_ClearPendingIRQ(EXTI_IRQChannel);
	EXTI_ClearITPendingBit(EXTI_LineN);
}
