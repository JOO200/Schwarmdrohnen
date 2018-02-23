#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../../../vendor/CMSIS/CMSIS/Driver/Include/Driver_SPI.h"
#include "stm32f4xx_spi.h"
#include "deck_spi.h"
<<<<<<< HEAD
#include "stm32f4xx_exti.h"	//wird benoetigt um externe interrupts zu initialisieren
=======
#include "stm32f4xx_exti.h"	//wird ben�tigt um externe interrupts zu initialisieren
#include "../ai_datatypes.h"
>>>>>>> 531d29733538e874e19555b9bccba685fbe8a8e8


//--------------------------------Instructions:
//Bit 7 -> 0
//Bit 7: Read - 0, Write - 1
//Bit 6: 0
// Bit 5 - 0: Reg ID
#define READ_TFC 0b00001000
#define WRITE_TFC 0b10001000

#define WRITE_TXBUFFER 0b10001001

#define READ_SYS_CTRL 0b00001101
#define WRITE_SYS_CTRL 0b10001101
#define SET_TRANSMIT_START WRITE_TRANSMIT_BITs

#define READ_SYS_STATUS 0b00001111
#define WRITE_SYS_STATUS 0b10001111

#define READ_RXBUFFER 0b00010001	//Reg ID:0x11
#define READ_RXTIMESTAMP 0b00010101 //REG ID 0x15
//--------------------------------Register:
/*Register "Transmit Frame Control", besteht aus 5 Byte
Beschreibung:
Orientierung: 
Bit 39, Bit 38, ..., Bit 0
Letztendlich muss man nur noch die einzelnen Zahlen addieren und erh�lt das 5-Byte gro�e Register
Bin�r -> Hex
00000000 00000000 00000000 00000000 00001100 -> 0x000000000C - TFLEN, Transmission Frame Length (0-6)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TFLE, Transmit Frame Length Extension (7-9)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - R, Reserved (10-12)
00000000 00000000 00000000 00100000 00000000 -> 0x0000002000 - TXBR, Transmit Bit Rate (13+14)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TR, Transmit Ranging enable  (15)
00000000 00000000 00000001 00000000 00000000 -> 0x0000010000 - TXPRF, Transmit Pulse Repetition Frequency (16+17)
00000000 00000000 00000100 00000000 00000000 -> 0x0000040000 - TXPSR, Transmit Pramble Symbol Repititions  (18+19)
00000000 00000000 00010000 00000000 00000000 -> 0x0000100000 - PE, Preamble Extension (20+21)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - TXBOFFS, Transmit Buffer index offset (22-31)
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 - IFSDELAY, Extension f�r TXBOFFS  (32-39)
Addiert:
00000000 00000000 00010101 00100000 00001100 -> 0x000015200C - TFC 
*/
#define WRITE_INIT_TX_FCTRL 0x000015200C;	//Instruction Manual S.12; Register-ID: 0x08, Bei INIT Schreiben

/*Register "System Control Register", besteht aus 4 Byte
Beschreibung:
Orientierung:
Bit 31, Bit 30, ..., Bit 0
Letztendlich muss man nur noch die einzelnen Zahlen addieren und erh�lt das 4-Byte gro�e Register
Bin�r -> Hex
00000000 00000000 00000000 00001011  -> 0x0000000B 
*/
#define READ_SYS_CTRL 0x00000000; //System Control Register, Register-ID: 0x0D, Diesen Wert einem Speicherbereich zuschreiben, dann den Inhalt des Registers darauf lesen

#define WRITE_TRANSMIT_BITs 0x00000003;	//hiermit | -> dann Transmit Bit auf jeden Fall gesetzt

/*Register "System Status Register", besteht aus 5 Byte
Beschreibung:
Orientierung:
Bit 39, Bit 38, ..., Bit 0
Letztendlich muss man nur noch die einzelnen Zahlen addieren und erh�lt das 5-Byte gro�e Register
Bin�r -> Hex
00000000 00000000 00000000 00000000 00000000 -> 0x0000000000 

Es koenten folgende interupts gesetzt werden, die für uns relevant sind:
RXDFR (Bit 13) -> wenn die Nachricht fertig gesendet ist
RXFCG (Bit 14) -> Checksummenvergleich erfolgreich am Ende des Frames - FRAME GUT GESENDET
RXFCE (Bit 15) -> Checksummenvergleich nicht erfolgreich am Ende des Frames	- FRAME SCHLECHT GESENDET
*/
#define READ_SYS_STATUS 0x0000000000 //System Event Status Register, Register-ID: 0x0F, Diesen Wert einem Speicherbereich zuschreiben, dann den Inhalt des Registers darauf lesen

/*Register "System Status Register", besteht aus 5 Byte
Beschreibung:
Orientierung:
Bit 39, Bit 38, ..., Bit 0
Letztendlich muss man nur noch die einzelnen Zahlen addieren und erh�lt das 5-Byte gro�e Register
Bin�r -> Hex
00000000 00000000 00000000 00100000 00000000 -> 0x0000000000

Es koenten folgende interupts gesetzt werden, die für uns relevant sind:
RXDFR (Bit 13) -> wenn die Nachricht fertig gesendet ist
RXFCG (Bit 14) -> Checksummenvergleich erfolgreich am Ende des Frames - FRAME GUT GESENDET
RXFCE (Bit 15) -> Checksummenvergleich nicht erfolgreich am Ende des Frames	- FRAME SCHLECHT GESENDET
*/
#define MESSAGE_RECEIVED_STATUS 0x0000002000;	//Wenn bei verunden mit diesem Wert und dem System Status Register (0x0F) > 0 rauskommt -> Nachricht erhalten

/*Register "System Control Register", besteht aus 4 Byte
Beschreibung:
Orientierung:
Bit 31, Bit 30, ..., Bit 0
Letztendlich muss man nur noch die einzelnen Zahlen addieren und erh�lt das 4-Byte gro�e Register
Bin�r -> Hex
00000000 00000000 00000001 00000000  -> 0x00000100*/
#define WRITE_RECEIVE_ENABLE 0x00000100 //Nach lesen einer Nachricht aus den Receive Buffer muss dieses Bit gesetzt werden um neue Nachrichten empfangen zu können



#include "../ai_task.h"

#define BaudRate 0x0008//SPI_BAUDRATE_21MHZ	//hier auch 11.5, 5.25, 2.625, 1.3125 ausw�hlbar -- 21MHZ --> (uint16_t)0x0008

//defines fuer Interrupt Config
#define EXTI_Line11 0x00800
#define EXTI_LineN 	EXTI_Line11	//bestimmt exti input port (von Bitcraze RX genannt)
#define EXTI_Mode_Interrupt 0x00	//interrupt mode - aus stm32f4xx_exti.h
#define EXTI_Trigger_Rising 0x08	//aus stm32f4xx_exti.h
#define ENABLE 1;

//defines fuer SPI
#define CS_PIN DECK_GPIO_IO1		//CS/SS Pin ist "IO_1"
#define GPIO_Mode_OUT 0x01
#define GPIO_OType_OD 0x01
#define GPIO_PIN_IRQ GPIO_PIN_11


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


bool dwm1000_SendData(void * data, int lengthOfData, enum e_message_type_t message_type /*Adressen?, ...*/) {
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
	*(enum e_message_type_t*)((int*)sendData + sizeof(char)) = message_type;						//message ytpe ist zweites Byte der Nachricht
	for (int i = 0; i < lengthOfData; i++)															//jedes byte einzeln auf alloc Speicher schreiben
	{
		*((char*)sendData + sizeof(char) + sizeof(enum e_message_type_t) + i) = *((char*)data + i);
	}

	//2. Data auf Transmit Data Buffer Register packen
	char instruction = WRITE_TXBUFFER;
	char receiveByte = 0x00;

	void *placeHolder = malloc(lengthOfData);

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
	//1. Funktion aktiviert, nach Distance request eingang
	//2. Antworten, damit requester Zeit stopppen kann
	//3. Zeit zwischen Receive Timestamp und Transmit Timestamp an requester schicken, damit von gestoppter zeit abgezogen werden kann
	//Receive Timestamp - Transmit Timestamp

	timestamp_t Slave_timestamp

		getDistance()
		getimmediateAnswer


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






