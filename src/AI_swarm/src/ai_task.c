#include "ai_task.h"
#include "ai_datatypes.h"
#include "ai_distance.h"
#include "task.h"
#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../ai_config.h"
#include "ledring12.h"

#include "worker.h"

/* FreeRtos includes */
#include "FreeRTOS.h"

unsigned long ai_taskTicks;

bool ai_init = 0;
st_message_t testMessage;
unsigned char DWM1000_IRQ_Counter = 0;		//damit Zugriff Atomar ist wird hier char eingesetzt
/*static unsigned char irqCounterLog = 0;
static e_message_type_t logMSGType;*/


/*LOG_GROUP_START(debugReceive)
LOG_ADD(LOG_UINT8, intCount, &irqCounterLog)
LOG_ADD(LOG_INT16, logMSGType, &logMSGType)
LOG_GROUP_STOP(debugReceive)*/

//Ranging Flags, state and Distance:
st_rangingState_t rangingState[NR_OF_DRONES];

e_message_type_t lastMessageType;
unsigned char lastMessageTarget;

uint8_t FiFo_availible(st_buffer* fifo) {
	if(fifo->read != fifo->write) return 1;
	return 0;
}

uint8_t FiFo_read(st_buffer* fifo, st_message_t * message) {
	if(fifo->read == fifo->write) return 0;
	fifo->read++;
	if(fifo->read >= BUFFER_SIZE) fifo->read = 0;
	message = &(fifo->messages[fifo->read]);
	return 1;
}

uint8_t FiFo_write(st_buffer* fifo, st_message_t * message) {
	fifo->write++;
	if(fifo->write == BUFFER_SIZE) fifo->write = 0;
	memcpy(message, &(fifo->messages[fifo->write]), sizeof(st_message_t));
	return 1;
}
/*
void nop(){}

//alle requesteeflags resetten um neuen Rangingvorgang einzuleiten
void resetRangingStateRequestee(unsigned char targetID){
	rangingState[targetID].requestTransmitTimestampPending = false;//_/			//awaiting sent interrupt
	rangingState[targetID].immediateAnswerPending = false;//_/					//if immediate answer hasnt been received jet
	rangingState[targetID].processingTimePending = false;//				//awaiting processing time


	//requestee times
	rangingState[targetID].requestTxTimestamp.full = 0;
	rangingState[targetID].immediateAnswerRxTimestamp.full = 0;
	rangingState[targetID].tRound.full = 0;
}

//alle Targetflags resetten um neuen Rangingvorgang einzuleiten
void resetRangingStateTarget(unsigned char requesteeID){
	rangingState[requesteeID].immediateAnswerPending = FALSE;
	rangingState[requesteeID].transmitProcessingTimePendingFlag = FALSE;
	rangingState[requesteeID].requestRxTimestamp.full = 0;
	rangingState[requesteeID].requestTxTimestamp.full = 0;
}

void receiveHandler() {
	
	st_message_t message;
	FiFo_read(&fifo, &message);
	if (message.targetID != AI_NAME)
		return;

	switch (message.messageType)
	{
	case DISTANCE_TABLE:
		//1. aktuelle Distance Tabelle holen
		//2. an Absender zurück schicken
		break;
	case MASTER_STATE:
		//Status des Masters aktualisieren
		break;
	case DISTANCE_REQUEST:
		resetRangingStateTarget(message.senderID);

		rangingState[message.senderID].distanceRequested = TRUE;
		//1. Immediate Answer raussenden
		st_message_t immediateAnswer;
		immediateAnswer.senderID = AI_NAME;
		immediateAnswer.targetID = immediateAnswer.senderID;
		immediateAnswer.messageType = IMMEDIATE_ANSWER;
		dwm1000_SendData(&immediateAnswer);
		//2. danach Processing Time nachsenden (pending flag setzen)
		rangingState[message.senderID].transmitProcessingTimePendingFlag = TRUE;
		rangingState[message.senderID].distanceRequested = false;
		break;
	case IMMEDIATE_ANSWER:
		//Tround berechnen
		if (!rangingState[message.senderID].immediateAnswerPending)
			break;
		rangingState[message.senderID].immediateAnswerRxTimestamp = dwm1000_getRxTimestamp();
		rangingState[message.senderID].tRound.full = rangingState[message.senderID].immediateAnswerRxTimestamp.full - rangingState[message.senderID].requestTxTimestamp.full;
		rangingState[message.senderID].immediateAnswerPending = FALSE;

		rangingState[message.senderID].processingTimePending = TRUE;
		break;
	case PROCESSING_TIME:
		rangingState[message.senderID].processingTimePending = FALSE;
		//Distanz berechnen und eintragen
		//Rangin-Struct leeren
		double tdistDWM, troundDWM, tprocessingDWM, tdist;
		troundDWM = rangingState[message.senderID].immediateAnswerRxTimestamp.low32 - rangingState[message.senderID].requestTxTimestamp.low32;
		tprocessingDWM = message.time.low32;		//da aktuelle message processing time schickt ist diese hier zu entnehmen

		tdistDWM = 0.5*(troundDWM - tprocessingDWM);

		tdist = tdistDWM / LOCODECK_TS_FREQ;		//kp was das hier ist - kopiert von lpsTwrTag.c


		rangingState[message.senderID].distance = SPEED_OF_LIGHT * tdist;

		//Zeitberechnung in ai_lpsTwrTag Zeile 172-202
		break;
	default:
		break;
	}
}*/

void transmitDoneHandler(){
	if (rangingState[lastMessageTarget].transmitProcessingTimePendingFlag && lastMessageType == IMMEDIATE_ANSWER) {
		dwm1000_sendProcessingTime(lastMessageTarget);
		rangingState[lastMessageTarget].transmitProcessingTimePendingFlag = FALSE;
	}
	else if (rangingState[lastMessageTarget].requestTransmitTimestampPending && lastMessageType == DISTANCE_REQUEST){
		rangingState[lastMessageTarget].requestTxTimestamp = dwm1000_getTxTimestamp();
		rangingState[lastMessageTarget].requestTransmitTimestampPending = FALSE;
	}
	else
		return;
}


/*
//eventuell muessen args als void *
void startRanging(unsigned char targetID) {
	//requestMessage senden
	rangingState[targetID].lastRangingAi_Ticks = ai_taskTicks;//_/			//awaiting sent interrupt

	resetRangingStateRequest(targetID);

	dwm1000_requestDistance(targetID);

	//flags und vars für Momentanzustand setzen
	rangingState[targetID].requestTransmitTimestampPending = TRUE;
	rangingState[targetID].immediateAnswerPending = TRUE;

	lastMessageType = DISTANCE_REQUEST;
	lastMessageTarget = targetID;
}*/


bool initAi_Swarm() {
	//UWB_Deck fuer Josy und Janik ((((neuerdings))) auch Nico)
	//hier euer/unser init-shizzlel
	//setup_dwm1000_communication();		//HW-Setup


	//...
	return TRUE;
}

//hier unsere main
//wird ausgefuehrt von FreeRTOS-Scheduler sobald dieser das fuer sinnvoll haelt (und natuerlich, nachdem dieser in "main.c" gestartet wurde)
void ai_Task(void * arg) {
	//... lokale Vars, init
	//bool transmitProcessingTimePendingFlag = 0;
	//unsigned char distanceRequesterID;
	systemWaitStart();
	//ai_dwm1000Init((DeckInfo*)0);
	if (ai_init == 0){
		initAi_Swarm();

		/*const DeckDriver * uwbDriver = deckFindDriverByName("bcDWM1000");
		uwbDriver->init((DeckInfo*)NULL);*/
		//testMessage.messageType = DISTANCE_REQUEST;
		//dwm1000_SendData(&testMessage);
		ai_init = 1;
	}

	//ai_showDistance(0.5f);
	while (1) {
			/*testMessage.messageType = DISTANCE_REQUEST;
			testMessage.senderID = 12;
			testMessage.targetID = 5;
			dwm1000_SendData(&testMessage);
			vTaskDelay(M2T(500));*/
		//... repetetives

		/*if (DWM1000_IRQ_FLAG){
			nop();
			DWM1000_IRQ_FLAG = 0;
			e_interrupt_type_t intType = dwm1000_EvalInterrupt();
			testMessage.messageType = UNDEFINED;
			dwm1000_ReceiveData(&testMessage);
			//irqCounterLog = DWM1000_IRQ_Counter;
			//logMSGType = testMessage.messageType;
			nop();
		}*/

		if(FiFo_availible(&fifo) == 1) {
			// receiveHandler();
		}
		if (DWM1000_IRQ_FLAG){		//"ISR"
			/*
			DWM1000_IRQ_FLAG = false;
			e_interrupt_type_t interruptType = dwm1000_EvalInterrupt();

			switch (interruptType) {
				case RX_DONE:
					break;
				case TX_DONE:
					transmitDoneHandler();
					break;
				case FAILED_EVAL:
					break;
				default:
					//Unhandled messaging errors
					break;
			}*/
		}
		/*; i < NR_OF_DRONES; i++){
			if (i == AI_NAME){	//nicht zu mir selbst rangen
				continue;
			}


			if (rangingState[i].distanceRequested == TRUE)
				dwm1000_immediateDistanceAnswer(i);

			//dwTime_t timeSinceRanging = time.now - lastRanging[i];		//Zeit bestimmen, seid der Entfernung zu dieser Drohne das letzte mal bestimmt wurde
			//if (timeSinceRanging >= 1/RANGING_FREQUENCY){
			//	startRanging(i);											//falls diese über Schwellenwert --> neu Rangen
			//}
			if (ai_taskTicks - rangingState[i].lastRangingAi_Ticks >= 10*TASK_FREQUENCY)		//10s seit letztem Rangingvorgang abgelaufen --> erneut Starten, da Fehler erwartet
				resetRangingStateRequestee(i);

			if (!PASSIVE_MODE && !rangingState[i].distanceRequested & !rangingState[i].requestTransmitTimestampPending & !rangingState[i].processingTimePending){
				startRanging(i);
			}
		}*/
		/*
		if (!PASSIVE_MODE)
			ai_showDistance(rangingState[1].distance);*/

		vTaskDelay(M2T(1000/TASK_FREQUENCY));

	}
	vTaskDelete(0);
}



//wird in initphase von main in main.c aufgerufen, bevor scheduler gestartet wird
void ai_launch(void)
{
	xTaskCreate(ai_Task, AI_TASK_NAME,
		AI_TASK_STACKSIZE, 0,
		AI_TASK_PRIO, 0);
}

