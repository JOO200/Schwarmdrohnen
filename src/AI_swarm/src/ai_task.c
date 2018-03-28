#include "ai_task.h"
#include "ai_datatypes.h"
#include "ai_distance.h"
#include "task.h"
#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../ai_config.h"
#include "ledring12.h"
#include "ai_manageInterrupts.h"

#include "worker.h"

/* FreeRtos includes */
#include "FreeRTOS.h"

uint64_t ai_taskTicks;



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
		//dwm1000_sendProcessingTime(lastMessageTarget);
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
	rangingState[targetID].lastRanginInAITicks = ai_taskTicks;//_/			//awaiting sent interrupt

	resetRangingStateRequest(targetID);

	dwm1000_requestDistance(targetID);

	//flags und vars für Momentanzustand setzen
	rangingState[targetID].requestTransmitTimestampPending = TRUE;
	rangingState[targetID].immediateAnswerPending = TRUE;

	lastMessageType = DISTANCE_REQUEST;
	lastMessageTarget = targetID;
}*/

void resetRequestee(unsigned char targetID){
	rangingState[targetID].requestTxTimestamp.full = 0;
	rangingState[targetID].immediateAnswerRxTimestamp.full = 0;
	rangingState[targetID].tRound.full = 0;
	rangingState[targetID].rangingDuration = 0;
}

void resetTarget(unsigned char requesteeID){
	rangingState[requesteeID].requestTxTimestamp.full = 0;
	rangingState[requesteeID].immediateAnswerRxTimestamp.full = 0;
	rangingState[requesteeID].tRound.full = 0;
}


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

		const DeckDriver * uwbDriver = deckFindDriverByName("bcDWM1000");
		uwbDriver->init((DeckInfo*)NULL);
		//testMessage.messageType = DISTANCE_REQUEST;
		//dwm1000_SendData(&testMessage);
		ai_init = 1;
	}

	//ai_showDistance(0.5f);
	while (1) {
		for (unsigned char i = 0; i < NR_OF_DRONES; i++){
			if (i == AI_NAME)
				continue;

			//Statemachine für jeden anderen Teilnehmer durchgehen
			//Statemachine, bei der diese Drohne aktiv/requestee ist
			if (rangingState[i].rangingDuration > AI_TASKTICKS_TO_NEW_RANGING) {
				rangingState[i].requesteeState = REQ_STATE_IDLE;
			}
			switch (rangingState[i].requesteeState) {
				case REQ_STATE_IDLE:				//kein raging Prozess als requestee mit i im Gang
					if ((ai_taskTicks - rangingState[i].lastRanginInAITicks) >= AI_TASKTICKS_TO_NEW_RANGING){
						resetRequestee(i);
						initiateAiRanging(i);
						rangingState[i].requesteeState = REQ_STATE_REQUESTED;
					}
					rangingState[i].rangingDuration++;
					break;

				case REQ_STATE_REQUESTED:			//TxTimestamp von DistanceRequest lesen und abspeichern

					if (lookForRequestTxTs(i)){
						dwTime_t reqTx = getReqTxTs(i);
						rangingState[i].requestTxTimestamp = reqTx;

						rangingState[i].requesteeState = REQ_STATE_IMMANSWERRECEIVE;
					}
					rangingState[i].rangingDuration++;
					break;

				case REQ_STATE_IMMANSWERRECEIVE:	// Immediate Answer RxTimestamp speichern
					if (lookForImmedatateAnswerRxTs(i)){
						dwTime_t ImAnRx = getImmediateAnswerRxTs(i);
						rangingState[i].immediateAnswerTxTimestamp = ImAnRx;
						rangingState[i].requesteeState = REQ_STATE_IMMANSWERACK;
					}
					rangingState[i].rangingDuration++;
					break;

				case REQ_STATE_IMMANSWERACK: ;
					st_message_t ack;
					ack.senderID = AI_NAME;
					ack.targetID = i;
					ack.messageType = IMMEDIATE_ANSWER_ACK;
					dwm1000_SendData(&ack);
					break;
				case REQ_STATE_CALCTROUND:			//Tround berechnen und in rangingState eintragen
					rangingState[i].tRound.full = rangingState[i].immediateAnswerRxTimestamp.full - rangingState[i].requestTxTimestamp.full;
					rangingState[i].requesteeState = REQ_STATE_CALCDIST;
					rangingState[i].rangingDuration++;
					break;

				case REQ_STATE_CALCDIST:			//Processing Time empfangen und Distanz berechnen --> dann Sprung auf IDLE
					if (lookForProcessingTime(i)){
						dwTime_t procTime = getProcessingTime(i);
						double tdistDWM, troundDWM, tprocessingDWM, tdist;
						troundDWM = rangingState[i].immediateAnswerRxTimestamp.low32 - rangingState[i].requestTxTimestamp.low32;
						tprocessingDWM = procTime.low32;		//da aktuelle message processing time schickt ist diese hier zu entnehmen
						tdistDWM = 0.5*(troundDWM - tprocessingDWM);
						tdist = tdistDWM / LOCODECK_TS_FREQ;		//Umrechnung von DWM1000 Zeiteinheit in Sekunden
						rangingState[i].distance = SPEED_OF_LIGHT * tdist;
						rangingState[i].requesteeState = REQ_STATE_IDLE;

						rangingState[i].lastRanginInAITicks = rangingState[i].rangingDuration;
					}
					rangingState[i].requesteeState = REQ_STATE_IDLE;
					rangingState[i].rangingDuration++;
					break;

				default:
					rangingState[i].requesteeState = REQ_STATE_IDLE;
					break;
			}

			//Statemachine, bei der diese Drohne passiv/target ist
			switch (rangingState[i].targetState) {
				case TARGET_STATE_IDLE:				//von i kein Ranging angefordert
					if (lookForImmedatateAnswerRxTs(i)){
						resetTarget(i);
						rangingState[i].immediateAnswerRxTimestamp = getImmediateAnswerRxTs(i);
						rangingState[i].targetState = TARGET_STATE_DISTREQUESTED;
					}
					break;

				case TARGET_STATE_DISTREQUESTED: ;
					st_message_t immediateAnswerMSG;
					immediateAnswerMSG.messageType = IMMEDIATE_ANSWER;
					immediateAnswerMSG.senderID = AI_NAME;
					immediateAnswerMSG.targetID = i;
					dwm1000_SendData(&immediateAnswerMSG);
					rangingState[i].targetState = TARGET_STATE_GETIMMANSWERTS;
					break;

				case TARGET_STATE_GETIMMANSWERTS:
					if(lookForImmediatateAnswerTxTs(i)) {
						rangingState[i].targetState = TARGET_STATE_READYPROCESSING;
						rangingState[i].immediateAnswerTxTimestamp = getImmediateAnswerTxTs(i);
					}
					break;

				case TARGET_STATE_READYPROCESSING:
					if (lookForAckAnswerTxTs(i)){
						st_message_t processingTime;
						processingTime.messageType = PROCESSING_TIME;
						processingTime.senderID = AI_NAME;
						processingTime.targetID = i;
						processingTime.time = rangingState[i].immediateAnswerTxTimestamp;
						dwm1000_SendData(&processingTime);
						rangingState[i].targetState = TARGET_STATE_IDLE;
					}
					break;

				default:
					rangingState[i].targetState = TARGET_STATE_IDLE;
					break;
			}
		}
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

