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

bool ai_init = 0;
st_message_t testMessage;
bool DMW1000_IRQ_Flag = 0;

//Ranging Flags, state and Distance:
st_rangingState_t rangingState[NR_OF_DRONES];

e_message_type_t lastMessageType;
char lastMessageTarget;


//hier unsere main
//wird ausgefuehrt von FreeRTOS-Scheduler sobald dieser das fuer sinnvoll haelt (und natuerlich,nachdem dieser in "main.c" gestartet wurde)
void ai_Task(void * arg) {
	//... lokale Vars, init
	//bool transmitProcessingTimePendingFlag = 0;
	//unsigned char distanceRequesterID;
	if (ai_init == 0){
		initAi_Swarm();
		testMessage.messageType = MASTER_STATE;
		ai_init = 1;
	}
	while(1){
		//TickType_t delay = 3000000;//portTICK_PERIOD_MS;	//ermöglicht Eingabe der Zeit in ms - test ---> NOPE IST NET KONFIGURIERT...
		//vTaskDelay(delay);	//Einheit: ca. 6 Nano Sekunden 3.000.000 --> ca. 18ms
		dwm1000_SendData(&testMessage);
	}

	if (AI_ROLE == AI_SLAVE) {
		//slave inits
        // @ai_motors.c : Deaktiviere evtl den Task STABILIZER
	}

	while (1) {
		//... repetetives

		if (DMW1000_IRQ_Flag){		//"ISR"
			DMW1000_IRQ_Flag = false;
			e_interrupt_type_t interruptType = dwm1000_handlInterrupt();

			switch (interruptType) {
				case RX_DONE:
					receiveHandler();
					break;
				case TX_DONE:
					transmitDoneHandler();
					break;
				default:
					//Unhandled messaging errors
					break;
			}
		}

		for (char i = 0; i < NR_OF_DRONES; i++){
			if (i = AI_NAME)	//nicht zu mir selbst rangen
				continue;


			ai_showDistance(rangingState[i].distance);

			if (distanceRequested[i])
				dwm1000_immediateDistanceAnswer(i);

			/*dwTime_t timeSinceRanging = time.now - lastRanging[i];		//Zeit bestimmen, seid der Entfernung zu dieser Drohne das letzte mal bestimmt wurde
			if (timeSinceRanging >= 1/RANGING_FREQUENCY){
				startRanging(i);											//falls diese über Schwellenwert --> neu Rangen
			}*/
			if (!PASSIVE_MODE && !rangingState[i].distanceRequested[i] & !rangingState[i].requestTransmitTimestampPending[i] & !rangingState[i].processingTimePending[i] & !rangingState[i].immediateAnswerTransmitTimestampPending[i]){
				startRanging(i);
				rangingState[i].requestTransmitTimestampPending = true;
				rangingState[i].immediateAnswerPending = true;
			}
		}
		vTaskDelay(5000);

	}
	vTaskDelete(0); //waere schlecht, wenn das hier aufgerufen wird...*/
}

void receiveHandler() {
	
	st_message_t message;
	dwm1000_ReceiveData(&message);
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
		rangingState[message.senderID].distanceRequested = 1;
		//1. Immediate Answer raussenden
		st_message_t immediateAnswer;
		immediateAnswer.senderID = AI_NAME;
		immediateAnswer.targetID = immediateAnswer.senderID
		immediateAnswer.messageType = IMMEDIATE_ANSWER;
		dwm1000_SendData(&immediateAnswer);
		//2. danach Processing Time nachsenden (pending flag setzen)
		rangingState[message.senderID].transmitProcessingTimePendingFlag = true;
		break;
	case IMMEDIATE_ANSWER:
		//Tround berechnen
		if (!rangingState[message.senderID].immediateAnswerPending)
			break;
		rangingState[message.senderID].immediateAnswerRxTimestamp = dwm1000_getRxTimestamp();
		rangingState[message.senderID].tRound = rangingState[message.senderID].immediateAnswerRxTimestamp.full - rangingState[message.senderID].requestTxTimestamp.full
		rangingState[message.senderID].immediateAnswerPending = false;
		break;
	case PROCESSING_TIME:
		//Distanz berechnen und eintragen
		//Rangin-Struct leeren


		//Zeitberechnung in ai_lpsTwrTag Zeile 172-202
		break
	default:
		break;
	}
}

void transmitDoneHandler(){
	if (rangingState[lastMessageTarget].transmitProcessingTimePendingFlag) {
		dwm1000_sendProcessingTime(lastMessageTarget);
	}
	else if (rangingState[lastMessageTarget].requestTransmitTimestampPending){
		rangingState[lastMessageTarget].requestTxTimestamp = dwm1000_getTxTimestamp();
	}
	else
		return;
}

//eventuell muessen args als void *
void startRanging(char targetID) {
	//requestMessage senden
	dwm1000_requestDistance(targetID);

	//flags und vars für Momentanzustand setzen
	distanceRequested[targetID] = 1;
	requestTransmitTimestampPending[targetID] = 1;

	lastMessageType = DISTANCE_REQUEST;
	lastMessageTarget = targetID;
}


bool initAi_Swarm() {
	//UWB_Deck fuer Josy und Janik (neuerdings auch Nico)
	//hier euer/unser init-shizzle
	setup_dwm1000_communication();		//HW-Setup

	dwm1000_init();



	//...
	return 1;
}


//wird in initphase von main in main.c aufgerufen, bevor scheduler gestartet wird
void ai_launch(void)
{
	xTaskCreate(ai_Task, AI_TASK_NAME,
		AI_TASK_STACKSIZE, NULL,
		AI_TASK_PRIO, NULL);
}
