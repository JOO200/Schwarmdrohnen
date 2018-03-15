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
unsigned char lastMessageTarget;




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
		rangingState[message.senderID].distanceRequested = TRUE;
		//1. Immediate Answer raussenden
		st_message_t immediateAnswer;
		immediateAnswer.senderID = AI_NAME;
		immediateAnswer.targetID = immediateAnswer.senderID;
		immediateAnswer.messageType = IMMEDIATE_ANSWER;
		dwm1000_SendData(&immediateAnswer);
		//2. danach Processing Time nachsenden (pending flag setzen)
		rangingState[message.senderID].transmitProcessingTimePendingFlag = TRUE;
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

		//Zeitberechnung in ai_lpsTwrTag Zeile 172-202
		break;
	default:
		break;
	}
}

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

//eventuell muessen args als void *
void startRanging(unsigned char targetID) {
	//requestMessage senden
	dwm1000_requestDistance(targetID);

	//flags und vars für Momentanzustand setzen
	rangingState[targetID].requestTransmitTimestampPending = TRUE;
	rangingState[targetID].immediateAnswerPending = TRUE;

	lastMessageType = DISTANCE_REQUEST;
	lastMessageTarget = targetID;
}


bool initAi_Swarm() {
	//UWB_Deck fuer Josy und Janik ((((neuerdings))) auch Nico)
	//hier euer/unser init-shizzlel
	setup_dwm1000_communication();		//HW-Setup


	//...
	return TRUE;
}

//hier unsere main
//wird ausgefuehrt von FreeRTOS-Scheduler sobald dieser das fuer sinnvoll haelt (und natuerlich, nachdem dieser in "main.c" gestartet wurde)
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
			DMW1000_IRQ_Flag = FALSE;
			e_interrupt_type_t interruptType = dwm1000_EvalInterrupt();

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

		for (unsigned char i = 0; i < NR_OF_DRONES; i++){
			if (i == AI_NAME){	//nicht zu mir selbst rangen
				continue;
			}


			//ai_showDistance(rangingState[i].distance);

			if (rangingState[i].distanceRequested == TRUE)
				dwm1000_immediateDistanceAnswer(i);

			/*dwTime_t timeSinceRanging = time.now - lastRanging[i];		//Zeit bestimmen, seid der Entfernung zu dieser Drohne das letzte mal bestimmt wurde
			if (timeSinceRanging >= 1/RANGING_FREQUENCY){
				startRanging(i);											//falls diese über Schwellenwert --> neu Rangen
			}*/
			if (!PASSIVE_MODE && !rangingState[i].distanceRequested & !rangingState[i].requestTransmitTimestampPending & !rangingState[i].processingTimePending){
				startRanging(i);
			}
		}
		vTaskDelay(5000);

	}
	vTaskDelete(0); //waere schlecht, wenn das hier aufgerufen wird...*/
}


//wird in initphase von main in main.c aufgerufen, bevor scheduler gestartet wird
void ai_launch(void)
{
	xTaskCreate(ai_Task, AI_TASK_NAME,
		AI_TASK_STACKSIZE, NULL,
		AI_TASK_PRIO, NULL);
}
