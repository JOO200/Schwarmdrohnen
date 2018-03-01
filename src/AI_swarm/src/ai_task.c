#include "ai_task.h"
#include "ai_datatypes.h"
#include "ai_distance.h"
#include "task.h"
#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../ai_config.h"

#include "worker.h"

/* FreeRtos includes */
#include "FreeRTOS.h"

bool ai_init = 0;
st_message_t testMessage;
bool DMW1000_IRQ_Flag = 0;



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

	/*if (AI_ROLE == AI_SLAVE) {
		//slave inits
        // @ai_motors.c : Deaktiviere evtl den Task STABILIZER
	}

	while (1) {
		//... repetetives

		if (DMW1000_IRQ_Flag){		//"ISR"
			DMW1000_IRQ_Flag = false;
			//1. Interrupt Register anschauen
			//2. Evaluieren
			e_interrupt_type_t interruptType = dwm1000_EvalInterrupt();

			//3. Entsprechende Funktion aufrufen
			switch (interruptType)
			{
			case RX_DONE:
				receiveHandler();
				break;
			case TX_DONE:
				if (transmitProcessingTimePendingFlag) {
					dwm1000_sendProcessingTime(distanceRequesterID);
				}
				break;
			default:
				break;
			}
		}

	}
	vTaskDelete(0); //waere schlecht, wenn das hier aufgerufen wird...*/
}

void receiveHandler(unsigned char *distanceRequesterID) {
	
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
		*distanceRequesterID = message.senderID;
		//1. Immediate Answer raussenden
		//2. danach Processing Time nachsenden
		break;
	default:
		break;
	}
}

//eventuell muessen args als void *
void getDistances(st_distances_t * data) {
	//hier call der deckinterface.distances 

}


//eventuell muessen args als void *
void calculatePosition(st_distances_t * data)
{
	//hier call der positionsberechnung
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
