#include "ai_task.h"
#include "ai_datatypes.h"
#include "ai_distance.h"
#include "task.h"
#include "../Deck_Header/ai_dwm1000_driver.h"
#include "../config.h"

#include "worker.h"

/* FreeRtos includes */
#include "FreeRTOS.h"



//hier unsere main
//wird ausgefuehrt von FreeRTOS-Scheduler sobald dieser das fuer sinnvoll haelt (und natuerlich,nachdem dieser in "main.c" gestartet wurde)
void ai_Task(void * arg) {
	//... lokale Vars, init
	bool DMW1000_IRQ_Flag = 0;
	bool transmitProcessingTimePendingFlag = 0;
	unsigned char distanceRequesterID;

	initAi_Swarm();

	if (my_ai_role == AI_SLAVE) {
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
	vTaskDelete(0); //waere schlecht, wenn das hier aufgerufen wird...
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


	//Rolle eigendlich geringster Name im Netzwerk --> erst Netzwerk noetig
	if (UWB_NAME == 0) {
		my_ai_role = AI_MASTER;
	}
	else
	{
		my_ai_role = AI_SLAVE;
	}

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
