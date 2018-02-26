#include "ai_task.h"
#include "ai_datatypes.h"
#include "ai_distance.h"

#include "worker.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

bool DMW1000_IRQ_Flag;

//wird in initphase von main in main.c aufgerufen, bevor scheduler gestartet wird
void ai_launch(void)
{
	xTaskCreate(ai_Task, AI_TASK_NAME,		
		AI_TASK_STACKSIZE, NULL,
		AI_TASK_PRIO, NULL);
}


//hier unsere main
//wird ausgefuehrt von FreeRTOS-Scheduler sobald dieser das fuer sinnvoll haelt (und natuerlich,nachdem dieser in "main.c" gestartet wurde)
void ai_Task(void * arg) {
	//... lokale Vars, init
	st_distances_t tableDistances;

	initAi_Swarm();

	if (my_ai_role == SLAVE) {
		//slave inits
        // @ai_motors.c : Deaktiviere evtl den Task STABILIZER
	}

	while (1) {
		//... repetetives

		if (DMW1000_IRQ_Flag){		//"ISR"
			DMW1000_IRQ_Flag = false;
			//1. Interrupt Register anschauen
			//2. Evaluieren
			enum e_interrupt_type_t interruptType = dwm1000_EvalInterrupt();

			//3. Entsprechende Funktion aufrufen
			switch (interruptType)
			{
			case RX_DONE:
				receiveHandler();
				break;
			case TX_DONE:
				if (transmitProcessingTimePendingFlag) {
					transmitProcessingTime();
				}
				break;
			default:
				break;
			}
		}

	}
	vTaskDelete(NULL); //waere schlecht, wenn das hier aufgerufen wird...
}

void receiveHandler() {
	
	st_message_t message;
	dwm1000_ReceiveData(&message);
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
	FillDistanceTable(); 
}


//eventuell muessen args als void *
void calculatePosition(tableDistances * data)
{
	//hier call der positionsberechnung
}

bool initAi_Swarm() {
	//UWB_Deck fuer Josy und Janik (neuerdings auch Nico)
	//hier euer/unser init-shizzle


	//Rolle eigendlich geringster Name im Netzwerk --> erst Netzwerk noetig
	if (UWB_NAME == 0) {
		my_ai_role = MASTER;
	}
	else
	{
		my_ai_role = SLAVE;
	}

	//...
}

