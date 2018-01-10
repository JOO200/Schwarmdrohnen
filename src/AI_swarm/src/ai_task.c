#include "ai_task.h"
#include "ai_datatypes.h"


#include "worker.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"



//wird in initphase von main in main.c aufgerufen, bevor scheduler gestartet wird
void ai_launch(void)
{
	xTaskCreate(ai_Task, AI_TASK_NAME,		
		AI_TASK_STACKSIZE, NULL,
		AI_TASK_PRIO, NULL);
}

//hier unsere main
//wird ausgeführt von FreeRTOS-Scheduler sobald dieser das für sinnvoll hält (und natürlich,nachdem dieser in "main.c" gestartet wurde)
void ai_Task(void * arg) {
	//... lokale Vars, init
	st_distances_t tableDistances;

	
	initAi_Swarm();

	if (my_ai_role == SLAVE) {
		//slave inits
	}

	while (1) {
		//... repetetives
	
		if (my_ai_role == SLAVE) {
			//distanzen aktualisieren
			workerSchedule(getDistances, &tableDistances);
			
			//position neu berechnen
			workerSchedule(calculatePosition, &tableDistances);
		}	
		else
		{
			//pennen
			vTaskDelete(NULL); //braucht Taskt nicht^^ - zumindes vor on the fly änderungen
		}
	}
	vTaskDelete(NULL); //wäre schlecht, wenn das hier aufgerufen wird...
}

//eventuell müssen args als void *
void getDistances(st_distances_t * data) {
	//hier call der deckinterface.distances 
	FillDistanceTable();
}


//eventuell müssen args als void *
void calculatePosition(tableDistances * data)
{
	//hier call der positionsberechnung
}

bool initAi_Swarm() {
	//UWB_Deck für Josy und Janüüüühk
	//hier euer init-shizzle


	//Rolle eigendlich geringster Name im Netzwerk --> erst Netzwerk nötig
	if (UWB_NAME == 0) {
		my_ai_role = MASTER;
	}
	else
	{
		my_ai_role = SLAVE;
	}

	

	//...
}


