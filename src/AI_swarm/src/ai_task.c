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

//Rolle der Drohne
enum e_role_t {
	AI_ROLE_NO_ROLE = 0,
	AI_ROLE_MASTER,
	AI_ROLE_SLAVE
}; // e_role_t;

//hier unsere main
//wird ausgef�hrt von FreeRTOS-Scheduler sobald dieser das f�r sinnvoll h�lt (und nat�rlich,nachdem dieser in "main.c" gestartet wurde)
void ai_Task(void * arg) {
	//... lokale Vars, init
	st_distances_t tableDistances;

	initAi_Swarm();

	if (my_ai_role == SLAVE) {
		//slave inits
	}

	while (1) {
		//... repetetives

		if (DWM1000_IRQ_Flag){
			DWM1000_IRQ_Flag = false;
			//1. Interrupt Register anschauen
			//2. Evaluieren
			//3. Entsprechende Funktion aufrufen
		}

	}
	vTaskDelete(NULL); //w�re schlecht, wenn das hier aufgerufen wird...
}

//eventuell m�ssen args als void *
void getDistances(st_distances_t * data) {
	//hier call der deckinterface.distances 
	FillDistanceTable(); 
}


//eventuell m�ssen args als void *
void calculatePosition(tableDistances * data)
{
	//hier call der positionsberechnung
}

bool initAi_Swarm() {
	//UWB_Deck f�r Josy und Jan����hk (neuerdings auch N���kk�h)
	//hier euer/unser init-shizzle


	//Rolle eigendlich geringster Name im Netzwerk --> erst Netzwerk n�tig
	if (UWB_NAME == 0) {
		my_ai_role = MASTER;
	}
	else
	{
		my_ai_role = SLAVE;
	}

	//...
}

//isr des externen Interrupts von dwm1000
void DWM1000_IRQ_ISR(void) {

	DWM1000_IRQ_Flag = true;

}
