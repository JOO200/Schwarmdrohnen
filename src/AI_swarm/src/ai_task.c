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
//wird ausgef�hrt von FreeRTOS-Scheduler sobald dieser das f�r sinnvoll h�lt (und nat�rlich,nachdem dieser in "main.c" gestartet wurde)
void ai_Task(void * arg) {
	//... lokale Vars, init

	initAi_Swarm();

	while (1) {
		//... repetetives
	}
	vTaskDelete(NULL); //w�re schlecht, wenn das hier aufgerufen wird...
}

bool initAi_Swarm() {
	//Rolle eigendlich geringster Name im Netzwerk --> erst Netzwerk n�tig
	if (UWB_NAME == 0) {
		my_ai_role = MASTER;
	}
	else
	{
		my_ai_role = SLAVE;
	}

	//UWB_Deck

	//...
}


