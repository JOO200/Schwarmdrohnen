#include "ai_task.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#define AI_TASK_NAME "ai_task"
#define AI_TASK_STACKSIZE 1024  //kp was genug/zu viel - MIST
#define AI_TASK_PRIO 40			//hier einfaches aber evtl. schlechtes tuning m�glich - MIST


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
	//... lokale Vars, function calls

	while (1) {
		//... repetetives
	}
}


