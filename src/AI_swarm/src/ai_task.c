#include "ai_launch.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#define AI_TASK_NAME "ai_task"
#define AI_TASK_STACKSIZE 1024  //kp was genug/zu viel - MIST
#define AI_TASK_PRIO 40			//hier einfaches aber schlechtes tuning möglich - MIST


void ai_launch(void)
{
	xTaskCreate(ai_Task, AI_TASK_NAME,		
		AI_TASK_STACKSIZE, NULL,
		AI_TASK_PRIO, NULL);
}

//hier unsere main
void ai_Task(void * arg) {


	while (1) {
		//...
	}
}


