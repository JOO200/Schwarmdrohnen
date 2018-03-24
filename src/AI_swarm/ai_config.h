#include "ai_datatypes.h"
//#include "task.h"
#define NR_OF_DRONES 4

//Name der Drohne bei jder Drohne unterschiedlich defined
#define AI_NAME 0 // 0;1;2;3;...
#define AI_ROLE AI_MASTER

#define MAX_HISTORY 0

#define TASK_FREQUENCY 100 //Frequenz in Hz


//Taskconfig
#define AI_TASK_NAME "ai_task"
#define AI_TASK_STACKSIZE 1024  //scheint gut genug zu sein... nach test durch FreeRTOS Funktion
#define AI_TASK_PRIO 2			//hier einfaches aber evtl. schlechtes tuning m�glich - Maximum 6, je h�her desto wichtiger (idle hat 0), task der l�uft hat immer die h�chste prio der aktivierbaren tasks
 
#define PASSIVE_MODE 0			//hier 1 falls drohne nur auf rangen antworten soll

#define TRUE 1
#define FALSE 0
#define true 1
#define false 0
