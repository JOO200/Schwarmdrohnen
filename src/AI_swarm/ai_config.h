#define MAX_DRONES 7

//Name der Drohne bei jder Drohne unterschiedlich defined
#define UWB_NAME 0 //oder 1;2;3;...

#define MAX_HISTORY 4

//Taskconfig
#define AI_TASK_NAME "ai_task"
#define AI_TASK_STACKSIZE 1024  //kp was genug/zu viel - MIST
#define AI_TASK_PRIO 2			//hier einfaches aber evtl. schlechtes tuning m�glich - Maximum 6, je h�her desto wichtiger (idle hat 0), task der l�uft hat immer die h�chste prio der aktivierbaren tasks
