#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include "ai_datatypes.h"

void ai_launch(void);
void DWM1000_IRQ_ISR(void);

bool DMW1000_IRQ_Flag;

bool initAi_Swarm();

void calculatePosition(st_distances_t * data);

void getDistances(st_distances_t * data);

void receiveHandler();
